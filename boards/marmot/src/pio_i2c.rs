//! PIO-backed async I2C controller driver based on Raspberry Pi's `i2c.pio` and `rp-rs/i2c-pio-rs`.
//! Supports 4 state machines per PIO block with arbitrary SDA/SCL pin assignments and 4-word FIFO pipelining.

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::i2c::{AbortReason, Error};
use embassy_rp::pio::program::{self as pio, pio_asm};
use embassy_rp::pio::{
    Common, Config, Direction, Instance, Irq, IrqFlags, LoadedProgram, Pin, PioPin, ShiftDirection,
    StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;

const ICOUNT_LSB: u16 = 10;
const DATA_LSB: u16 = 2;
const NAK_LSB: u16 = 1;
const FINAL_LSB: u16 = 0;

// Set pin direction / side-set instruction table (`set_scl_sda` from `i2c.pio`):
// .side_set 1 opt pindirs
// set pindirs, 0 side 0 [7] ; SCL = 0, SDA = 0
// set pindirs, 1 side 0 [7] ; SCL = 0, SDA = 1
// set pindirs, 0 side 1 [7] ; SCL = 1, SDA = 0
// set pindirs, 1 side 1 [7] ; SCL = 1, SDA = 1
const I2C_SC0_SD0: u16 = 0b111_1_0_111_100_00000;
const I2C_SC0_SD1: u16 = 0b111_1_0_111_100_00001;
const I2C_SC1_SD0: u16 = 0b111_1_1_111_100_00000;
const I2C_SC1_SD1: u16 = 0b111_1_1_111_100_00001;

// `in null, 8` (used at the end of STOP to push a completion token into RX FIFO once STOP finishes)
const IN_NULL_8: u16 =
    pio::InstructionOperands::IN { source: pio::InSource::NULL, bit_count: 8 }.encode();

/// Loaded PIO I2C program in PIO instruction memory (21 instructions).
pub struct PioI2cProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioI2cProgram<'d, PIO> {
    /// Load the shared I2C program into the given PIO block.
    ///
    /// Maps `IN_BASE = SCL` (`wait 1 pin 0`) and `JMP_PIN = SDA` (`jmp pin`),
    /// allowing all 4 state machines (`sm0`..`sm3`) on the PIO block to share this single
    /// 21-instruction program with independent, arbitrary SDA and SCL pins.
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            r#"
            .side_set 1 opt pindirs

            do_nack:
                jmp y-- entry_point        ; Continue if NAK was expected
                irq wait 0 rel             ; Otherwise stop, ask for help

            do_byte:
                set x, 7                   ; Loop 8 times
                set y, 1                   ; Constant 1 for shifting high SDA bits into ISR
            bitloop:
                out pindirs, 1         [7] ; Serialise write data (all-ones if reading)
                nop             side 1 [2] ; SCL rising edge
                wait 1 pin 0           [4] ; Allow clock to be stretched (pin 0 = IN_BASE = SCL)
                jmp pin sda_high           ; Sample SDA via JMP_PIN (1 cycle)
                in null, 1             [5] ; SDA == 0: shift 0 into ISR (6 cycles)
                jmp bit_next               ; (1 cycle -> 8 cycles total for sample phase)
            sda_high:
                in y, 1                [6] ; SDA == 1: shift 1 into ISR (7 cycles -> 8 cycles total)
            bit_next:
                jmp x-- bitloop side 0 [7] ; SCL falling edge

                ; Handle ACK pulse
                out pindirs, 1         [7] ; On reads, we provide the ACK
                out y, 1        side 1 [7] ; Unpack FINAL (NAK ignore) bit into Y & raise SCL
                wait 1 pin 0           [7] ; Allow clock to be stretched (pin 0 = IN_BASE = SCL)
                jmp pin do_nack side 0 [2] ; Test SDA (JMP_PIN) for ACK/NAK, fall through if ACK

            public entry_point:
            .wrap_target
                out x, 6                   ; Unpack Instr count
                jmp !x do_byte             ; Instr == 0, this is a data record
                out null, 32               ; Instr > 0, remainder of this OSR is invalid
            do_exec:
                out exec, 16               ; Execute one instruction per FIFO word
                jmp x-- do_exec            ; Repeat n + 1 times
            .wrap
            "#
        );

        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Async PIO-backed I2C Master driver.
pub struct PioI2c<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    irq: Irq<'d, PIO, SM>,
    irq_flags: IrqFlags<'d, PIO>,
    _sda_pin: Pin<'d, PIO>,
    _scl_pin: Pin<'d, PIO>,
    wrap_target: u8,
}

impl<'d, PIO: Instance, const SM: usize> PioI2c<'d, PIO, SM> {
    /// Create and initialize a new PIO I2C master instance sharing `program`.
    ///
    /// `sda` and `scl` can be any distinct PIO pins in any order.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        irq: Irq<'d, PIO, SM>,
        irq_flags: IrqFlags<'d, PIO>,
        program: &PioI2cProgram<'d, PIO>,
        sda: Peri<'d, impl PioPin + 'd>,
        scl: Peri<'d, impl PioPin + 'd>,
        freq_hz: u32,
    ) -> Self {
        let mut sda_pin = common.make_pio_pin(sda);
        let mut scl_pin = common.make_pio_pin(scl);

        assert_ne!(sda_pin.pin(), scl_pin.pin(), "SDA and SCL must be distinct pins");

        sda_pin.set_pull(Pull::Up);
        scl_pin.set_pull(Pull::Up);

        let wrap_target = program.prg.wrap.target;

        let mut cfg = Config::default();
        // Side-set controls SCL pindirs
        cfg.use_program(&program.prg, &[&scl_pin]);
        // OUT and SET control SDA pindirs
        cfg.set_out_pins(&[&sda_pin]);
        cfg.set_set_pins(&[&sda_pin]);
        // IN_BASE is SCL so `wait 1 pin 0` monitors SCL for clock stretching
        cfg.set_in_pins(&[&scl_pin]);
        // JMP_PIN is SDA so `jmp pin` samples SDA data bits and ACK/NACK
        cfg.set_jmp_pin(&sda_pin);

        // embassy-rp 0.10.0 shifts pinctrl bases by -16 when GPIOBASE=16 (any pin >= 32),
        // but writes config.exec.jmp_pin unshifted to the 5-bit EXECCTRL.JMP_PIN field.
        if sda_pin.pin() >= 32 || scl_pin.pin() >= 32 {
            let mut exec = cfg.get_exec();
            exec.jmp_pin -= 16;
            // SAFETY: ExecConfig is updated before sm.set_config using fields from cfg.get_exec().
            unsafe { cfg.set_exec(exec) };
        }

        cfg.shift_out.auto_fill = true;
        cfg.shift_out.direction = ShiftDirection::Left;
        cfg.shift_out.threshold = 16;

        cfg.shift_in.auto_fill = true;
        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.shift_in.threshold = 8;

        // 1 bit cycle in `bitloop` is 32 PIO clock cycles.
        cfg.clock_divider = calculate_pio_clock_divider(32 * freq_hz);

        sm.set_config(&cfg);

        // Glitch-free open-drain initialization matching `i2c_program_init` in `i2c.pio`:
        // 1. Set PINS = High and PINDIRS = Out first so lines remain High prior to OE inversion.
        // 2. Enable OE inversion (`pindirs = 1` -> `OE = 0` High-Z pulled High; `pindirs = 0` -> `OE = 1`).
        // 3. Set PINS = Low so whenever `pindirs = 0` (`OE = 1`), the pad drives Low.
        sm.set_pins(Level::High, &[&sda_pin, &scl_pin]);
        sm.set_pin_dirs(Direction::Out, &[&sda_pin, &scl_pin]);
        sda_pin.set_output_enable_inversion(true);
        scl_pin.set_output_enable_inversion(true);
        sm.set_pins(Level::Low, &[&sda_pin, &scl_pin]);

        // Clear any pending state machine IRQ and jump to `entry_point` (`wrap_target`).
        irq_flags.clear(SM);
        sm.clear_fifos();
        sm.restart();
        unsafe {
            sm.exec_jmp(wrap_target);
        }
        sm.set_enable(true);

        Self {
            sm,
            irq,
            irq_flags,
            _sda_pin: sda_pin,
            _scl_pin: scl_pin,
            wrap_target,
        }
    }

    #[inline]
    fn has_errored(&self) -> bool {
        self.irq_flags.check(SM as u8)
    }

    #[inline]
    fn try_push16(&mut self, word: u16) -> bool {
        self.sm.tx().try_push((word as u32) << 16)
    }

    async fn push16_or_err(&mut self, word: u16) -> Result<(), Error> {
        if self.has_errored() {
            return Err(Error::Abort(AbortReason::NoAcknowledge));
        }
        let val = (word as u32) << 16;
        match select(self.sm.tx().wait_push(val), self.irq.wait()).await {
            Either::First(()) => {
                if self.has_errored() {
                    Err(Error::Abort(AbortReason::NoAcknowledge))
                } else {
                    Ok(())
                }
            }
            Either::Second(()) => Err(Error::Abort(AbortReason::NoAcknowledge)),
        }
    }

    async fn pull8_or_err(&mut self) -> Result<u8, Error> {
        if self.has_errored() {
            return Err(Error::Abort(AbortReason::NoAcknowledge));
        }
        match select(self.sm.rx().wait_pull(), self.irq.wait()).await {
            Either::First(v) => {
                if self.has_errored() {
                    Err(Error::Abort(AbortReason::NoAcknowledge))
                } else {
                    Ok(v as u8)
                }
            }
            Either::Second(()) => Err(Error::Abort(AbortReason::NoAcknowledge)),
        }
    }

    async fn recover_and_stop(&mut self) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.restart();
        self.irq_flags.clear(SM);
        unsafe {
            self.sm.exec_jmp(self.wrap_target);
        }
        self.sm.set_enable(true);

        // Issue a STOP sequence to release SCL and SDA back to idle high.
        let _ = self.stop().await;
    }

    async fn start(&mut self) -> Result<(), Error> {
        self.push16_or_err(1 << ICOUNT_LSB).await?;
        self.push16_or_err(I2C_SC1_SD0).await?;
        self.push16_or_err(I2C_SC0_SD0).await
    }

    async fn repstart(&mut self) -> Result<(), Error> {
        self.push16_or_err(3 << ICOUNT_LSB).await?;
        self.push16_or_err(I2C_SC0_SD1).await?;
        self.push16_or_err(I2C_SC1_SD1).await?;
        self.push16_or_err(I2C_SC1_SD0).await?;
        self.push16_or_err(I2C_SC0_SD0).await
    }

    async fn stop(&mut self) -> Result<(), Error> {
        self.push16_or_err(3 << ICOUNT_LSB).await?;
        self.push16_or_err(I2C_SC0_SD0).await?;
        self.push16_or_err(I2C_SC1_SD0).await?;
        self.push16_or_err(I2C_SC1_SD1).await?;
        self.push16_or_err(IN_NULL_8).await?;
        let _ = self.pull8_or_err().await?;
        Ok(())
    }

    /// Pipelined write: keeps up to 4 bytes in flight in the PIO FIFOs.
    async fn write_inner(&mut self, addr: u8, bytes: &[u8], send_stop: bool) -> Result<(), Error> {
        self.start().await?;

        let total = 1 + bytes.len();
        let mut tx_idx = 0usize;
        let mut rx_idx = 0usize;
        let mut in_flight = 0usize;

        while rx_idx < total {
            if self.has_errored() {
                return Err(Error::Abort(AbortReason::NoAcknowledge));
            }

            // 1. Drain any completed dummy RX bytes without yielding
            while in_flight > 0 {
                if self.sm.rx().try_pull().is_some() {
                    in_flight -= 1;
                    rx_idx += 1;
                } else {
                    break;
                }
            }

            // 2. Fill TX FIFO up to the 4-byte RX FIFO capacity limit
            while tx_idx < total && in_flight < 4 {
                let byte = if tx_idx == 0 { addr << 1 } else { bytes[tx_idx - 1] };
                let word = ((byte as u16) << DATA_LSB) | (1 << NAK_LSB);
                if self.try_push16(word) {
                    tx_idx += 1;
                    in_flight += 1;
                } else {
                    break;
                }
            }

            if rx_idx == total {
                break;
            }

            // 3. Yield only when necessary
            if in_flight > 0 {
                let _ = self.pull8_or_err().await?;
                in_flight -= 1;
                rx_idx += 1;
            } else {
                // TX FIFO is still full from START; await space for the first byte
                let byte = if tx_idx == 0 { addr << 1 } else { bytes[tx_idx - 1] };
                let word = ((byte as u16) << DATA_LSB) | (1 << NAK_LSB);
                self.push16_or_err(word).await?;
                tx_idx += 1;
                in_flight += 1;
            }
        }

        if send_stop {
            self.stop().await?;
        }
        Ok(())
    }

    /// Pipelined read: queues `ADDR+R` and up to 3 read bytes immediately, keeping 4 bytes in flight.
    async fn read_inner(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        is_repstart: bool,
    ) -> Result<(), Error> {
        if is_repstart {
            self.repstart().await?;
        } else {
            self.start().await?;
        }

        // Total bytes on the bus = 1 (address byte) + buffer.len()
        let total = 1 + buffer.len();
        let mut tx_idx = 0usize;
        let mut rx_idx = 0usize;
        let mut in_flight = 0usize;

        while rx_idx < total {
            if self.has_errored() {
                return Err(Error::Abort(AbortReason::NoAcknowledge));
            }

            // 1. Drain any completed RX bytes without yielding
            while in_flight > 0 {
                if let Some(v) = self.sm.rx().try_pull() {
                    if rx_idx > 0 {
                        buffer[rx_idx - 1] = v as u8;
                    }
                    in_flight -= 1;
                    rx_idx += 1;
                } else {
                    break;
                }
            }

            // 2. Fill TX FIFO up to 4 in-flight bytes
            while tx_idx < total && in_flight < 4 {
                let word = if tx_idx == 0 {
                    (((addr << 1) | 1) as u16) << DATA_LSB | (1 << NAK_LSB)
                } else {
                    let last = tx_idx + 1 == total;
                    (0xff << DATA_LSB) | if last { (1 << FINAL_LSB) | (1 << NAK_LSB) } else { 0 }
                };

                if self.try_push16(word) {
                    tx_idx += 1;
                    in_flight += 1;
                } else {
                    break;
                }
            }

            if rx_idx == total {
                break;
            }

            // 3. Yield only when waiting on in-flight RX data (or TX space after REPSTART)
            if in_flight > 0 {
                let v = self.pull8_or_err().await?;
                if rx_idx > 0 {
                    buffer[rx_idx - 1] = v as u8;
                }
                in_flight -= 1;
                rx_idx += 1;
            } else {
                let word = if tx_idx == 0 {
                    (((addr << 1) | 1) as u16) << DATA_LSB | (1 << NAK_LSB)
                } else {
                    let last = tx_idx + 1 == total;
                    (0xff << DATA_LSB) | if last { (1 << FINAL_LSB) | (1 << NAK_LSB) } else { 0 }
                };
                self.push16_or_err(word).await?;
                tx_idx += 1;
                in_flight += 1;
            }
        }

        self.stop().await
    }

    /// Write to I2C address cleanly with START and STOP.
    pub async fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let res = self.write_inner(addr, bytes, true).await;
        if res.is_err() {
            self.recover_and_stop().await;
        }
        res
    }

    /// Read from I2C address cleanly with START and STOP.
    pub async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let res = self.read_inner(addr, buffer, false).await;
        if res.is_err() {
            self.recover_and_stop().await;
        }
        res
    }

    /// Write to I2C address followed by Repeated START and Read, ending with STOP.
    pub async fn write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        let res = async {
            self.write_inner(addr, bytes, false).await?;
            self.read_inner(addr, buffer, true).await
        }
        .await;
        if res.is_err() {
            self.recover_and_stop().await;
        }
        res
    }
}

impl<'d, PIO: Instance, const SM: usize> embedded_hal::i2c::ErrorType for PioI2c<'d, PIO, SM> {
    type Error = Error;
}

impl<'d, PIO: Instance, const SM: usize> embedded_hal_async::i2c::I2c for PioI2c<'d, PIO, SM> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        PioI2c::read(self, address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        PioI2c::write(self, address, write).await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        PioI2c::write_read(self, address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal::i2c::Operation::Read(buf) => self.read(address, buf).await?,
                embedded_hal::i2c::Operation::Write(buf) => self.write(address, buf).await?,
            }
        }
        Ok(())
    }
}
