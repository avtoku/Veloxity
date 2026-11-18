//! PIO-backed async Quad SPI (QSPI) driver for the Winbond W25Q128JVPIM (128 Mbit / 16 MB NOR Flash).
//!
//! ### Pin Allocation (GPIO 19..=24 in order)
//! - **GPIO 19**: `CS` (Chip Select, active low)
//! - **GPIO 20**: `CLK` (Serial Clock, driven via PIO side-set)
//! - **GPIO 21**: `D0` (`IO0` / DI)
//! - **GPIO 22**: `D1` (`IO1` / DO)
//! - **GPIO 23**: `D2` (`IO2` / `/WP`)
//! - **GPIO 24**: `D3` (`IO3` / `/HOLD` / `/RESET`)

use embassy_futures::join::join;
use embassy_rp::Peri;
use embassy_rp::dma::{self, Channel};
use embassy_rp::gpio::{Drive, Level, Output, Pin as GpioPin, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction, Instance, LoadedProgram, Pin, PioPin, ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::Timer;

// -----------------------------------------------------------------------------
// W25Q128JVPIM Instruction Set & Geometry Constants
// -----------------------------------------------------------------------------
pub const W25Q_PAGE_SIZE: usize = 256;
pub const W25Q_SECTOR_SIZE: usize = 4096;
pub const W25Q_BLOCK_32K_SIZE: usize = 32 * 1024;
pub const W25Q_BLOCK_64K_SIZE: usize = 64 * 1024;
pub const W25Q_CAPACITY_BYTES: usize = 16 * 1024 * 1024; // 128 Mbit = 16 MiB

pub const CMD_WRITE_ENABLE: u8 = 0x06;
pub const CMD_VOLATILE_SR_WRITE_ENABLE: u8 = 0x50;
pub const CMD_WRITE_DISABLE: u8 = 0x04;

pub const CMD_READ_SR1: u8 = 0x05;
pub const CMD_READ_SR2: u8 = 0x35;
pub const CMD_READ_SR3: u8 = 0x15;
pub const CMD_WRITE_SR1: u8 = 0x01;
pub const CMD_WRITE_SR2: u8 = 0x31;
pub const CMD_WRITE_SR3: u8 = 0x11;

pub const CMD_READ_DATA: u8 = 0x03;
pub const CMD_FAST_READ: u8 = 0x0B;
pub const CMD_FAST_READ_QUAD_OUTPUT: u8 = 0x6B;
pub const CMD_FAST_READ_QUAD_IO: u8 = 0xEB;

pub const CMD_PAGE_PROGRAM: u8 = 0x02;
pub const CMD_QUAD_PAGE_PROGRAM: u8 = 0x32;

pub const CMD_SECTOR_ERASE_4K: u8 = 0x20;
pub const CMD_BLOCK_ERASE_32K: u8 = 0x52;
pub const CMD_BLOCK_ERASE_64K: u8 = 0xD8;
pub const CMD_CHIP_ERASE: u8 = 0xC7;

pub const CMD_ENABLE_RESET: u8 = 0x66;
pub const CMD_RESET_DEVICE: u8 = 0x99;
pub const CMD_JEDEC_ID: u8 = 0x9F;
pub const CMD_READ_UNIQUE_ID: u8 = 0x4B;

/// Status Register 1 bits
pub const SR1_BUSY: u8 = 0x01;
pub const SR1_WEL: u8 = 0x02;

/// Status Register 2 bits
pub const SR2_QE: u8 = 0x02;

/// Expected Winbond Manufacturer ID
pub const WINBOND_MFG_ID: u8 = 0xEF;
/// Expected W25Q128JV capacity ID (128 Mbit)
pub const W25Q128_CAPACITY_ID: u8 = 0x18;

/// Pin direction nibble masks for `[D3, D2, D1, D0]`
const PINDIRS_SPI_1BIT: u8 = 0b1101;
const PINDIRS_QUAD_TX: u8 = 0b1111;
const PINDIRS_QUAD_RX: u8 = 0b0000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QspiError {
    InvalidJEDECId([u8; 3]),
    UnalignedAddress,
    PageBoundaryCrossed,
    LengthZero,
}

// -----------------------------------------------------------------------------
// PIO QSPI Program (7 instructions)
// -----------------------------------------------------------------------------

/// Loaded PIO QSPI program in PIO instruction memory (7 instructions).
pub struct PioQspiProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioQspiProgram<'d, PIO> {
    /// Load the 7-instruction PIO QSPI program into the given PIO block.
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            r#"
            .side_set 1

            .wrap_target
            public entry_point:
                out x, 24          side 0 ; Word 0[31:8]: X = (nibble_count - 1) [1..=16,777,216 nibbles]
                out pindirs, 4     side 0 ; Word 1[31:28]: Set D0..D3 pin directions
                out null, 4        side 0 ; Word 1[27:24]: Discard padding -> auto-pull next byte
            bitloop:
                out pins, 4        side 0 ; Cycle 0: CLK=0, drive D0..D3 (setup before rising edge)
                nop                side 1 ; Cycle 1: CLK=1 (rising edge, flash latches D0..D3)
                in pins, 4         side 1 ; Cycle 2: CLK=1, sample D0..D3 into ISR
                jmp x-- bitloop    side 0 ; Cycle 3: CLK=0 (falling edge, flash shifts next nibble)
            .wrap
            "#
        );

        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

// -----------------------------------------------------------------------------
// PIO QSPI Driver for W25Q128JVPIM
// -----------------------------------------------------------------------------

pub struct PioQspi<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    tx_dma: Channel<'d>,
    rx_dma: Channel<'d>,
    cs: Output<'d>,
    _clk_pin: Pin<'d, PIO>,
    _d0_pin: Pin<'d, PIO>,
    _d1_pin: Pin<'d, PIO>,
    _d2_pin: Pin<'d, PIO>,
    _d3_pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PioQspi<'d, PIO, SM> {
    /// Create and initialize a new DMA-backed PIO Quad SPI driver for the W25Q128JVPIM.
    #[allow(clippy::too_many_arguments)]
    pub fn new<TxDma: dma::ChannelInstance, RxDma: dma::ChannelInstance>(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        program: &PioQspiProgram<'d, PIO>,
        cs: Peri<'d, impl GpioPin + 'd>,
        clk: Peri<'d, impl PioPin + 'd>,
        d0: Peri<'d, impl PioPin + 'd>,
        d1: Peri<'d, impl PioPin + 'd>,
        d2: Peri<'d, impl PioPin + 'd>,
        d3: Peri<'d, impl PioPin + 'd>,
        tx_dma: Peri<'d, TxDma>,
        rx_dma: Peri<'d, RxDma>,
        irq: impl Binding<TxDma::Interrupt, dma::InterruptHandler<TxDma>>
            + Binding<RxDma::Interrupt, dma::InterruptHandler<RxDma>>
            + 'd,
        freq_hz: u32,
    ) -> Self {
        let cs_pin = Output::new(cs, Level::High);

        let mut clk_pin = common.make_pio_pin(clk);
        let mut d0_pin = common.make_pio_pin(d0);
        let mut d1_pin = common.make_pio_pin(d1);
        let mut d2_pin = common.make_pio_pin(d2);
        let mut d3_pin = common.make_pio_pin(d3);

        // Verify D0..D3 are contiguous (e.g. GPIO 21, 22, 23, 24)
        assert_eq!(d1_pin.pin(), d0_pin.pin() + 1, "D1 must be D0 + 1");
        assert_eq!(d2_pin.pin(), d0_pin.pin() + 2, "D2 must be D0 + 2");
        assert_eq!(d3_pin.pin(), d0_pin.pin() + 3, "D3 must be D0 + 3");

        // Enable weak pull-ups on D0..D3 so lines float high during turnaround/dummy cycles
        clk_pin.set_pull(Pull::None);
        d0_pin.set_pull(Pull::Up);
        d1_pin.set_pull(Pull::Up);
        d2_pin.set_pull(Pull::Up);
        d3_pin.set_pull(Pull::Up);

        for pin in [
            &mut clk_pin,
            &mut d0_pin,
            &mut d1_pin,
            &mut d2_pin,
            &mut d3_pin,
        ] {
            pin.set_slew_rate(SlewRate::Fast);
            pin.set_drive_strength(Drive::_8mA);
        }

        let mut cfg = Config::default();
        // Side-set controls CLK (GPIO 20)
        cfg.use_program(&program.prg, &[&clk_pin]);
        // OUT and IN control contiguous D0..D3 (GPIO 21..=24)
        cfg.set_out_pins(&[&d0_pin, &d1_pin, &d2_pin, &d3_pin]);
        cfg.set_in_pins(&[&d0_pin, &d1_pin, &d2_pin, &d3_pin]);

        // Auto-pull and auto-push every 8 bits (2 nibbles = 1 Quad byte)
        cfg.shift_out.auto_fill = true;
        cfg.shift_out.direction = ShiftDirection::Left;
        cfg.shift_out.threshold = 8;

        cfg.shift_in.auto_fill = true;
        cfg.shift_in.direction = ShiftDirection::Left;
        cfg.shift_in.threshold = 8;

        // 1 SCLK cycle in `bitloop` is 4 PIO clock cycles.
        cfg.clock_divider = calculate_pio_clock_divider(4 * freq_hz);

        sm.set_config(&cfg);

        // PIO0 shares one GPIOBASE bit across SM0 (Mag GPIO 42/43), SM1 (Flash GPIO 20..24),
        // and SM2 (Pitot GPIO 36/38). embassy-rp sets shift=0 whenever all pins of one SM are < 32,
        // so force PIO0 GPIOBASE=16 and shift Flash pinctrl bases by -16 before set_pins/set_pin_dirs.
        if clk_pin.pin() >= 16 && d0_pin.pin() >= 16 {
            let pac_pio0 = embassy_rp::pac::PIO0;
            pac_pio0.gpiobase().write(|w| w.set_gpiobase(true));
            pac_pio0.sm(SM).pinctrl().modify(|w| {
                w.set_in_base(d0_pin.pin() - 16);
                w.set_out_base(d0_pin.pin() - 16);
                w.set_sideset_base(clk_pin.pin() - 16);
            });
        }

        // Initialize CLK low and D2/D3 high (deassert /WP and /HOLD prior to QE enable)
        sm.set_pins(Level::Low, &[&clk_pin, &d0_pin, &d1_pin]);
        sm.set_pins(Level::High, &[&d2_pin, &d3_pin]);
        sm.set_pin_dirs(Direction::Out, &[&clk_pin, &d0_pin, &d2_pin, &d3_pin]);
        sm.set_pin_dirs(Direction::In, &[&d1_pin]);

        sm.clear_fifos();
        sm.restart();
        unsafe {
            sm.exec_jmp(program.prg.wrap.target);
        }
        sm.set_enable(true);

        let tx_dma = Channel::new(tx_dma, irq);
        let rx_dma = Channel::new(rx_dma, irq);

        Self {
            sm,
            tx_dma,
            rx_dma,
            cs: cs_pin,
            _clk_pin: clk_pin,
            _d0_pin: d0_pin,
            _d1_pin: d1_pin,
            _d2_pin: d2_pin,
            _d3_pin: d3_pin,
        }
    }

    #[inline]
    async fn start_phase(&mut self, nibble_count: usize, pindirs: u8) {
        debug_assert!(nibble_count >= 2 && nibble_count <= 0x100_0000 && nibble_count % 2 == 0);
        self.sm.tx().wait_push(((nibble_count - 1) as u32) << 8).await;
        self.sm.tx().wait_push((pindirs as u32) << 28).await;
    }

    #[inline]
    fn expand_spi_byte(byte: u8) -> [u8; 4] {
        let mut out = [0u8; 4];
        for i in 0..4 {
            let b_hi = (byte >> (7 - i * 2)) & 1;
            let b_lo = (byte >> (6 - i * 2)) & 1;
            let nib_hi = 0b1100 | b_hi;
            let nib_lo = 0b1100 | b_lo;
            out[i] = (nib_hi << 4) | nib_lo;
        }
        out
    }

    #[inline]
    fn collapse_spi_byte(rx: [u8; 4]) -> u8 {
        let mut out = 0u8;
        for i in 0..4 {
            let miso_hi = (rx[i] >> 5) & 1;
            let miso_lo = (rx[i] >> 1) & 1;
            out = (out << 2) | (miso_hi << 1) | miso_lo;
        }
        out
    }

    async fn spi_transfer_no_cs(&mut self, tx: &[u8], mut rx: Option<&mut [u8]>) {
        const MAX_SPI_CHUNK: usize = 16; // 16 SPI bytes = 64 expanded FIFO bytes
        let total_bytes = tx.len().max(rx.as_ref().map_or(0, |r| r.len()));
        if total_bytes == 0 {
            return;
        }

        let mut offset = 0;
        let mut tx_expanded = [0u8; MAX_SPI_CHUNK * 4];
        let mut rx_expanded = [0u8; MAX_SPI_CHUNK * 4];

        while offset < total_bytes {
            let chunk_bytes = (total_bytes - offset).min(MAX_SPI_CHUNK);
            let fifo_bytes = chunk_bytes * 4;
            let nibble_count = chunk_bytes * 8;

            for i in 0..chunk_bytes {
                let tx_byte = tx.get(offset + i).copied().unwrap_or(0xFF);
                let exp = Self::expand_spi_byte(tx_byte);
                tx_expanded[i * 4..i * 4 + 4].copy_from_slice(&exp);
            }

            self.start_phase(nibble_count, PINDIRS_SPI_1BIT).await;

            let (sm_rx, sm_tx) = self.sm.rx_tx();
            let tx_transfer = sm_tx.dma_push(&mut self.tx_dma, &tx_expanded[..fifo_bytes], false);

            if let Some(rx_slice) = rx.as_deref_mut() {
                let rx_transfer =
                    sm_rx.dma_pull(&mut self.rx_dma, &mut rx_expanded[..fifo_bytes], false);
                join(tx_transfer, rx_transfer).await;

                for i in 0..chunk_bytes {
                    if offset + i < rx_slice.len() {
                        let raw: [u8; 4] = rx_expanded[i * 4..i * 4 + 4].try_into().unwrap();
                        rx_slice[offset + i] = Self::collapse_spi_byte(raw);
                    }
                }
            } else {
                let rx_transfer = sm_rx.dma_pull_discard::<u8>(&mut self.rx_dma, fifo_bytes);
                join(tx_transfer, rx_transfer).await;
            }

            offset += chunk_bytes;
        }
    }

    async fn quad_write_no_cs(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        self.start_phase(data.len() * 2, PINDIRS_QUAD_TX).await;
        let (rx, tx) = self.sm.rx_tx();
        let rx_transfer = rx.dma_pull_discard::<u8>(&mut self.rx_dma, data.len());
        let tx_transfer = tx.dma_push(&mut self.tx_dma, data, false);
        join(tx_transfer, rx_transfer).await;
    }

    /// Initialize the W25Q128JVPIM: reset, verify JEDEC ID (`0xEF 0x40/0x70 0x18`), and enable Quad mode (`QE=1`).
    pub async fn init(&mut self) -> Result<[u8; 3], QspiError> {
        self.reset().await;

        let id = self.read_jedec_id().await;
        if id[0] != WINBOND_MFG_ID || id[2] != W25Q128_CAPACITY_ID {
            return Err(QspiError::InvalidJEDECId(id));
        }

        self.enable_quad_mode().await;
        Ok(id)
    }

    /// Issue software Reset Enable (`0x66`) + Reset Device (`0x99`) and wait 35 us (`tRST`).
    pub async fn reset(&mut self) {
        self.cs.set_low();
        self.spi_transfer_no_cs(&[0xFF, 0xFF], None).await;
        self.cs.set_high();

        Timer::after_micros(2).await;

        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_ENABLE_RESET], None).await;
        self.cs.set_high();

        Timer::after_micros(2).await;

        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_RESET_DEVICE], None).await;
        self.cs.set_high();

        Timer::after_micros(35).await;
    }

    /// Read the 3-byte JEDEC ID (`[manufacturer_id, memory_type, capacity]`).
    pub async fn read_jedec_id(&mut self) -> [u8; 3] {
        let mut rx = [0u8; 4];
        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_JEDEC_ID, 0x00, 0x00, 0x00], Some(&mut rx))
            .await;
        self.cs.set_high();
        [rx[1], rx[2], rx[3]]
    }

    /// Read Status Register 1 (`0x05`).
    pub async fn read_sr1(&mut self) -> u8 {
        let mut rx = [0u8; 2];
        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_READ_SR1, 0x00], Some(&mut rx))
            .await;
        self.cs.set_high();
        rx[1]
    }

    /// Read Status Register 2 (`0x35`).
    pub async fn read_sr2(&mut self) -> u8 {
        let mut rx = [0u8; 2];
        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_READ_SR2, 0x00], Some(&mut rx))
            .await;
        self.cs.set_high();
        rx[1]
    }

    /// Send Write Enable (`0x06`).
    pub async fn write_enable(&mut self) {
        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_WRITE_ENABLE], None).await;
        self.cs.set_high();
    }

    /// Poll Status Register 1 until `BUSY` (bit 0) clears, sleeping `poll_interval_us` between polls.
    pub async fn wait_busy_with_interval(&mut self, poll_interval_us: u64) {
        while (self.read_sr1().await & SR1_BUSY) != 0 {
            Timer::after_micros(poll_interval_us).await;
        }
    }

    /// Ensure the `QE` (Quad Enable, bit 1 of Status Register 2) bit is set.
    pub async fn enable_quad_mode(&mut self) {
        let sr2 = self.read_sr2().await;
        if (sr2 & SR2_QE) == 0 {
            self.write_enable().await;
            self.cs.set_low();
            self.spi_transfer_no_cs(&[CMD_WRITE_SR2, sr2 | SR2_QE], None)
                .await;
            self.cs.set_high();
            self.wait_busy_with_interval(1_000).await;
        }
    }

    /// Read arbitrary length data into `buf` starting at 24-bit `addr` using Fast Read Quad I/O (`0xEB`).
    pub async fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), QspiError> {
        if buf.is_empty() {
            return Ok(());
        }

        let addr_mode = [
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
            0xFF, // M7..M0 = 0xFF (non-continuous read mode)
        ];

        self.cs.set_low();
        self.spi_transfer_no_cs(&[CMD_FAST_READ_QUAD_IO], None)
            .await;
        self.quad_write_no_cs(&addr_mode).await;

        // 2 dummy bytes (4 dummy clocks) + `buf.len()` data bytes in a single Quad RX phase
        self.start_phase((2 + buf.len()) * 2, PINDIRS_QUAD_RX).await;
        {
            let (rx, tx) = self.sm.rx_tx();
            let tx_transfer = tx.dma_push_zeros::<u8>(&mut self.tx_dma, 2 + buf.len());
            let rx_transfer = async {
                rx.dma_pull_discard::<u8>(&mut self.rx_dma, 2).await;
                rx.dma_pull(&mut self.rx_dma, buf, false).await;
            };
            join(tx_transfer, rx_transfer).await;
        }
        self.cs.set_high();

        Ok(())
    }

    /// Program up to 256 bytes within a single 256-byte page using Quad Input Page Program (`0x32`).
    pub async fn write_page(&mut self, addr: u32, data: &[u8]) -> Result<(), QspiError> {
        if data.is_empty() {
            return Err(QspiError::LengthZero);
        }
        let page_offset = (addr as usize) & (W25Q_PAGE_SIZE - 1);
        if page_offset + data.len() > W25Q_PAGE_SIZE {
            return Err(QspiError::PageBoundaryCrossed);
        }

        self.write_enable().await;

        let hdr = [
            CMD_QUAD_PAGE_PROGRAM,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];

        self.cs.set_low();
        self.spi_transfer_no_cs(&hdr, None).await;
        self.quad_write_no_cs(data).await;
        self.cs.set_high();

        self.wait_busy_with_interval(250).await;
        Ok(())
    }

    /// Write an arbitrary slice across page boundaries at `addr`.
    pub async fn write(&mut self, mut addr: u32, mut data: &[u8]) -> Result<(), QspiError> {
        while !data.is_empty() {
            let page_rem = W25Q_PAGE_SIZE - ((addr as usize) & (W25Q_PAGE_SIZE - 1));
            let chunk_len = data.len().min(page_rem);
            self.write_page(addr, &data[..chunk_len]).await?;
            addr += chunk_len as u32;
            data = &data[chunk_len..];
        }
        Ok(())
    }

    /// Erase a 4 KB sector (`0x20`) at `addr` (must be 4 KB aligned).
    pub async fn erase_sector_4k(&mut self, addr: u32) -> Result<(), QspiError> {
        if (addr as usize) & (W25Q_SECTOR_SIZE - 1) != 0 {
            return Err(QspiError::UnalignedAddress);
        }

        self.write_enable().await;
        let cmd = [
            CMD_SECTOR_ERASE_4K,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];
        self.cs.set_low();
        self.spi_transfer_no_cs(&cmd, None).await;
        self.cs.set_high();

        self.wait_busy_with_interval(2_000).await;
        Ok(())
    }
}
