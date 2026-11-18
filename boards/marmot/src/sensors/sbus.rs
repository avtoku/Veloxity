use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, Pin, PioPin,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::{Duration, Instant, with_timeout};
use veloxity_core::packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader};

use super::SHARED_RC_QUEUE;

pub const SBUS_BAUD: u32 = 100_000;
pub const SBUS_FRAME_SIZE: usize = 25;
pub const SBUS_START_BYTE: u8 = 0x0F;
pub const SBUS_11_BIT_CHANNELS: usize = 16;
const SBUS_IDLE_GAP: Duration = Duration::from_micros(500);

pub struct PioSbusRxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSbusRxProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            r#"
            .wrap_target
            start:
                wait 0 pin 0            ; Stall until start bit (0 after hardware input inversion)
                set x, 8    [10]        ; 9 bits (8 data + 1 even parity), delay 12 clocks to center of D0
            rx_bitloop:
                in pins, 1              ; Shift bit into ISR (Right shift: D0..D7, then Parity)
                jmp x-- rx_bitloop [6]  ; 8 PIO clocks per bit
                jmp pin check_stop2     ; Check Stop Bit 1 (must be High)
                jmp framing_err
            check_stop2:
                nop         [6]         ; Advance 8 PIO clocks to center of Stop Bit 2
                jmp pin good_rx_stop    ; Check Stop Bit 2 (must be High)
            framing_err:
                wait 1 pin 0            ; Wait for line to return to idle High
                jmp start               ; Discard byte on stop-bit error
            good_rx_stop:
                in null, 23             ; Align 9 bits into ISR[8:0] (ISR[7:0] = byte, ISR[8] = parity)
                push noblock            ; Push 32-bit word to RX FIFO
            .wrap
            "#
        );

        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

pub struct PioSbusRx<'d, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'d, PIO, SM>,
    _rx_pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PioSbusRx<'d, PIO, SM> {
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin + 'd>,
        program: &PioSbusRxProgram<'d, PIO>,
    ) -> Self {
        let mut rx_pin = common.make_pio_pin(rx_pin);
        // S.Bus idles physical Low on the wire; enable hardware IO_BANK0 input inversion.
        rx_pin.set_pull(Pull::Down);
        embassy_rp::pac::IO_BANK0
            .gpio(rx_pin.pin() as usize)
            .ctrl()
            .modify(|w| w.set_inover(embassy_rp::pac::io::vals::Inover::INVERT));

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        cfg.clock_divider = calculate_pio_clock_divider(8 * SBUS_BAUD);
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;

        sm_rx.set_config(&cfg);
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);
        sm_rx.clear_fifos();
        sm_rx.restart();
        sm_rx.set_enable(true);

        Self {
            sm_rx,
            _rx_pin: rx_pin,
        }
    }

    pub async fn read_byte(&mut self) -> Result<u8, ()> {
        let word = self.sm_rx.rx().wait_pull().await;
        let byte = (word & 0xFF) as u8;
        let rx_parity = ((word >> 8) & 1) as u8;
        let expected_parity = (byte.count_ones() as u8) & 1;

        if rx_parity == expected_parity {
            Ok(byte)
        } else {
            Err(())
        }
    }

    pub async fn read_frame(&mut self, buff: &mut [u8; SBUS_FRAME_SIZE]) {
        let mut idx = 0usize;
        let mut need_gap = false;
        let mut last_byte = Instant::now();

        loop {
            match self.read_byte().await {
                Ok(byte) => {
                    let now = Instant::now();
                    if now.duration_since(last_byte) > SBUS_IDLE_GAP {
                        idx = 0;
                        need_gap = false;
                    }
                    last_byte = now;

                    if need_gap {
                        continue;
                    }

                    if idx == 0 && byte != SBUS_START_BYTE {
                        continue;
                    }

                    buff[idx] = byte;
                    idx += 1;

                    if idx == SBUS_FRAME_SIZE {
                        return;
                    }
                }
                Err(()) => {
                    idx = 0;
                    need_gap = true;
                    last_byte = Instant::now();
                }
            }
        }
    }
}

fn extract_chan(bytes: &[u8], bit_offset: usize) -> f32 {
    let byte_offset = bit_offset / 8;
    let bit_start = bit_offset % 8;

    let mut value: u16 = 0;
    for i in 0..11 {
        let byte_index = byte_offset + (bit_start + i) / 8;
        let bit_index = (bit_start + i) % 8;
        let bit = (bytes[byte_index] >> bit_index) & 1;
        value |= (bit as u16) << i;
    }

    value as f32
}

#[embassy_executor::task]
pub async fn task(mut rx: PioSbusRx<'static, PIO1, 0>) -> ! {
    let mut buff = [0u8; SBUS_FRAME_SIZE];
    let mut last_channels = [0.0f32; RC_PACKET_CHANNELS];

    loop {
        if with_timeout(Duration::from_secs(1), rx.read_frame(&mut buff))
            .await
            .is_err()
        {
            let dig = 0x1C;
            SHARED_RC_QUEUE.push(Ok(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: Instant::now().as_micros(),
                    status: dig,
                },
                n_chan: RC_PACKET_CHANNELS as u32,
                chan: last_channels,
                lol: true,
            }));
            continue;
        }

        let valid_header = buff[0] == SBUS_START_BYTE;
        let valid_footer = matches!(buff[24], 0x00 | 0x04 | 0x14 | 0x24 | 0x34);
        if valid_header && valid_footer {
            let dig = buff[23] as u16;
            let mut chan = [0.0f32; RC_PACKET_CHANNELS];

            for i in 0..SBUS_11_BIT_CHANNELS {
                chan[i] = (extract_chan(&buff, 8 + i * 11) - 172.0) / 1639.0;
            }
            chan[SBUS_11_BIT_CHANNELS] = (((dig & 0x01) as f32) - 172.0) / 1639.0;
            chan[SBUS_11_BIT_CHANNELS + 1] = ((((dig >> 1) & 0x01) as f32) - 172.0) / 1639.0;
            last_channels = chan;

            SHARED_RC_QUEUE.push(Ok(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: Instant::now().as_micros(),
                    status: dig,
                },
                n_chan: RC_PACKET_CHANNELS as u32,
                chan,
                lol: (dig & 0x0C) != 0,
            }));
        }
    }
}
