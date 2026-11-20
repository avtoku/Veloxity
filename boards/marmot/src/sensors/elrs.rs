// -----------------------------------------------------------------------------
// ExpressLRS (ELRS / CRSF) Driver - 420 kbaud 8N1 over PIO1 UART + DMA
// -----------------------------------------------------------------------------

use embassy_rp::Peri;
use embassy_rp::dma::{self, Channel};
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, Pin, PioPin,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, with_timeout};
use veloxity_core::packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader};

use super::SHARED_RC_QUEUE;

// -----------------------------------------------------------------------------
// CRSF Protocol Constants
// -----------------------------------------------------------------------------

pub const CRSF_BAUD: u32 = 420_000;
pub const CRSF_MAX_FRAME_SIZE: usize = 64;
pub const CRSF_MAX_PAYLOAD_SIZE: usize = 60;
const CRSF_FRAME_DMA_TIMEOUT: Duration = Duration::from_millis(2);
const CRSF_LINK_TIMEOUT: Duration = Duration::from_millis(50); // Supports any ELRS frame rate >= 50 Hz

/// CRSF sync / device address bytes
pub const CRSF_ADDRESS_FLIGHT_CONTROLLER: u8 = 0xC8;
pub const CRSF_ADDRESS_RADIO_TRANSMITTER: u8 = 0xEA;
pub const CRSF_ADDRESS_CRSF_RECEIVER: u8 = 0xEC;
pub const CRSF_ADDRESS_CRSF_TRANSMITTER: u8 = 0xEE;

/// CRSF frame type IDs
pub const CRSF_FRAMETYPE_GPS: u8 = 0x02;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: u8 = 0x08;
pub const CRSF_FRAMETYPE_BARO_ALTITUDE: u8 = 0x09;
pub const CRSF_FRAMETYPE_LINK_STATISTICS: u8 = 0x14;
pub const CRSF_FRAMETYPE_RC_CHANNELS_PACKED: u8 = 0x16;
pub const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

pub const CRSF_NUM_CHANNELS: usize = 16;

// -----------------------------------------------------------------------------
// Inter-Task Signals & Data Types
// -----------------------------------------------------------------------------

pub static ELRS_LINK_STATS_SIGNAL: Signal<CriticalSectionRawMutex, ElrsLinkStats> = Signal::new();
pub static ELRS_TELEMETRY_SIGNAL: Signal<CriticalSectionRawMutex, ElrsTelemetry> = Signal::new();

#[derive(Clone, Copy, Default)]
pub struct ElrsLinkStats {
    pub timestamp_us: u64,
    pub uplink_rssi_1: i8,
    pub uplink_rssi_2: i8,
    pub uplink_link_quality: u8,
    pub uplink_snr: i8,
    pub active_antenna: u8,
    pub rf_mode: u8,
    pub uplink_tx_power: u8,
    pub downlink_rssi: i8,
    pub downlink_link_quality: u8,
    pub downlink_snr: i8,
}

#[derive(Clone, Copy)]
pub enum ElrsTelemetry {
    Battery {
        /// Battery voltage in millivolts
        voltage_mv: u16,
        /// Current draw in units of 10 mA (0.01 A)
        current_10ma: u16,
        /// Consumed capacity in mAh (24-bit)
        capacity_mah: u32,
        /// Remaining battery percentage (`0..=100`)
        remaining_pct: u8,
    },
    Attitude {
        /// Pitch angle in radians
        pitch_rad: f32,
        /// Roll angle in radians
        roll_rad: f32,
        /// Yaw angle in radians
        yaw_rad: f32,
    },
    Gps {
        /// Latitude in degrees * 1e7
        lat_1e7: i32,
        /// Longitude in degrees * 1e7
        lon_1e7: i32,
        /// Ground speed in cm/s
        groundspeed_cms: u16,
        /// Heading in degrees * 100
        heading_cdeg: u16,
        /// Altitude in meters above sea level
        altitude_m: i16,
        /// Number of satellites in view
        sats: u8,
    },
    BaroAltitude {
        /// Altitude in meters (packed into CRSF 0.1 m / 1 m format)
        altitude_m: f32,
        /// Vertical speed in m/s
        vario_mps: f32,
    },
    FlightMode {
        mode: [u8; 16],
        len: u8,
    },
}

// -----------------------------------------------------------------------------
// CRSF Helpers (CRC-8/DVB-S2 & Channel Packing/Conversion)
// -----------------------------------------------------------------------------

/// Compute CRSF CRC-8/DVB-S2 (polynomial `0xD5`, init `0x00`, MSB-first).
pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Convert a raw 11-bit CRSF channel value (`172..=1811`) to microseconds (`988..=2012 us`).
#[inline]
pub fn crsf_to_us(raw: u16) -> u16 {
    let us = ((raw as i32 - 992) * 5) / 8 + 1500;
    us.clamp(800, 2200) as u16
}

/// Normalize a raw 11-bit CRSF channel value (`172..=1811`) to `[0.0, 1.0]`.
#[inline]
pub fn crsf_to_unit(raw: u16) -> f32 {
    (((raw as f32) - 172.0) / 1639.0).clamp(0.0, 1.0)
}

/// Unpack 16 x 11-bit LSB-first channels from a 22-byte `CRSF_FRAMETYPE_RC_CHANNELS_PACKED` payload.
pub fn unpack_channels(payload: &[u8; 22]) -> [u16; CRSF_NUM_CHANNELS] {
    let mut channels = [0u16; CRSF_NUM_CHANNELS];
    let mut bit_offset = 0usize;

    for ch in &mut channels {
        let byte_idx = bit_offset / 8;
        let bit_shift = bit_offset % 8;

        let b0 = payload[byte_idx] as u32;
        let b1 = payload[byte_idx + 1] as u32;
        let b2 = if byte_idx + 2 < payload.len() {
            payload[byte_idx + 2] as u32
        } else {
            0
        };

        let word = b0 | (b1 << 8) | (b2 << 16);
        *ch = ((word >> bit_shift) & 0x07FF) as u16;
        bit_offset += 11;
    }

    channels
}

#[inline]
fn is_valid_sync(byte: u8) -> bool {
    matches!(
        byte,
        CRSF_ADDRESS_FLIGHT_CONTROLLER
            | CRSF_ADDRESS_RADIO_TRANSMITTER
            | CRSF_ADDRESS_CRSF_RECEIVER
            | CRSF_ADDRESS_CRSF_TRANSMITTER
    )
}

/// Encode a CRSF frame into `out[..total_len]` and return `total_len`.
fn build_frame(frame_type: u8, payload: &[u8], out: &mut [u8; CRSF_MAX_FRAME_SIZE]) -> usize {
    let payload_len = payload.len().min(CRSF_MAX_PAYLOAD_SIZE);
    let len = payload_len + 2; // type (1) + payload + crc (1)

    out[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    out[1] = len as u8;
    out[2] = frame_type;
    out[3..3 + payload_len].copy_from_slice(&payload[..payload_len]);
    out[3 + payload_len] = crc8_dvb_s2(&out[2..3 + payload_len]);

    4 + payload_len
}

// -----------------------------------------------------------------------------
// PIO UART + DMA Transport (Zero Per-Byte CPU Interrupts)
// -----------------------------------------------------------------------------

pub struct PioElrsRxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioElrsRxProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            r#"
            start:
                wait 0 pin 0            ; Stall until start bit
                set x, 7    [10]        ; 8 data bits, delay to center of D0
            rx_bitloop:
                in pins, 1              ; Shift bit into ISR (Right shift)
                jmp x-- rx_bitloop [6]  ; 8 PIO clocks per bit
                jmp pin good_rx_stop    ; Check stop bit (High)
                irq 4 rel               ; Framing error flag
                wait 1 pin 0            ; Wait for idle High before restarting
                jmp start
            good_rx_stop:
                in null, 24             ; Align data byte into ISR[7:0] for 8-bit DMA reads
                push
            "#
        );

        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

pub struct PioElrsRx<'d, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'d, PIO, SM>,
    rx_dma: Channel<'d>,
    _rx_pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PioElrsRx<'d, PIO, SM> {
    pub fn new<RxDma: dma::ChannelInstance>(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin + 'd>,
        rx_dma: Peri<'d, RxDma>,
        irq: impl Binding<RxDma::Interrupt, dma::InterruptHandler<RxDma>> + 'd,
        program: &PioElrsRxProgram<'d, PIO>,
    ) -> Self {
        let mut rx_pin = common.make_pio_pin(rx_pin);
        rx_pin.set_pull(Pull::Up);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        cfg.clock_divider = calculate_pio_clock_divider(8 * baud);
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
            rx_dma: Channel::new(rx_dma, irq),
            _rx_pin: rx_pin,
        }
    }

    pub async fn read_dma(&mut self, buf: &mut [u8]) {
        self.sm_rx.rx().dma_pull(&mut self.rx_dma, buf, false).await;
    }

    pub fn drain_fifo(&mut self) {
        while self.sm_rx.rx().try_pull().is_some() {}
    }
}

pub struct PioElrsTxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioElrsTxProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            r#"
            .side_set 1 opt
                pull       side 1 [7]   ; Assert idle/stop bit, stall until TX FIFO has data
                set x, 7   side 0 [7]   ; Assert start bit for 8 PIO clocks
            bitloop:
                out pins, 1             ; Shift LSB first onto TX pin
                jmp x-- bitloop [6]     ; 8 PIO clocks per bit
            "#
        );

        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

pub struct PioElrsTx<'d, PIO: Instance, const SM: usize> {
    sm_tx: StateMachine<'d, PIO, SM>,
    tx_dma: Channel<'d>,
    _tx_pin: Pin<'d, PIO>,
}

impl<'d, PIO: Instance, const SM: usize> PioElrsTx<'d, PIO, SM> {
    pub fn new<TxDma: dma::ChannelInstance>(
        baud: u32,
        common: &mut Common<'d, PIO>,
        mut sm_tx: StateMachine<'d, PIO, SM>,
        tx_pin: Peri<'d, impl PioPin + 'd>,
        tx_dma: Peri<'d, TxDma>,
        irq: impl Binding<TxDma::Interrupt, dma::InterruptHandler<TxDma>> + 'd,
        program: &PioElrsTxProgram<'d, PIO>,
    ) -> Self {
        let tx_pin = common.make_pio_pin(tx_pin);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[&tx_pin]);
        cfg.set_out_pins(&[&tx_pin]);
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = calculate_pio_clock_divider(8 * baud);

        sm_tx.set_config(&cfg);
        sm_tx.set_pins(Level::High, &[&tx_pin]);
        sm_tx.set_pin_dirs(PioDirection::Out, &[&tx_pin]);
        sm_tx.clear_fifos();
        sm_tx.restart();
        sm_tx.set_enable(true);

        Self {
            sm_tx,
            tx_dma: Channel::new(tx_dma, irq),
            _tx_pin: tx_pin,
        }
    }

    pub async fn write_dma(&mut self, bytes: &[u8]) {
        self.sm_tx.tx().dma_push(&mut self.tx_dma, bytes, false).await;
    }
}

// -----------------------------------------------------------------------------
// ELRS RX Task (2-Stage Exact-Length DMA: 2-Byte Header + N-Byte Body)
// -----------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn rx_task(mut rx: PioElrsRx<'static, PIO1, 0>) -> ! {
    let mut buf = [0u8; CRSF_MAX_FRAME_SIZE];
    let mut last_rc_time = Instant::now();
    let mut uplink_lq_lost = false;
    let mut last_rc = RcPacket {
        header: RosflightPacketHeader {
            timestamp: 0,
            status: 0,
        },
        n_chan: CRSF_NUM_CHANNELS as u32,
        chan: [0.0_f32; RC_PACKET_CHANNELS],
        lol: true,
    };

    'frame_loop: loop {
        // Stage 1: DMA 2-byte header [sync, len] (1 DMA IRQ instead of per-byte IRQs)
        if with_timeout(CRSF_LINK_TIMEOUT, rx.read_dma(&mut buf[0..2]))
            .await
            .is_err()
        {
            rx.drain_fifo();
            last_rc.header.timestamp = Instant::now().as_micros();
            last_rc.lol = true;
            SHARED_RC_QUEUE.push(Ok(last_rc));
            continue;
        }

        // Slide 1 byte via DMA until [sync, valid_len] aligns
        while !is_valid_sync(buf[0]) || !(2..=CRSF_MAX_FRAME_SIZE as u8 - 2).contains(&buf[1]) {
            buf[0] = buf[1];
            if with_timeout(CRSF_LINK_TIMEOUT, rx.read_dma(&mut buf[1..2]))
                .await
                .is_err()
            {
                rx.drain_fifo();
                last_rc.header.timestamp = Instant::now().as_micros();
                last_rc.lol = true;
                SHARED_RC_QUEUE.push(Ok(last_rc));
                continue 'frame_loop;
            }
        }

        let expected_total = buf[1] as usize + 2;

        // Stage 2: DMA exact remaining frame [type + payload + crc] (1 DMA IRQ on final CRC byte)
        if with_timeout(CRSF_FRAME_DMA_TIMEOUT, rx.read_dma(&mut buf[2..expected_total]))
            .await
            .is_err()
        {
            rx.drain_fifo();
            continue;
        }

        let now = Instant::now();
        let crc_idx = expected_total - 1;
        let expected_crc = crc8_dvb_s2(&buf[2..crc_idx]);

        if buf[crc_idx] == expected_crc {
            let frame_type = buf[2];
            let payload = &buf[3..crc_idx];

            match frame_type {
                CRSF_FRAMETYPE_RC_CHANNELS_PACKED if payload.len() == 22 => {
                    let raw: &[u8; 22] = payload.try_into().unwrap();
                    let raw_channels = unpack_channels(raw);
                    let mut chan = [0.0_f32; RC_PACKET_CHANNELS];
                    for (dst, &raw_ch) in chan.iter_mut().zip(raw_channels.iter()) {
                        *dst = crsf_to_unit(raw_ch);
                    }
                    last_rc_time = now;
                    let packet = RcPacket {
                        header: RosflightPacketHeader {
                            timestamp: now.as_micros(),
                            status: 0,
                        },
                        n_chan: CRSF_NUM_CHANNELS as u32,
                        chan,
                        lol: uplink_lq_lost,
                    };
                    last_rc = packet;
                    SHARED_RC_QUEUE.push(Ok(packet));
                }
                CRSF_FRAMETYPE_LINK_STATISTICS if payload.len() >= 10 => {
                    let stats = ElrsLinkStats {
                        timestamp_us: now.as_micros(),
                        uplink_rssi_1: -(payload[0] as i8),
                        uplink_rssi_2: -(payload[1] as i8),
                        uplink_link_quality: payload[2],
                        uplink_snr: payload[3] as i8,
                        active_antenna: payload[4],
                        rf_mode: payload[5],
                        uplink_tx_power: payload[6],
                        downlink_rssi: -(payload[7] as i8),
                        downlink_link_quality: payload[8],
                        downlink_snr: payload[9] as i8,
                    };
                    uplink_lq_lost = stats.uplink_link_quality == 0;
                    if uplink_lq_lost || now.duration_since(last_rc_time) > CRSF_LINK_TIMEOUT {
                        last_rc.header.timestamp = now.as_micros();
                        last_rc.lol = true;
                        SHARED_RC_QUEUE.push(Ok(last_rc));
                    }
                    ELRS_LINK_STATS_SIGNAL.signal(stats);
                }
                _ => {}
            }
        }
    }
}

// -----------------------------------------------------------------------------
// ELRS TX Task
// -----------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn tx_task(mut tx: PioElrsTx<'static, PIO1, 1>) -> ! {
    let mut frame = [0u8; CRSF_MAX_FRAME_SIZE];

    loop {
        let telem = ELRS_TELEMETRY_SIGNAL.wait().await;
        let len = match telem {
            ElrsTelemetry::Battery {
                voltage_mv,
                current_10ma,
                capacity_mah,
                remaining_pct,
            } => {
                let voltage_dv = (voltage_mv / 100).to_be_bytes();
                let current_da = (current_10ma / 10).to_be_bytes();
                let cap = capacity_mah.to_be_bytes();
                let payload = [
                    voltage_dv[0],
                    voltage_dv[1],
                    current_da[0],
                    current_da[1],
                    cap[1],
                    cap[2],
                    cap[3],
                    remaining_pct.min(100),
                ];
                build_frame(CRSF_FRAMETYPE_BATTERY_SENSOR, &payload, &mut frame)
            }
            ElrsTelemetry::Attitude {
                pitch_rad,
                roll_rad,
                yaw_rad,
            } => {
                let pitch = ((pitch_rad * 10_000.0) as i16).to_be_bytes();
                let roll = ((roll_rad * 10_000.0) as i16).to_be_bytes();
                let yaw = ((yaw_rad * 10_000.0) as i16).to_be_bytes();
                let payload = [pitch[0], pitch[1], roll[0], roll[1], yaw[0], yaw[1]];
                build_frame(CRSF_FRAMETYPE_ATTITUDE, &payload, &mut frame)
            }
            ElrsTelemetry::Gps {
                lat_1e7,
                lon_1e7,
                groundspeed_cms,
                heading_cdeg,
                altitude_m,
                sats,
            } => {
                let lat = lat_1e7.to_be_bytes();
                let lon = lon_1e7.to_be_bytes();
                // CRSF groundspeed is km/h * 10 (0.1 km/h): cm/s * 0.36
                let gspd = (((groundspeed_cms as u32) * 36) / 100) as u16;
                let gspd_be = gspd.to_be_bytes();
                let hdg_be = heading_cdeg.to_be_bytes();
                let alt_be =
                    ((altitude_m as i32 + 1000).clamp(0, u16::MAX as i32) as u16).to_be_bytes();
                let payload = [
                    lat[0], lat[1], lat[2], lat[3], lon[0], lon[1], lon[2], lon[3], gspd_be[0],
                    gspd_be[1], hdg_be[0], hdg_be[1], alt_be[0], alt_be[1], sats,
                ];
                build_frame(CRSF_FRAMETYPE_GPS, &payload, &mut frame)
            }
            ElrsTelemetry::BaroAltitude {
                altitude_m,
                vario_mps,
            } => {
                // CRSF baro altitude: < 0x8000 => decimeters + 10000 offset; >= 0x8000 => meters
                let alt_dm = (altitude_m * 10.0) as i32 + 10_000;
                let alt_packed: u16 = if (0..=0x7FFF).contains(&alt_dm) {
                    alt_dm as u16
                } else {
                    (altitude_m as i32).clamp(0, 0x7FFF) as u16 | 0x8000
                };
                let alt_be = alt_packed.to_be_bytes();
                let vario_cms = ((vario_mps * 100.0) as i16).clamp(i16::MIN, i16::MAX);
                let vario_be = vario_cms.to_be_bytes();
                let payload = [alt_be[0], alt_be[1], vario_be[0], vario_be[1]];
                build_frame(CRSF_FRAMETYPE_BARO_ALTITUDE, &payload, &mut frame)
            }
            ElrsTelemetry::FlightMode { mode, len } => {
                let mut payload = [0u8; 17];
                let copy_len = (len as usize).min(16);
                payload[..copy_len].copy_from_slice(&mode[..copy_len]);
                payload[copy_len] = 0; // null-terminated
                build_frame(CRSF_FRAMETYPE_FLIGHT_MODE, &payload[..=copy_len], &mut frame)
            }
        };

        tx.write_dma(&frame[..len]).await;
    }
}
