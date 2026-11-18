use embassy_rp::gpio::Input;
use embassy_rp::uart::{Async, Error as UartError, Uart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use veloxity_core::packets::{
    GNSSFixType, GNSSPacket, RosflightPacketHeader, rosflight_c_gnss_timestamp,
};

use super::SHARED_GNSS_QUEUE;

pub const UBX_SYNC1: u8 = 0xB5;
pub const UBX_SYNC2: u8 = 0x62;

pub const UBX_CLASS_NAV: u8 = 0x01;
pub const UBX_ID_NAV_PVT: u8 = 0x07;

pub const UBX_CLASS_ACK: u8 = 0x05;
pub const UBX_ID_ACK_NAK: u8 = 0x00;
pub const UBX_ID_ACK_ACK: u8 = 0x01;

pub const UBX_CLASS_CFG: u8 = 0x06;
pub const UBX_ID_CFG_VALSET: u8 = 0x8A;

pub const UBX_NAV_PVT_LEN: usize = 92;
pub const UBX_MAX_PAYLOAD: usize = 100;

pub const TARGET_BAUDRATE: u32 = 115_200;
const BAUDRATES: [u32; 5] = [9_600, 115_200, 38_400, 57_600, 230_400];

pub const CFG_UART1_BAUDRATE: u32 = 0x4052_0001;
pub const CFG_UART1INPROT_UBX: u32 = 0x1073_0001;
pub const CFG_UART1INPROT_NMEA: u32 = 0x1073_0002;
pub const CFG_UART1OUTPROT_UBX: u32 = 0x1074_0001;
pub const CFG_UART1OUTPROT_NMEA: u32 = 0x1074_0002;

pub const CFG_RATE_MEAS: u32 = 0x3021_0001;
pub const CFG_RATE_NAV: u32 = 0x3021_0002;
pub const CFG_RATE_TIMEREF: u32 = 0x2021_0003;

pub const CFG_NAVSPG_FIXMODE: u32 = 0x2011_0011;
pub const CFG_NAVSPG_DYNMODEL: u32 = 0x2011_0021;

pub const CFG_MSGOUT_UBX_NAV_PVT_UART1: u32 = 0x2091_0007;

pub const CFG_TP_PULSE_DEF: u32 = 0x2005_0023;
pub const CFG_TP_PULSE_LENGTH_DEF: u32 = 0x2005_0030;
pub const CFG_TP_PERIOD_TP1: u32 = 0x4005_0002;
pub const CFG_TP_PERIOD_LOCK_TP1: u32 = 0x4005_0003;
pub const CFG_TP_LEN_TP1: u32 = 0x4005_0004;
pub const CFG_TP_LEN_LOCK_TP1: u32 = 0x4005_0005;
pub const CFG_TP_TP1_ENA: u32 = 0x1005_0007;
pub const CFG_TP_SYNC_GNSS_TP1: u32 = 0x1005_0008;
pub const CFG_TP_USE_LOCKED_TP1: u32 = 0x1005_0009;
pub const CFG_TP_ALIGN_TO_TOW_TP1: u32 = 0x1005_000A;
pub const CFG_TP_POL_TP1: u32 = 0x1005_000B;
pub const CFG_TP_TIMEGRID_TP1: u32 = 0x2005_000C;

pub const VALSET_LAYER_RAM: u8 = 0x01;
pub const NAV_PERIOD_MS: u16 = 100;

pub static PPS_SIGNAL: Signal<CriticalSectionRawMutex, u64> = Signal::new();

#[derive(Clone, Copy)]
struct PvtPayload {
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    min: u8,
    sec: u8,
    valid: u8,
    nano: i32,
    fix_type: u8,
    flags: u8,
    flags2: u8,
    num_sv: u8,
    lon: i32,
    lat: i32,
    h_msl: i32,
    h_acc: u32,
    v_acc: u32,
    vel_n: i32,
    vel_e: i32,
    vel_d: i32,
    s_acc: u32,
    flags3: u16,
    mag_dec: i16,
}

impl PvtPayload {
    fn from_le_bytes(b: &[u8; UBX_NAV_PVT_LEN]) -> Self {
        Self {
            year: u16::from_le_bytes([b[4], b[5]]),
            month: b[6],
            day: b[7],
            hour: b[8],
            min: b[9],
            sec: b[10],
            valid: b[11],
            nano: i32::from_le_bytes([b[16], b[17], b[18], b[19]]),
            fix_type: b[20],
            flags: b[21],
            flags2: b[22],
            num_sv: b[23],
            lon: i32::from_le_bytes([b[24], b[25], b[26], b[27]]),
            lat: i32::from_le_bytes([b[28], b[29], b[30], b[31]]),
            h_msl: i32::from_le_bytes([b[36], b[37], b[38], b[39]]),
            h_acc: u32::from_le_bytes([b[40], b[41], b[42], b[43]]),
            v_acc: u32::from_le_bytes([b[44], b[45], b[46], b[47]]),
            vel_n: i32::from_le_bytes([b[48], b[49], b[50], b[51]]),
            vel_e: i32::from_le_bytes([b[52], b[53], b[54], b[55]]),
            vel_d: i32::from_le_bytes([b[56], b[57], b[58], b[59]]),
            s_acc: u32::from_le_bytes([b[68], b[69], b[70], b[71]]),
            flags3: u16::from_le_bytes([b[78], b[79]]),
            mag_dec: i16::from_le_bytes([b[88], b[89]]),
        }
    }
}

#[derive(Clone, Copy)]
struct UbxPacket {
    cl: u8,
    id: u8,
    len: u16,
    data: [u8; UBX_MAX_PAYLOAD],
}

impl Default for UbxPacket {
    fn default() -> Self {
        Self {
            cl: 0,
            id: 0,
            len: 0,
            data: [0u8; UBX_MAX_PAYLOAD],
        }
    }
}

fn unix_seconds_from_utc(year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> i64 {
    let mut y = year as i32;
    let m = month as i32;
    y -= (m <= 2) as i32;
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = m + if m > 2 { -3 } else { 9 };
    let doy = (153 * mp + 2) / 5 + day as i32 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146097 + doe - 719468;
    days as i64 * 86_400 + hour as i64 * 3_600 + min as i64 * 60 + sec as i64
}

fn ubx_checksum(header_body: &[u8], payload: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in header_body.iter().chain(payload.iter()) {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

fn make_ubx_packet(class: u8, id: u8, payload: &[u8], out: &mut [u8]) -> usize {
    let len = payload.len();
    let len_bytes = (len as u16).to_le_bytes();
    let hdr_body = [class, id, len_bytes[0], len_bytes[1]];
    let (ck_a, ck_b) = ubx_checksum(&hdr_body, payload);

    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2..6].copy_from_slice(&hdr_body);
    out[6..6 + len].copy_from_slice(payload);
    out[6 + len] = ck_a;
    out[7 + len] = ck_b;
    len + 8
}

struct ValsetBuilder {
    buf: [u8; 96],
    pos: usize,
}

impl ValsetBuilder {
    fn new(layers: u8) -> Self {
        let mut buf = [0u8; 96];
        buf[0] = 0x00;
        buf[1] = layers;
        Self { buf, pos: 4 }
    }

    fn push_u8(&mut self, key: u32, val: u8) -> &mut Self {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&key.to_le_bytes());
        self.buf[self.pos + 4] = val;
        self.pos += 5;
        self
    }

    fn push_u16(&mut self, key: u32, val: u16) -> &mut Self {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&key.to_le_bytes());
        self.buf[self.pos + 4..self.pos + 6].copy_from_slice(&val.to_le_bytes());
        self.pos += 6;
        self
    }

    fn push_u32(&mut self, key: u32, val: u32) -> &mut Self {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&key.to_le_bytes());
        self.buf[self.pos + 4..self.pos + 8].copy_from_slice(&val.to_le_bytes());
        self.pos += 8;
        self
    }

    fn payload(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

async fn read_ubx_packet(uart: &mut Uart<'static, Async>, pkt: &mut UbxPacket) -> Result<(), UartError> {
    let mut hdr = [0u8; 6];
    let mut body = [0u8; UBX_MAX_PAYLOAD + 2];

    uart.read(&mut hdr).await?;

    loop {
        while hdr[0] != UBX_SYNC1 || hdr[1] != UBX_SYNC2 {
            let mut shift = None;
            for k in 1..6 {
                if hdr[k] == UBX_SYNC1 && (k == 5 || hdr[k + 1] == UBX_SYNC2) {
                    shift = Some(k);
                    break;
                }
            }

            if let Some(k) = shift {
                hdr.copy_within(k..6, 0);
                uart.read(&mut hdr[6 - k..6]).await?;
            } else {
                uart.read(&mut hdr).await?;
            }
        }

        let len = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if len > UBX_MAX_PAYLOAD {
            hdr[0] = 0;
            continue;
        }

        uart.read(&mut body[..len + 2]).await?;

        let (ck_a, ck_b) = ubx_checksum(&hdr[2..6], &body[..len]);
        if ck_a == body[len] && ck_b == body[len + 1] {
            pkt.cl = hdr[2];
            pkt.id = hdr[3];
            pkt.len = len as u16;
            pkt.data[..len].copy_from_slice(&body[..len]);
            return Ok(());
        }

        hdr[0] = 0;
    }
}

async fn send_valset_and_wait_ack(
    uart: &mut Uart<'static, Async>,
    valset: &ValsetBuilder,
    timeout_ms: u64,
) -> bool {
    let mut tx_buf = [0u8; 104];
    let n = make_ubx_packet(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, valset.payload(), &mut tx_buf);

    if uart.write(&tx_buf[..n]).await.is_err() {
        return false;
    }

    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    let mut pkt = UbxPacket::default();

    while Instant::now() < deadline {
        let remaining = deadline.saturating_duration_since(Instant::now());
        match with_timeout(remaining, read_ubx_packet(uart, &mut pkt)).await {
            Ok(Ok(())) => {
                if pkt.cl == UBX_CLASS_ACK
                    && pkt.id == UBX_ID_ACK_ACK
                    && pkt.len == 2
                    && pkt.data[0] == UBX_CLASS_CFG
                    && pkt.data[1] == UBX_ID_CFG_VALSET
                {
                    return true;
                } else if pkt.cl == UBX_CLASS_ACK
                    && pkt.id == UBX_ID_ACK_NAK
                    && pkt.len == 2
                    && pkt.data[0] == UBX_CLASS_CFG
                    && pkt.data[1] == UBX_ID_CFG_VALSET
                {
                    return false;
                }
            }
            Ok(Err(_)) => {}
            Err(_) => break,
        }
    }

    false
}

async fn sync_baudrate(uart: &mut Uart<'static, Async>) -> bool {
    let mut valset = ValsetBuilder::new(VALSET_LAYER_RAM);
    valset
        .push_u32(CFG_UART1_BAUDRATE, TARGET_BAUDRATE)
        .push_u8(CFG_UART1INPROT_UBX, 1)
        .push_u8(CFG_UART1INPROT_NMEA, 0)
        .push_u8(CFG_UART1OUTPROT_UBX, 1)
        .push_u8(CFG_UART1OUTPROT_NMEA, 0);

    let mut tx_buf = [0u8; 64];
    let n = make_ubx_packet(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, valset.payload(), &mut tx_buf);

    for _attempt in 0..3 {
        for &baud in &BAUDRATES {
            uart.set_baudrate(baud);
            Timer::after_millis(10).await;

            let _ = uart.write(&tx_buf[..n]).await;
            Timer::after_millis(50).await;

            uart.set_baudrate(TARGET_BAUDRATE);
            Timer::after_millis(10).await;

            if send_valset_and_wait_ack(uart, &valset, 250).await {
                return true;
            }
        }
    }

    false
}

async fn configure_timepulse(uart: &mut Uart<'static, Async>) -> bool {
    let mut valset = ValsetBuilder::new(VALSET_LAYER_RAM);
    valset
        .push_u8(CFG_TP_TP1_ENA, 1)
        .push_u8(CFG_TP_PULSE_DEF, 0)
        .push_u8(CFG_TP_PULSE_LENGTH_DEF, 1)
        .push_u32(CFG_TP_PERIOD_TP1, 1_000_000)
        .push_u32(CFG_TP_PERIOD_LOCK_TP1, 1_000_000)
        .push_u32(CFG_TP_LEN_TP1, 2_000)
        .push_u32(CFG_TP_LEN_LOCK_TP1, 1_000)
        .push_u8(CFG_TP_SYNC_GNSS_TP1, 1)
        .push_u8(CFG_TP_USE_LOCKED_TP1, 1)
        .push_u8(CFG_TP_ALIGN_TO_TOW_TP1, 1)
        .push_u8(CFG_TP_POL_TP1, 1)
        .push_u8(CFG_TP_TIMEGRID_TP1, 0);

    send_valset_and_wait_ack(uart, &valset, 250).await
}

async fn configure_navigation(uart: &mut Uart<'static, Async>) -> bool {
    let mut valset = ValsetBuilder::new(VALSET_LAYER_RAM);
    valset
        .push_u8(CFG_NAVSPG_FIXMODE, 3)
        .push_u8(CFG_NAVSPG_DYNMODEL, 8)
        .push_u8(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1)
        .push_u16(CFG_RATE_MEAS, NAV_PERIOD_MS)
        .push_u16(CFG_RATE_NAV, 1)
        .push_u8(CFG_RATE_TIMEREF, 0);

    send_valset_and_wait_ack(uart, &valset, 250).await
}

fn nav_pvt_status(valid: u8, flags: u8, flags2: u8, flags3: u16) -> u16 {
    let mut status = 0_u16;
    if valid & 0x01 != 0 {
        status |= 0x0001;
    }
    if valid & 0x02 != 0 {
        status |= 0x0002;
    }
    if valid & 0x04 != 0 {
        status |= 0x0004;
    }
    if valid & 0x08 != 0 {
        status |= 0x0008;
    }
    if flags & 0x01 != 0 {
        status |= 0x0010;
    }
    if flags & 0x02 != 0 {
        status |= 0x0020;
    }
    if flags & 0x10 != 0 {
        status |= 0x0020;
    }
    if flags & 0x20 != 0 {
        status |= 0x0040;
    }
    if flags & 0x80 != 0 {
        status |= 0x0080;
    }
    if flags2 & 0x20 != 0 {
        status |= 0x0100;
    }
    if flags2 & 0x40 != 0 {
        status |= 0x0200;
    }
    if flags2 & 0x80 != 0 {
        status |= 0x0400;
    }
    if flags3 & 0x0001 != 0 {
        status |= 0x0800;
    }
    if flags3 & 0x0010 != 0 {
        status |= 0x1000;
    }
    status
}

#[embassy_executor::task]
pub async fn pps_task(mut pps: Input<'static>) -> ! {
    loop {
        pps.wait_for_rising_edge().await;
        PPS_SIGNAL.signal(Instant::now().as_micros());
    }
}

#[embassy_executor::task]
pub async fn task(mut uart: Uart<'static, Async>) -> ! {
    Timer::after_millis(100).await;

    loop {
        if !sync_baudrate(&mut uart).await
            || !configure_timepulse(&mut uart).await
            || !configure_navigation(&mut uart).await
        {
            Timer::after_millis(1_000).await;
            continue;
        }

        let mut pkt = UbxPacket::default();
        let mut pps_timestamp_us: u64 = 0;

        loop {
            match read_ubx_packet(&mut uart, &mut pkt).await {
                Ok(()) => {
                    let end_of_packet_us = Instant::now().as_micros();
                    if let Some(ts) = PPS_SIGNAL.try_take() {
                        pps_timestamp_us = ts;
                    }

                    if pkt.cl == UBX_CLASS_NAV
                        && pkt.id == UBX_ID_NAV_PVT
                        && (pkt.len as usize) == UBX_NAV_PVT_LEN
                    {
                        let raw: &[u8; UBX_NAV_PVT_LEN] =
                            pkt.data[..UBX_NAV_PVT_LEN].try_into().unwrap();
                        let pvt = PvtPayload::from_le_bytes(raw);

                        let time_and_fix_valid =
                            (pvt.valid & 0x03) == 0x03 && (pvt.flags & 0x01) != 0;
                        let timestamp_us = rosflight_c_gnss_timestamp(
                            pps_timestamp_us,
                            end_of_packet_us,
                            pvt.nano,
                            time_and_fix_valid,
                        );

                        let mut unix_seconds = unix_seconds_from_utc(
                            pvt.year,
                            pvt.month,
                            pvt.day,
                            pvt.hour,
                            pvt.min,
                            pvt.sec,
                        );
                        let mut unix_nanos = pvt.nano;
                        if unix_nanos < 0 {
                            unix_seconds -= 1;
                            unix_nanos += 1_000_000_000;
                        }

                        SHARED_GNSS_QUEUE.push(Ok(GNSSPacket {
                            header: RosflightPacketHeader {
                                timestamp: timestamp_us,
                                status: nav_pvt_status(
                                    pvt.valid,
                                    pvt.flags,
                                    pvt.flags2,
                                    pvt.flags3,
                                ),
                            },
                            unix_seconds,
                            unix_nanos,
                            // Preserve full f64 precision for geographic coordinates.
                            lat: (pvt.lat as f64) * 1.0e-7,
                            lon: (pvt.lon as f64) * 1.0e-7,
                            height_msl: (pvt.h_msl as f32) * 1.0e-3,
                            vel_n: (pvt.vel_n as f32) * 1.0e-3,
                            vel_e: (pvt.vel_e as f32) * 1.0e-3,
                            vel_d: (pvt.vel_d as f32) * 1.0e-3,
                            h_acc: (pvt.h_acc as f32) * 1.0e-3,
                            v_acc: (pvt.v_acc as f32) * 1.0e-3,
                            s_acc: (pvt.s_acc as f32) * 1.0e-3,
                            month: pvt.month,
                            year: pvt.year,
                            day: pvt.day,
                            hour: pvt.hour,
                            min: pvt.min,
                            sec: pvt.sec,
                            nano: pvt.nano,
                            fix_type: GNSSFixType::from_u8(pvt.fix_type),
                            num_sats: pvt.num_sv,
                            mag_dec: (pvt.mag_dec as f32) * 1.745_329_251_994_329_6e-4,
                            time_correction: 0,
                        }));
                    }
                }
                Err(_) => {
                    Timer::after_millis(5).await;
                }
            }
        }
    }
}
