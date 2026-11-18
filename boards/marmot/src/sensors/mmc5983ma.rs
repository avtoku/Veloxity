use embassy_rp::gpio::Input;
use embassy_rp::peripherals::PIO0;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use veloxity_core::errors::SensorError;
use veloxity_core::packets::{MagPacket, RosflightPacketHeader};

use crate::pio_i2c::PioI2c;

use super::SHARED_MAG_QUEUE;

pub const MMC_ADDR: u8 = 0x30;
pub const MMC_REG_ID: u8 = 0x2F;
pub const MMC_ID_VALUE: u8 = 0x30;

pub const MMC_REG_DATA: u8 = 0x00;
pub const MMC_DATA_LEN: usize = 7;

pub const MMC_REG_STATUS: u8 = 0x08;
const STATUS_MEAS_M_DONE: u8 = 1 << 0;
const STATUS_MEAS_T_DONE: u8 = 1 << 1;

pub const MMC_REG_CTRL0: u8 = 0x09;
const CTRL0_INT_MEAS_DONE_EN: u8 = 1 << 2;
const CTRL0_AUTO_SR_EN: u8 = 1 << 5;
const CTRL0_SET: u8 = 1 << 3;
pub const MMC_CTRL0_START_VALUE: u8 = CTRL0_AUTO_SR_EN | CTRL0_INT_MEAS_DONE_EN;

pub const MMC_REG_CTRL1: u8 = 0x0A;
const CTRL1_BW_200HZ: u8 = 0x01;
pub const MMC_CTRL1_VALUE: u8 = CTRL1_BW_200HZ;

pub const MMC_REG_CTRL2: u8 = 0x0B;
const CTRL2_CM_FREQ_200HZ: u8 = 0x06;
const CTRL2_CMM_EN: u8 = 1 << 3;
const CTRL2_PRD_SET_100: u8 = 0x03 << 4;
const CTRL2_EN_PRD_SET: u8 = 1 << 7;
pub const MMC_CTRL2_VALUE: u8 =
    CTRL2_CMM_EN | CTRL2_PRD_SET_100 | CTRL2_EN_PRD_SET | CTRL2_CM_FREQ_200HZ;

pub const MMC_REG_CTRL3: u8 = 0x0C;
pub const MMC_CTRL3_VALUE: u8 = 0x00;

const NULL_OFFSET: i32 = 131_072; // 0x20000
const SCALE_H_TESLA: f32 = 6.103515625e-9; // (1e-4 T/G) / (16384 LSB/G)

async fn configure_mmc5983ma(i2c: &mut PioI2c<'static, PIO0, 0>) -> Result<(), ()> {
    let mut id = [0u8];
    i2c.write_read(MMC_ADDR, &[MMC_REG_ID], &mut id)
        .await
        .map_err(|_| ())?;
    if id[0] != MMC_ID_VALUE {
        return Err(());
    }

    i2c.write(MMC_ADDR, &[MMC_REG_CTRL1, 0x80])
        .await
        .map_err(|_| ())?;
    Timer::after_millis(15).await;

    let _ = i2c.write(MMC_ADDR, &[MMC_REG_CTRL0, CTRL0_SET]).await;
    Timer::after_millis(1).await;

    i2c.write(
        MMC_ADDR,
        &[MMC_REG_STATUS, STATUS_MEAS_M_DONE | STATUS_MEAS_T_DONE],
    )
    .await
    .map_err(|_| ())?;
    i2c.write(MMC_ADDR, &[MMC_REG_CTRL1, MMC_CTRL1_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write(MMC_ADDR, &[MMC_REG_CTRL3, MMC_CTRL3_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write(MMC_ADDR, &[MMC_REG_CTRL0, MMC_CTRL0_START_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write(MMC_ADDR, &[MMC_REG_CTRL2, MMC_CTRL2_VALUE])
        .await
        .map_err(|_| ())?;

    Ok(())
}

#[embassy_executor::task]
pub async fn task(mut i2c: PioI2c<'static, PIO0, 0>, mut drdy: Input<'static>) -> ! {
    loop {
        if configure_mmc5983ma(&mut i2c).await.is_err() {
            SHARED_MAG_QUEUE.push(Err(SensorError::GenericSensorError("MMC5983MA init failed")));
            Timer::after_millis(1_000).await;
            continue;
        }

        let mut data = [0u8; MMC_DATA_LEN];
        let mut previous_raw_h: Option<[u32; 3]> = None;

        loop {
            if !drdy.is_high()
                && with_timeout(Duration::from_millis(100), drdy.wait_for_rising_edge())
                    .await
                    .is_err()
            {
                SHARED_MAG_QUEUE.push(Err(SensorError::GenericSensorError(
                    "MMC5983MA DRDY timeout",
                )));
                break;
            }
            let timestamp_us = Instant::now().as_micros();

            if i2c.write_read(MMC_ADDR, &[MMC_REG_DATA], &mut data)
                .await
                .is_err()
            {
                SHARED_MAG_QUEUE.push(Err(SensorError::GenericSensorError("MMC5983MA read failed")));
                break;
            }

            if i2c.write(MMC_ADDR, &[MMC_REG_STATUS, STATUS_MEAS_M_DONE])
                .await
                .is_err()
            {
                SHARED_MAG_QUEUE.push(Err(SensorError::GenericSensorError(
                    "MMC5983MA status clear failed",
                )));
                break;
            }

            let raw_h = [
                ((data[0] as u32) << 10) | ((data[1] as u32) << 2) | ((data[6] as u32) >> 6),
                ((data[2] as u32) << 10)
                    | ((data[3] as u32) << 2)
                    | (((data[6] as u32) >> 4) & 0x03),
                ((data[4] as u32) << 10)
                    | ((data[5] as u32) << 2)
                    | (((data[6] as u32) >> 2) & 0x03),
            ];

            let Some(prev) = previous_raw_h else {
                previous_raw_h = Some(raw_h);
                continue;
            };
            previous_raw_h = Some(raw_h);

            let avg_h = [
                (raw_h[0] + prev[0]) / 2,
                (raw_h[1] + prev[1]) / 2,
                (raw_h[2] + prev[2]) / 2,
            ];
            let flux = avg_h.map(|x| ((x as i32) - NULL_OFFSET) as f32 * SCALE_H_TESLA);

            SHARED_MAG_QUEUE.push(Ok(MagPacket {
                header: RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: 0,
                },
                flux,
                temperature: 0.0,
            }));
        }
    }
}
