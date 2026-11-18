use embassy_rp::gpio::Input;
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use veloxity_core::errors::SensorError;
use veloxity_core::packets::{BaroPacket, RosflightPacketHeader};

use super::SHARED_BARO_QUEUE;

pub const BMP_ADDR: u8 = 0x46;

pub const BMP_REG_CHIP_ID: u8 = 0x01;
pub const BMP_CHIP_ID: u8 = 0x50;
pub const BMP_REV_ID: u8 = 0x32;

pub const BMP_REG_INT_CONFIG: u8 = 0x14;
pub const BMP_REG_INT_CONFIG_VALUE: u8 = 0x3A;

pub const BMP_REG_INT_SOURCE: u8 = 0x15;
pub const BMP_REG_INT_SOURCE_VALUE: u8 = 0x01;

pub const BMP_REG_PRESSURE_DATA: u8 = 0x20;
pub const BMP_DATA_LEN: usize = 3;

pub const BMP_REG_INT_STATUS: u8 = 0x27;
pub const BMP_REG_OSR_CONFIG: u8 = 0x36;
pub const BMP_REG_OSR_CONFIG_VALUE: u8 = 0x58;
pub const BMP_REG_ODR_CONFIG: u8 = 0x37;
pub const BMP_REG_ODR_CONFIG_VALUE: u8 = 0xA3;
pub const BMP_REG_ODR_CONFIG_STANDBY_VALUE: u8 = 0xA0;
pub const BMP_REG_OSR_EFF: u8 = 0x38;

async fn configure_bmp581(i2c: &mut I2c<'static, I2C0, Async>) -> Result<(), ()> {
    let mut id = [0u8; 2];
    i2c.write_read_async(BMP_ADDR, [BMP_REG_CHIP_ID], &mut id)
        .await
        .map_err(|_| ())?;
    if id[0] != BMP_CHIP_ID || id[1] != BMP_REV_ID {
        return Err(());
    }

    i2c.write_async(BMP_ADDR, [BMP_REG_OSR_CONFIG, BMP_REG_OSR_CONFIG_VALUE])
        .await
        .map_err(|_| ())?;

    let mut osr = [0u8];
    i2c.write_read_async(BMP_ADDR, [BMP_REG_OSR_EFF], &mut osr)
        .await
        .map_err(|_| ())?;

    let mut status = [0u8];
    i2c.write_read_async(BMP_ADDR, [BMP_REG_INT_STATUS], &mut status)
        .await
        .map_err(|_| ())?;

    i2c.write_async(BMP_ADDR, [BMP_REG_INT_CONFIG, BMP_REG_INT_CONFIG_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write_async(BMP_ADDR, [BMP_REG_INT_SOURCE, BMP_REG_INT_SOURCE_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write_async(
        BMP_ADDR,
        [BMP_REG_ODR_CONFIG, BMP_REG_ODR_CONFIG_STANDBY_VALUE],
    )
    .await
    .map_err(|_| ())?;

    i2c.write_async(BMP_ADDR, [BMP_REG_INT_CONFIG, BMP_REG_INT_CONFIG_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write_async(BMP_ADDR, [BMP_REG_INT_SOURCE, BMP_REG_INT_SOURCE_VALUE])
        .await
        .map_err(|_| ())?;
    i2c.write_async(BMP_ADDR, [BMP_REG_ODR_CONFIG, BMP_REG_ODR_CONFIG_VALUE])
        .await
        .map_err(|_| ())?;

    i2c.write_async(BMP_ADDR, [BMP_REG_PRESSURE_DATA])
        .await
        .map_err(|_| ())?;

    Ok(())
}

#[embassy_executor::task]
pub async fn task(mut i2c: I2c<'static, I2C0, Async>, mut drdy: Input<'static>) -> ! {
    loop {
        if configure_bmp581(&mut i2c).await.is_err() {
            SHARED_BARO_QUEUE.push(Err(SensorError::GenericSensorError("BMP581 init failed")));
            Timer::after_millis(1_000).await;
            continue;
        }

        let mut data = [0u8; BMP_DATA_LEN];
        loop {
            if with_timeout(Duration::from_millis(100), drdy.wait_for_rising_edge())
                .await
                .is_err()
            {
                SHARED_BARO_QUEUE.push(Err(SensorError::GenericSensorError("BMP581 DRDY timeout")));
                break;
            }
            let timestamp_us = Instant::now().as_micros();

            if i2c.read_async(BMP_ADDR, &mut data).await.is_err() {
                SHARED_BARO_QUEUE.push(Err(SensorError::GenericSensorError("BMP581 read failed")));
                break;
            }

            let raw_press = (data[2] as i32) << 16 | (data[1] as i32) << 8 | (data[0] as i32);
            let pressure = (raw_press as f32) / 64.0;

            SHARED_BARO_QUEUE.push(Ok(BaroPacket {
                header: RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: 0,
                },
                pressure,
                temperature: 0.0,
                altitude: 0.0,
            }));
        }
    }
}
