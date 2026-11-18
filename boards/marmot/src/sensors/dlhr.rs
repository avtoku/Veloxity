use embassy_rp::gpio::Input;
use embassy_rp::peripherals::PIO0;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use veloxity_core::errors::SensorError;
use veloxity_core::packets::{PitotPacket, RosflightPacketHeader};

use crate::pio_i2c::PioI2c;

use super::SHARED_PITOT_QUEUE;

pub const DLHR_ADDR: u8 = 0x29;
pub const DLHR_DATA_LEN: usize = 4;
const DLHR_SINGLE_MEASURE_CMD: u8 = 0xAC;

#[embassy_executor::task]
pub async fn task(mut i2c: PioI2c<'static, PIO0, 2>, mut drdy: Input<'static>, fs: f32) -> ! {
    loop {
        let mut status = [0u8];
        if i2c.read(DLHR_ADDR, &mut status).await.is_err() || status[0] != 0x40 {
            SHARED_PITOT_QUEUE.push(Err(SensorError::GenericSensorError("DLHR init failed")));
            Timer::after_millis(1_000).await;
            continue;
        }

        let mut data = [0u8; DLHR_DATA_LEN];
        let mut timer = Ticker::every(Duration::from_millis(10));

        loop {
            timer.next().await;

            if i2c.write(DLHR_ADDR, &[DLHR_SINGLE_MEASURE_CMD]).await.is_err() {
                SHARED_PITOT_QUEUE.push(Err(SensorError::GenericSensorError("DLHR trigger failed")));
                continue;
            }
            Timer::after_micros(10).await;

            if drdy.is_low()
                && with_timeout(Duration::from_millis(50), drdy.wait_for_rising_edge())
                    .await
                    .is_err()
            {
                SHARED_PITOT_QUEUE.push(Err(SensorError::GenericSensorError("DLHR DRDY timeout")));
                continue;
            }

            let timestamp_us = Instant::now().as_micros();
            if i2c.read(DLHR_ADDR, &mut data).await.is_err() {
                SHARED_PITOT_QUEUE.push(Err(SensorError::GenericSensorError("DLHR read failed")));
                continue;
            }

            if data[0] != 0x40 {
                SHARED_PITOT_QUEUE.push(Err(SensorError::GenericSensorError("DLHR invalid status")));
                continue;
            }

            let u32_pressure =
                u32::from(data[1]) << 16 | u32::from(data[2]) << 8 | u32::from(data[3]);
            let differential_pressure =
                1.25 * fs * ((u32_pressure as f32) / 16_777_216.0 - 0.1);

            SHARED_PITOT_QUEUE.push(Ok(PitotPacket {
                header: RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: 0,
                },
                differential_pressure,
                temperature: 0.0,
                indicated_airspeed: 0.0,
            }));
        }
    }
}
