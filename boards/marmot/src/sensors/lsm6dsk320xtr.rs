use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Spi};
use embassy_time::{Instant, Timer};
use veloxity_core::packets::{ImuPacket, RosflightPacketHeader};

use super::SHARED_IMU_QUEUE;

pub const LSM_READ: u8 = 0x80;
pub const LSM_WRITE: u8 = 0x00;

pub const LSM_IF_CFG: u8 = 0x03;
pub const LSM_INT1_CTRL: u8 = 0x0D;
pub const LSM_WHOAMI_REG: u8 = 0x0F;
pub const LSM_WHOAMI_LSM6DSK320X: u8 = 0x75;
pub const LSM_WHOAMI_LSM6DSV320X: u8 = 0x73;

pub const LSM_CTRL1_XL: u8 = 0x10;
pub const LSM_CTRL2_XL: u8 = 0x11;
pub const LSM_CTRL3_C: u8 = 0x12;
pub const LSM_CTRL4_C: u8 = 0x13;
pub const LSM_CTRL6_C: u8 = 0x15;
pub const LSM_CTRL8_XL: u8 = 0x17;
pub const LSM_CTRL1_XL_HG: u8 = 0x4E;
pub const LSM_HAODR_CFG: u8 = 0x62;

pub const LSM_DATA_REG: u8 = 0x22;
pub const LSM_DATA_LEN: usize = 12;

const GYRO_SCALE_250DPS_RAD_S: f32 = 0.00015271631;
const ACCEL_SCALE_16G_M_S2: f32 = 0.0047856452;

async fn spi_write_reg(
    spi: &mut Spi<'static, SPI0, Async>,
    csn: &mut Output<'static>,
    reg: u8,
    val: u8,
) -> Result<(), embassy_rp::spi::Error> {
    let buf = [reg | LSM_WRITE, val];
    csn.set_low();
    let res = spi.write(&buf).await;
    csn.set_high();
    res
}

async fn spi_read_data(
    spi: &mut Spi<'static, SPI0, Async>,
    csn: &mut Output<'static>,
    reg: u8,
    data: &mut [u8],
) -> Result<(), embassy_rp::spi::Error> {
    data.fill(0);
    data[0] = reg | LSM_READ;
    csn.set_low();
    let res = spi.transfer_in_place(data).await;
    csn.set_high();
    res
}

async fn configure_imu(
    spi: &mut Spi<'static, SPI0, Async>,
    csn: &mut Output<'static>,
) -> Result<(), ()> {
    let mut whoami = [0u8; 2];
    spi_read_data(spi, csn, LSM_WHOAMI_REG, &mut whoami)
        .await
        .map_err(|_| ())?;
    if whoami[1] != LSM_WHOAMI_LSM6DSK320X && whoami[1] != LSM_WHOAMI_LSM6DSV320X {
        return Err(());
    }

    // Ensure all sensors are in Power-Down before configuring HAODR & interrupts
    spi_write_reg(spi, csn, LSM_CTRL1_XL, 0x00)
        .await
        .map_err(|_| ())?;
    spi_write_reg(spi, csn, LSM_CTRL2_XL, 0x00)
        .await
        .map_err(|_| ())?;
    spi_write_reg(spi, csn, LSM_CTRL1_XL_HG, 0x00)
        .await
        .map_err(|_| ())?;

    // Enable Block Data Update (BDU=1) and Auto-Increment (IF_INC=1)
    spi_write_reg(spi, csn, LSM_CTRL3_C, 0x44)
        .await
        .map_err(|_| ())?;
    // Push-Pull, Active-High interrupts
    spi_write_reg(spi, csn, LSM_IF_CFG, 0x00)
        .await
        .map_err(|_| ())?;
    // DRDY_MASK=1, DRDY_PULSED=1
    spi_write_reg(spi, csn, LSM_CTRL4_C, 0x0A)
        .await
        .map_err(|_| ())?;
    // Route gyro DRDY to INT1
    spi_write_reg(spi, csn, LSM_INT1_CTRL, 0x02)
        .await
        .map_err(|_| ())?;
    // Full-scale ranges: ±250 dps gyro, ±16g accel
    spi_write_reg(spi, csn, LSM_CTRL6_C, 0x09)
        .await
        .map_err(|_| ())?;
    spi_write_reg(spi, csn, LSM_CTRL8_XL, 0x03)
        .await
        .map_err(|_| ())?;
    // HAODR Set 2 (100/200/400/800/960 Hz)
    spi_write_reg(spi, csn, LSM_HAODR_CFG, 0x01)
        .await
        .map_err(|_| ())?;
    // 960 Hz HAODR for accel and gyro
    spi_write_reg(spi, csn, LSM_CTRL1_XL, 0x19)
        .await
        .map_err(|_| ())?;
    spi_write_reg(spi, csn, LSM_CTRL2_XL, 0x19)
        .await
        .map_err(|_| ())?;
    spi_write_reg(spi, csn, LSM_CTRL3_C, 0x44)
        .await
        .map_err(|_| ())?;

    Ok(())
}

#[embassy_executor::task]
pub async fn task(
    mut spi: Spi<'static, SPI0, Async>,
    mut csn: Output<'static>,
    mut drdy: Input<'static>,
) -> ! {
    let mut seq: u32 = 0;

    loop {
        if configure_imu(&mut spi, &mut csn).await.is_err() {
            Timer::after_millis(1_000).await;
            continue;
        }

        let mut data = [0u8; LSM_DATA_LEN + 1];
        loop {
            drdy.wait_for_rising_edge().await;
            let timestamp_us = Instant::now().as_micros();

            if spi_read_data(&mut spi, &mut csn, LSM_DATA_REG, &mut data)
                .await
                .is_err()
            {
                break;
            }

            let g = [
                i16::from_le_bytes([data[1], data[2]]),
                i16::from_le_bytes([data[3], data[4]]),
                i16::from_le_bytes([data[5], data[6]]),
            ];
            let a = [
                i16::from_le_bytes([data[7], data[8]]),
                i16::from_le_bytes([data[9], data[10]]),
                i16::from_le_bytes([data[11], data[12]]),
            ];

            let packet = ImuPacket {
                header: RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: 0,
                },
                accel: a.map(|x| (x as f32) * ACCEL_SCALE_16G_M_S2),
                gyro: g.map(|x| (x as f32) * GYRO_SCALE_250DPS_RAD_S),
                temperature: 0.0,
                seq,
            };
            seq = seq.wrapping_add(1);
            SHARED_IMU_QUEUE.push(packet);
        }
    }
}
