use embassy_futures::select::{Either, select};
use embassy_rp::peripherals::PIO0;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use veloxity_core::errors;
use veloxity_core::packets::{PARAM_PACKET_SIZE, ParamPacket, RosflightPacketHeader};
use veloxity_core::params::Params;
use veloxity_core::params::storage::{
    ROSFLIGHT_C_PARAM_STORAGE_SIZE, decode_rosflight_c_params, encode_rosflight_c_params,
};

use crate::pio_qspi::{PioQspi, QspiError, W25Q_SECTOR_SIZE};

const PARAM_BASE_ADDR: u32 = 0x0000_0000;
const CHECKSUM_SIZE: usize = size_of::<u32>();
const IMAGE_AND_CRC_SIZE: usize = ROSFLIGHT_C_PARAM_STORAGE_SIZE + CHECKSUM_SIZE;

const _: () = assert!(ROSFLIGHT_C_PARAM_STORAGE_SIZE == PARAM_PACKET_SIZE);
const _: () = assert!(IMAGE_AND_CRC_SIZE <= 2 * W25Q_SECTOR_SIZE);

pub static QSPI_WRITE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<ParamPacket, errors::SensorError>,
> = Signal::new();

pub static QSPI_READ_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<ParamPacket, errors::SensorError>,
> = Signal::new();

pub static QSPI_READ_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, u64> = Signal::new();

#[derive(Debug)]
pub enum QspiStorageError {
    Device(QspiError),
    ChecksumMismatch,
}

impl From<QspiError> for QspiStorageError {
    fn from(error: QspiError) -> Self {
        Self::Device(error)
    }
}

/// Encodes the C-compatible subset into ROSflight 2.0's persisted `params_t`.
pub fn encode_params(params: &Params) -> Option<ParamPacket> {
    let mut packet = ParamPacket::default();
    encode_rosflight_c_params(params, &mut packet.values).then_some(packet)
}

/// Decodes ROSflight 2.0's persisted `params_t` into the Rust parameter table.
pub fn decode_params(packet: &ParamPacket) -> Option<Params> {
    if packet.header.status != 1 {
        return None;
    }
    decode_rosflight_c_params(&packet.values)
}

pub struct QspiStorage {
    flash: PioQspi<'static, PIO0, 1>,
    ready: bool,
    scratch: [u8; IMAGE_AND_CRC_SIZE],
}

impl QspiStorage {
    pub fn new(flash: PioQspi<'static, PIO0, 1>) -> Self {
        Self {
            flash,
            ready: false,
            scratch: [0u8; IMAGE_AND_CRC_SIZE],
        }
    }

    async fn ensure_ready(&mut self) -> Result<(), QspiStorageError> {
        if !self.ready {
            self.flash.init().await?;
            self.ready = true;
        }
        Ok(())
    }

    pub async fn read(
        &mut self,
        destination: &mut [u8; PARAM_PACKET_SIZE],
    ) -> Result<(), QspiStorageError> {
        self.ensure_ready().await?;
        self.flash.read(PARAM_BASE_ADDR, &mut self.scratch).await?;

        let stored_crc = u32::from_le_bytes(
            self.scratch[PARAM_PACKET_SIZE..IMAGE_AND_CRC_SIZE]
                .try_into()
                .unwrap(),
        );
        if stored_crc != rosflight_crc32(&self.scratch[..PARAM_PACKET_SIZE]) {
            return Err(QspiStorageError::ChecksumMismatch);
        }

        destination.copy_from_slice(&self.scratch[..PARAM_PACKET_SIZE]);
        Ok(())
    }

    pub async fn write(
        &mut self,
        source: &[u8; PARAM_PACKET_SIZE],
    ) -> Result<(), QspiStorageError> {
        self.ensure_ready().await?;
        self.scratch[..PARAM_PACKET_SIZE].copy_from_slice(source);
        self.scratch[PARAM_PACKET_SIZE..IMAGE_AND_CRC_SIZE]
            .copy_from_slice(&rosflight_crc32(source).to_le_bytes());

        self.flash.erase_sector_4k(PARAM_BASE_ADDR).await?;
        self.flash
            .erase_sector_4k(PARAM_BASE_ADDR + W25Q_SECTOR_SIZE as u32)
            .await?;
        self.flash.write(PARAM_BASE_ADDR, &self.scratch).await?;
        Ok(())
    }

    pub async fn run(&mut self) -> ! {
        let _ = self.ensure_ready().await;

        loop {
            match select(QSPI_READ_REQUEST_SIGNAL.wait(), QSPI_WRITE_SIGNAL.wait()).await {
                Either::First(request_id) => {
                    let mut packet = ParamPacket {
                        header: RosflightPacketHeader {
                            timestamp: request_id,
                            status: 0,
                        },
                        values: [0u8; PARAM_PACKET_SIZE],
                    };
                    packet.header.status = u16::from(self.read(&mut packet.values).await.is_ok());
                    QSPI_READ_SIGNAL.signal(Ok(packet));
                }
                Either::Second(Ok(mut packet)) => {
                    packet.header.status = u16::from(self.write(&packet.values).await.is_ok());
                    QSPI_READ_SIGNAL.signal(Ok(packet));
                }
                Either::Second(Err(error)) => QSPI_READ_SIGNAL.signal(Err(error)),
            }
        }
    }
}

// Same CRC-32 used by ROSflight / STM32 parameter storage:
// polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final XOR.
fn rosflight_crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;

    for byte in bytes {
        crc ^= u32::from(*byte) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ 0x04C1_1DB7
            } else {
                crc << 1
            };
        }
    }

    crc
}

#[embassy_executor::task]
pub async fn task(flash: PioQspi<'static, PIO0, 1>) -> ! {
    let mut storage = QspiStorage::new(flash);
    storage.run().await
}
