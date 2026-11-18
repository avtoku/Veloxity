use embassy_rp::adc::{Adc, Async, Channel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};
use veloxity_core::battery::BatteryMonitorCalibration;
use veloxity_core::errors::SensorError;
use veloxity_core::packets::{BatteryPacket, RosflightPacketHeader};

use super::SHARED_BATTERY_QUEUE;

const SAMPLE_PERIOD: Duration = Duration::from_millis(100);
const VDDA_NOMINAL_V: f32 = 3.3;

// Default neutral scale until board parameters / voltage-current multipliers are configured.
const DEFAULT_BOARD_CALIBRATION: BatteryMonitorCalibration =
    BatteryMonitorCalibration::new(1.0, 0.0, 1.0, 0.0);

#[derive(Debug, Clone, Copy)]
struct BatteryMultipliers {
    voltage: f32,
    current: f32,
}

static MULTIPLIER_SIGNAL: Signal<CriticalSectionRawMutex, BatteryMultipliers> = Signal::new();

pub fn configure_multipliers(voltage: f32, current: f32) {
    MULTIPLIER_SIGNAL.signal(BatteryMultipliers { voltage, current });
}

#[inline]
fn scale_12_bit_to_16_bit(raw_12_bit: u16) -> u16 {
    let clamped = raw_12_bit & 0x0FFF;
    (clamped << 4) | (clamped >> 8)
}

#[embassy_executor::task]
pub async fn task(
    mut adc: Adc<'static, Async>,
    mut vbat_ch: Channel<'static>,
    mut ibat_ch: Channel<'static>,
) -> ! {
    let mut calibration = DEFAULT_BOARD_CALIBRATION;
    let mut ticker = Ticker::every(SAMPLE_PERIOD);

    loop {
        ticker.next().await;

        if let Some(multipliers) = MULTIPLIER_SIGNAL.try_take() {
            calibration.apply_multipliers(multipliers.voltage, multipliers.current);
        }

        let v_raw = match adc.read(&mut vbat_ch).await {
            Ok(raw) => raw,
            Err(_) => {
                SHARED_BATTERY_QUEUE
                    .push(Err(SensorError::GenericSensorError("ADC VBAT read failed")));
                continue;
            }
        };
        let i_raw = match adc.read(&mut ibat_ch).await {
            Ok(raw) => raw,
            Err(_) => {
                SHARED_BATTERY_QUEUE
                    .push(Err(SensorError::GenericSensorError("ADC IBAT read failed")));
                continue;
            }
        };

        let timestamp = Instant::now().as_micros();
        let voltage = calibration
            .voltage
            .convert_16_bit_sample(scale_12_bit_to_16_bit(v_raw), VDDA_NOMINAL_V);
        let current = calibration
            .current
            .convert_16_bit_sample(scale_12_bit_to_16_bit(i_raw), VDDA_NOMINAL_V);

        SHARED_BATTERY_QUEUE.push(Ok(BatteryPacket {
            header: RosflightPacketHeader {
                timestamp,
                status: 0,
            },
            voltage,
            current,
        }));
    }
}
