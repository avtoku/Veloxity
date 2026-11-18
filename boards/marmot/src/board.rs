use embassy_rp::gpio::Output;
use embassy_time::{Duration, Instant};
use veloxity_core::board::{BoardIo, SerialRxFrame, SerialTxPriority};
use veloxity_core::comm::messages::messages::DownlinkMessage;
use veloxity_core::errors;
use veloxity_core::math::FlightFloat;
use veloxity_core::packets::ParamPacket;
use veloxity_core::params::Params;
use veloxity_core::sensors::SensorBus;

use crate::comms::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
use crate::qspi_storage::{
    QSPI_READ_REQUEST_SIGNAL, QSPI_READ_SIGNAL, QSPI_WRITE_SIGNAL, decode_params, encode_params,
};
use crate::sensors::{
    SHARED_BARO_QUEUE, SHARED_BATTERY_QUEUE, SHARED_GNSS_QUEUE, SHARED_IMU_QUEUE,
    SHARED_MAG_QUEUE, SHARED_PITOT_QUEUE, SHARED_RC_QUEUE, battery,
};

const PARAM_STORAGE_TIMEOUT: Duration = Duration::from_secs(2);

pub struct BoardPins {
    pub led_red: Output<'static>,
    pub led_green: Output<'static>,
    pub led_blue: Output<'static>,
    pub tp4: Output<'static>,
    pub tp5: Output<'static>,
    pub tp6: Output<'static>,
}

pub struct Board {
    mavlink: SharedMavlinkMailbox,
    pins: BoardPins,
    boot_time: Instant,
    last_imu_seq: Option<u32>,
    imu_seq_gaps: u32,
    qspi_request_id: u64,
    pending_reboot: bool,
}

impl Board {
    pub fn new(pins: BoardPins) -> Self {
        Self {
            mavlink: SHARED_MAVLINK_MAILBOX,
            pins,
            boot_time: Instant::now(),
            last_imu_seq: None,
            imu_seq_gaps: 0,
            qspi_request_id: 0,
            pending_reboot: false,
        }
    }

    pub fn mavlink_mailbox(&self) -> SharedMavlinkMailbox {
        self.mavlink
    }

    fn next_qspi_request_id(&mut self) -> u64 {
        self.qspi_request_id = self.qspi_request_id.wrapping_add(1);
        self.qspi_request_id
    }

    fn drain_imu_into<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        let Some(imu) = SHARED_IMU_QUEUE.take_latest() else {
            return;
        };

        if let Some(last_seq) = self.last_imu_seq {
            let expected = last_seq.wrapping_add(1);
            if imu.seq != expected {
                self.imu_seq_gaps = self
                    .imu_seq_gaps
                    .wrapping_add(imu.seq.wrapping_sub(expected).max(1));
            }
        }
        self.last_imu_seq = Some(imu.seq);
        sensors.imu = Some(Ok(imu.cast()));
    }

    fn drain_service_into<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        if let Some(baro) = SHARED_BARO_QUEUE.take_latest() {
            sensors.baro = Some(baro);
        }
        if let Some(mag) = SHARED_MAG_QUEUE.take_latest() {
            sensors.mag = Some(mag);
        }
        if let Some(pitot) = SHARED_PITOT_QUEUE.take_latest() {
            sensors.pitot = Some(pitot);
        }
        if let Some(gnss) = SHARED_GNSS_QUEUE.take_latest() {
            sensors.gnss = Some(gnss);
        }
        if let Some(battery) = SHARED_BATTERY_QUEUE.take_latest() {
            sensors.battery = Some(battery);
        }
        if let Some(rc) = SHARED_RC_QUEUE.take_latest() {
            sensors.rc = Some(rc);
        }
    }
}

fn wait_for_qspi_response(request_id: u64) -> Option<ParamPacket> {
    let start = Instant::now();
    while start.elapsed() < PARAM_STORAGE_TIMEOUT {
        match QSPI_READ_SIGNAL.try_take() {
            Some(Ok(packet)) if packet.header.timestamp == request_id => return Some(packet),
            Some(Ok(_)) => {}
            Some(Err(_)) => return None,
            None => {}
        }
        core::hint::spin_loop();
    }
    None
}

impl BoardIo for Board {
    fn update_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        self.drain_imu_into(sensors);
        self.drain_service_into(sensors);
    }

    fn imu_pending(&self) -> bool {
        SHARED_IMU_QUEUE.has_pending()
    }

    fn update_imu_sensor<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        self.drain_imu_into(sensors);
    }

    fn update_service_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        self.drain_service_into(sensors);
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.read_into(buf)))
    }

    fn serial_rx_frame_read(&mut self) -> Option<Result<SerialRxFrame, errors::TelemError>> {
        self.mavlink.pop_rx_frame().map(Ok)
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        self.serial_tx_write_priority(bytes, SerialTxPriority::DEFAULT)
    }

    fn serial_tx_write_priority(
        &mut self,
        bytes: &[u8],
        priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.write_from_priority(bytes, priority)))
    }

    fn serial_tx_enqueue_downlink(
        &mut self,
        system_id: u8,
        msg: DownlinkMessage,
        priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.enqueue_downlink(system_id, msg, priority)))
    }

    fn serial_rx_pending(&self) -> bool {
        self.mavlink.has_pending_rx_frame()
    }

    fn clock_millis(&self) -> u32 {
        (self.clock_micros() / 1000) as u32
    }

    fn clock_micros(&self) -> u64 {
        self.boot_time.elapsed().as_micros()
    }

    fn set_test_pin_1(&mut self, high: bool) {
        if high {
            self.pins.tp4.set_high();
        } else {
            self.pins.tp4.set_low();
        }
    }

    fn set_test_pin_2(&mut self, high: bool) {
        if high {
            self.pins.tp5.set_high();
        } else {
            self.pins.tp5.set_low();
        }
    }

    fn set_test_pin_3(&mut self, high: bool) {
        if high {
            self.pins.tp6.set_high();
        } else {
            self.pins.tp6.set_low();
        }
    }

    fn configure_battery_monitor(&mut self, voltage_multiplier: f32, current_multiplier: f32) {
        battery::configure_multipliers(voltage_multiplier, current_multiplier);
    }

    fn read_params(&mut self, params: &mut Params) -> bool {
        let request_id = self.next_qspi_request_id();
        QSPI_READ_SIGNAL.reset();
        QSPI_READ_REQUEST_SIGNAL.signal(request_id);

        let Some(packet) = wait_for_qspi_response(request_id) else {
            return false;
        };
        let Some(stored) = decode_params(&packet) else {
            return false;
        };
        *params = stored;
        true
    }

    fn write_params(&mut self, params: &Params) -> bool {
        let Some(mut packet) = encode_params(params) else {
            return false;
        };
        let request_id = self.next_qspi_request_id();
        packet.header.timestamp = request_id;

        QSPI_READ_SIGNAL.reset();
        QSPI_WRITE_SIGNAL.signal(Ok(packet));

        matches!(
            wait_for_qspi_response(request_id),
            Some(packet) if packet.header.status == 1
        )
    }

    // RGB LED outputs are active-low on Marmot (Level::High = off, Level::Low = on).
    fn led0_on(&mut self) {
        self.pins.led_green.set_low();
    }

    fn led0_off(&mut self) {
        self.pins.led_green.set_high();
    }

    fn led0_toggle(&mut self) {
        self.pins.led_green.toggle();
    }

    fn led1_on(&mut self) {
        self.pins.led_red.set_low();
    }

    fn led1_off(&mut self) {
        self.pins.led_red.set_high();
    }

    fn led1_toggle(&mut self) {
        self.pins.led_red.toggle();
    }

    fn reboot(&mut self) -> bool {
        self.pending_reboot = true;
        true
    }

    fn run_deferred_board_actions(&mut self) {
        if self.pending_reboot {
            self.pending_reboot = false;
            let start = Instant::now();
            while start.elapsed() < Duration::from_millis(20) {
                core::hint::spin_loop();
            }
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
