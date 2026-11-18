pub mod battery;
pub mod bmp581;
pub mod dlhr;
pub mod lsm6dsk320xtr;
pub mod mmc5983ma;
pub mod sbus;
pub mod ubx;

use core::cell::RefCell;

use critical_section::Mutex;
use veloxity_core::{
    errors::SensorError,
    packets::{
        BaroPacket, BatteryPacket, GNSSPacket, ImuPacket, MagPacket, PitotPacket, RcPacket,
    },
};

pub struct LatestRing<T: Copy, const N: usize> {
    items: [Option<T>; N],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl<T: Copy, const N: usize> LatestRing<T, N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    pub fn push(&mut self, item: T) {
        if self.len == N {
            self.head = (self.head + 1) % N;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let tail = (self.head + self.len) % N;
        self.items[tail] = Some(item);
        self.len += 1;
    }

    pub fn take_latest(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }

        let latest = (self.head + self.len - 1) % N;
        let item = self.items[latest].take();
        self.head = 0;
        self.len = 0;
        item
    }

    pub fn has_pending(&self) -> bool {
        self.len != 0
    }

    pub fn dropped_oldest(&self) -> u32 {
        self.dropped_oldest
    }
}

#[derive(Clone, Copy)]
pub struct SharedSensorQueue<T: Copy + 'static, const N: usize> {
    inner: &'static Mutex<RefCell<LatestRing<T, N>>>,
}

unsafe impl<T: Copy + 'static, const N: usize> Send for SharedSensorQueue<T, N> {}
unsafe impl<T: Copy + 'static, const N: usize> Sync for SharedSensorQueue<T, N> {}

impl<T: Copy + 'static, const N: usize> SharedSensorQueue<T, N> {
    pub const fn new(inner: &'static Mutex<RefCell<LatestRing<T, N>>>) -> Self {
        Self { inner }
    }

    pub fn push(&self, item: T) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(item));
    }

    pub fn take_latest(&self) -> Option<T> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }

    pub fn has_pending(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).has_pending())
    }

    pub fn dropped_oldest(&self) -> u32 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).dropped_oldest())
    }
}

static IMU_QUEUE: Mutex<RefCell<LatestRing<ImuPacket<f32>, 8>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static BARO_QUEUE: Mutex<RefCell<LatestRing<Result<BaroPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static MAG_QUEUE: Mutex<RefCell<LatestRing<Result<MagPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static PITOT_QUEUE: Mutex<RefCell<LatestRing<Result<PitotPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static GNSS_QUEUE: Mutex<RefCell<LatestRing<Result<GNSSPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static BATTERY_QUEUE: Mutex<RefCell<LatestRing<Result<BatteryPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));
static RC_QUEUE: Mutex<RefCell<LatestRing<Result<RcPacket, SensorError>, 4>>> =
    Mutex::new(RefCell::new(LatestRing::new()));

pub const SHARED_IMU_QUEUE: SharedSensorQueue<ImuPacket<f32>, 8> =
    SharedSensorQueue::new(&IMU_QUEUE);
pub const SHARED_BARO_QUEUE: SharedSensorQueue<Result<BaroPacket, SensorError>, 4> =
    SharedSensorQueue::new(&BARO_QUEUE);
pub const SHARED_MAG_QUEUE: SharedSensorQueue<Result<MagPacket, SensorError>, 4> =
    SharedSensorQueue::new(&MAG_QUEUE);
pub const SHARED_PITOT_QUEUE: SharedSensorQueue<Result<PitotPacket, SensorError>, 4> =
    SharedSensorQueue::new(&PITOT_QUEUE);
pub const SHARED_GNSS_QUEUE: SharedSensorQueue<Result<GNSSPacket, SensorError>, 4> =
    SharedSensorQueue::new(&GNSS_QUEUE);
pub const SHARED_BATTERY_QUEUE: SharedSensorQueue<Result<BatteryPacket, SensorError>, 4> =
    SharedSensorQueue::new(&BATTERY_QUEUE);
pub const SHARED_RC_QUEUE: SharedSensorQueue<Result<RcPacket, SensorError>, 4> =
    SharedSensorQueue::new(&RC_QUEUE);
