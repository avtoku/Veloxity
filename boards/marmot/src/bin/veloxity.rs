#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embassy_rp::adc::{
    Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler,
};
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config as HalConfig;
use embassy_rp::executor::{Executor, InterruptExecutor};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config as I2cConfig, I2c};
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{
    ADC, CORE1, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5, DMA_CH6, DMA_CH7, DMA_CH8,
    DMA_CH9, I2C0, I2C1, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_23,
    PIN_24, PIN_26, PIN_27, PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_34, PIN_35, PIN_36, PIN_37,
    PIN_38, PIN_39, PIN_40, PIN_41, PIN_42, PIN_43, PIN_44, PIN_45, PIN_46, PIN_47, PIO0, PIO1,
    SPI0, UART0, UART1,
};
use embassy_rp::pio::{self, Pio};
use embassy_rp::spi::{self, Config as SpiConfig, Spi};
use embassy_rp::uart::{
    Async as UartAsync, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart,
    UartRx, UartTx,
};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::{Duration, Timer};
use marmot::board::{Board, BoardPins};
use marmot::comms::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
use marmot::pio_i2c::{PioI2c, PioI2cProgram};
use marmot::pio_qspi::{PioQspi, PioQspiProgram};
use marmot::pwm::{MarmotPwmDriver, MarmotPwmPins};
use marmot::qspi_storage;
use marmot::sensors::elrs::{PioElrsRx, PioElrsRxProgram, PioElrsTx, PioElrsTxProgram};
use marmot::sensors::{battery, bmp581, dlhr, elrs, lsm6dsk320xtr, mmc5983ma, ubx};
use panic_halt as _;
use static_cell::StaticCell;
use veloxity_core::board::{BoardIo, SerialRxPriority};
use veloxity_core::params::Params;
use veloxity_core::state_machine::StateManager;
use veloxity_core::vehicle::quadrotor;
use veloxity_core::world::{ControlLoopRates, RealtimeSchedulerStep, RealtimeServicePolicy, World};
use veloxity_mavlink::{MavlinkFrameEncoder, MavlinkInterface, parser::MavlinkParser};

type MarmotReal = f32;
const MARMOT_CONTROL_LOOP_HZ: u16 = 400;
const MARMOT_TELEM_STREAMS_PER_SERVICE_PHASE: usize = 2;

const TELEM_UART_BAUDRATE: u32 = 921_600;
const GPS_INITIAL_BAUDRATE: u32 = 9_600;
const IMU_SPI_HZ: u32 = 4_000_000;
const BARO_I2C_HZ: u32 = 400_000;
const MAG_I2C_HZ: u32 = 400_000;
const PITOT_I2C_HZ: u32 = 200_000;
const QSPI_HZ: u32 = 10_000_000;
const DLHR_FULL_SCALE_PA: f32 = 1245.4445;

const UART_TX_BATCH_BYTES: usize = 512;
const UART_RX_CHUNK_BYTES: usize = 16;
const UART_IDLE_DELAY_US: u64 = 50;

type MarmotWorld = World<
    Board,
    quadrotor::Estimator<MarmotReal>,
    quadrotor::Controller<MarmotReal>,
    quadrotor::Mixer<MarmotReal>,
    MavlinkInterface,
    MarmotPwmDriver,
    MarmotReal,
>;

static mut CORE1_STACK: Stack<32768> = Stack::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
static CORE1_IMU_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    UART0_IRQ => UartInterruptHandler<UART0>;
    UART1_IRQ => UartInterruptHandler<UART1>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>,
        dma::InterruptHandler<DMA_CH3>,
        dma::InterruptHandler<DMA_CH4>,
        dma::InterruptHandler<DMA_CH5>,
        dma::InterruptHandler<DMA_CH6>,
        dma::InterruptHandler<DMA_CH7>,
        dma::InterruptHandler<DMA_CH8>,
        dma::InterruptHandler<DMA_CH9>;
});

#[embassy_rp::interrupt]
unsafe fn SIO_IRQ_BELL() {
    unsafe { CORE1_IMU_EXECUTOR.on_interrupt() };
}

fn mavlink_uart_config() -> UartConfig {
    let mut cfg = UartConfig::default();
    cfg.baudrate = TELEM_UART_BAUDRATE;
    cfg
}

fn gps_uart_config() -> UartConfig {
    let mut cfg = UartConfig::default();
    cfg.baudrate = GPS_INITIAL_BAUDRATE;
    cfg
}

fn mavlink_rx_priority(message_id: u8) -> SerialRxPriority {
    match message_id {
        0 | 23 | 111 | 188 => SerialRxPriority::CRITICAL,
        20 | 21 | 180 | 193 | 195 => SerialRxPriority::DEFAULT,
        _ => SerialRxPriority::REPLACEABLE_TELEMETRY,
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart_tx: UartTx<'static, UartAsync>, mailbox: SharedMavlinkMailbox) -> ! {
    let mut tx = [0_u8; UART_TX_BATCH_BYTES];
    let mut encoder = MavlinkFrameEncoder::new();
    loop {
        let mut n = mailbox.drain_tx_batch_into(&mut tx);
        if n == 0 {
            if let Some(queued) = mailbox.pop_downlink_message() {
                n = encoder
                    .encode_downlink(queued.system_id, queued.msg, &mut tx)
                    .unwrap_or(0);
            }
        }
        if n == 0 {
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
            continue;
        }
        if uart_tx.write(&tx[..n]).await.is_ok() {
            mailbox.record_uart_tx_batch(n);
        } else {
            mailbox.record_uart_tx_error();
        }
    }
}

#[embassy_executor::task]
async fn uart_rx_task(mut uart_rx: UartRx<'static, UartAsync>, mailbox: SharedMavlinkMailbox) -> ! {
    let mut parser = MavlinkParser::new();
    let mut rx = [0_u8; UART_RX_CHUNK_BYTES];
    loop {
        if uart_rx.read(&mut rx).await.is_ok() {
            mailbox.record_uart_rx_chunk(rx.len());
            for byte in rx {
                if let Some(frame) = parser.feed_byte(byte) {
                    let priority = mavlink_rx_priority(frame.data[5]);
                    let _ = mailbox.push_rx_frame_priority(&frame.data[..frame.len], priority);
                }
            }
        } else {
            mailbox.record_uart_rx_error();
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
        }
    }
}

struct Core1Resources {
    core1: Peri<'static, CORE1>,
    spi0: Peri<'static, SPI0>,
    i2c0: Peri<'static, I2C0>,
    i2c1: Peri<'static, I2C1>,
    pio0: Peri<'static, PIO0>,
    pio1: Peri<'static, PIO1>,
    uart0: Peri<'static, UART0>,
    uart1: Peri<'static, UART1>,
    adc: Peri<'static, ADC>,
    dma_ch0: Peri<'static, DMA_CH0>,
    dma_ch1: Peri<'static, DMA_CH1>,
    dma_ch2: Peri<'static, DMA_CH2>,
    dma_ch3: Peri<'static, DMA_CH3>,
    dma_ch4: Peri<'static, DMA_CH4>,
    dma_ch5: Peri<'static, DMA_CH5>,
    dma_ch6: Peri<'static, DMA_CH6>,
    dma_ch7: Peri<'static, DMA_CH7>,
    dma_ch8: Peri<'static, DMA_CH8>,
    dma_ch9: Peri<'static, DMA_CH9>,
    // EXT I2C (I2C1)
    pin15: Peri<'static, PIN_15>,
    pin18: Peri<'static, PIN_18>,
    // RC ELRS/CRSF (TX/RX)
    pin16: Peri<'static, PIN_16>,
    pin17: Peri<'static, PIN_17>,
    // QSPI Flash (W25Q128JVPIM)
    pin19: Peri<'static, PIN_19>,
    pin20: Peri<'static, PIN_20>,
    pin21: Peri<'static, PIN_21>,
    pin22: Peri<'static, PIN_22>,
    pin23: Peri<'static, PIN_23>,
    pin24: Peri<'static, PIN_24>,
    // TELEM MAVLink (UART1)
    pin26: Peri<'static, PIN_26>,
    pin27: Peri<'static, PIN_27>,
    // GNSS (UART0 + PPS)
    pin28: Peri<'static, PIN_28>,
    pin29: Peri<'static, PIN_29>,
    pin30: Peri<'static, PIN_30>,
    // IMU (SPI0 + DRDY)
    pin31: Peri<'static, PIN_31>,
    pin32: Peri<'static, PIN_32>,
    pin34: Peri<'static, PIN_34>,
    pin35: Peri<'static, PIN_35>,
    pin37: Peri<'static, PIN_37>,
    // Pitot (PIO0_SM2 + DRDY)
    pin36: Peri<'static, PIN_36>,
    pin38: Peri<'static, PIN_38>,
    pin45: Peri<'static, PIN_45>,
    // Baro (I2C0 + DRDY)
    pin39: Peri<'static, PIN_39>,
    pin40: Peri<'static, PIN_40>,
    pin41: Peri<'static, PIN_41>,
    // Mag (PIO0_SM0 + DRDY)
    pin42: Peri<'static, PIN_42>,
    pin43: Peri<'static, PIN_43>,
    pin44: Peri<'static, PIN_44>,
    // Battery ADC
    pin46: Peri<'static, PIN_46>,
    pin47: Peri<'static, PIN_47>,
}

fn configure_core1_interrupt_priorities() {
    interrupt::IO_IRQ_BANK0.set_priority(Priority::P0);
    interrupt::TIMER0_IRQ_0.set_priority(Priority::P0);
    interrupt::DMA_IRQ_0.set_priority(Priority::P0);
    interrupt::I2C0_IRQ.set_priority(Priority::P2);
    interrupt::I2C1_IRQ.set_priority(Priority::P2);
    interrupt::PIO0_IRQ_0.set_priority(Priority::P2);
    interrupt::PIO1_IRQ_0.set_priority(Priority::P2);
    interrupt::UART0_IRQ.set_priority(Priority::P2);
    interrupt::UART1_IRQ.set_priority(Priority::P2);
    interrupt::ADC_IRQ_FIFO.set_priority(Priority::P2);
}

fn spawn_core1_services(resources: Core1Resources, mailbox: SharedMavlinkMailbox) {
    spawn_core1(
        resources.core1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            mailbox.set_comms_state(20);
            configure_core1_interrupt_priorities();

            // IMU (SPI0)
            let mut spi_cfg = SpiConfig::default();
            spi_cfg.frequency = IMU_SPI_HZ;
            spi_cfg.polarity = spi::Polarity::IdleHigh;
            spi_cfg.phase = spi::Phase::CaptureOnSecondTransition;
            let imu_spi = Spi::new(
                resources.spi0,
                resources.pin34, // CLK
                resources.pin35, // MOSI
                resources.pin32, // MISO
                resources.dma_ch0,
                resources.dma_ch1,
                Irqs,
                spi_cfg,
            );
            let imu_csn = Output::new(resources.pin37, Level::High);
            let imu_drdy = Input::new(resources.pin31, Pull::None);

            // PIO0: Mag (SM0), QSPI Flash (SM1), Pitot (SM2) - all pins in GPIO 19..=43 (GPIOBASE = 16)
            let Pio {
                common: mut pio0_common,
                sm0: pio0_sm0,
                sm1: pio0_sm1,
                sm2: pio0_sm2,
                irq0: pio0_irq0,
                irq2: pio0_irq2,
                irq_flags: pio0_irq_flags,
                ..
            } = Pio::new(resources.pio0, Irqs);
            let pio_i2c_prog = PioI2cProgram::new(&mut pio0_common);
            let pio_qspi_prog = PioQspiProgram::new(&mut pio0_common);

            let mag_i2c = PioI2c::new(
                &mut pio0_common,
                pio0_sm0,
                pio0_irq0,
                pio0_irq_flags.clone(),
                &pio_i2c_prog,
                resources.pin43, // SDA
                resources.pin42, // SCL
                MAG_I2C_HZ,
            );
            let mag_drdy = Input::new(resources.pin44, Pull::None);

            let flash = PioQspi::new(
                &mut pio0_common,
                pio0_sm1,
                &pio_qspi_prog,
                resources.pin19, // CS
                resources.pin20, // CLK
                resources.pin21, // D0
                resources.pin22, // D1
                resources.pin23, // D2
                resources.pin24, // D3
                resources.dma_ch4,
                resources.dma_ch5,
                Irqs,
                QSPI_HZ,
            );

            let pitot_i2c = PioI2c::new(
                &mut pio0_common,
                pio0_sm2,
                pio0_irq2,
                pio0_irq_flags,
                &pio_i2c_prog,
                resources.pin36, // SDA
                resources.pin38, // SCL
                PITOT_I2C_HZ,
            );
            let pitot_drdy = Input::new(resources.pin45, Pull::None);

            // PIO1: RC ELRS/CRSF RX (SM0 + DMA_CH8) and TX (SM1 + DMA_CH9)
            let Pio {
                common: mut pio1_common,
                sm0: pio1_sm0,
                sm1: pio1_sm1,
                ..
            } = Pio::new(resources.pio1, Irqs);
            let pio_rx_prog = PioElrsRxProgram::new(&mut pio1_common);
            let pio_tx_prog = PioElrsTxProgram::new(&mut pio1_common);
            let rc_uart_rx = PioElrsRx::new(
                elrs::CRSF_BAUD,
                &mut pio1_common,
                pio1_sm0,
                resources.pin17,
                resources.dma_ch8,
                Irqs,
                &pio_rx_prog,
            );
            let rc_uart_tx = PioElrsTx::new(
                elrs::CRSF_BAUD,
                &mut pio1_common,
                pio1_sm1,
                resources.pin16,
                resources.dma_ch9,
                Irqs,
                &pio_tx_prog,
            );

            // Baro (I2C0)
            let mut baro_i2c_cfg = I2cConfig::default();
            baro_i2c_cfg.frequency = BARO_I2C_HZ;
            let baro_i2c = I2c::new_async(
                resources.i2c0,
                resources.pin41, // SCL
                resources.pin40, // SDA
                Irqs,
                baro_i2c_cfg,
            );
            let baro_drdy = Input::new(resources.pin39, Pull::None);

            // External I2C (I2C1) initialized cleanly on its assigned pins
            let ext_i2c_cfg = I2cConfig::default();
            let _ext_i2c = I2c::new_async(
                resources.i2c1,
                resources.pin15, // SCL
                resources.pin18, // SDA
                Irqs,
                ext_i2c_cfg,
            );

            // GNSS (UART0 + PPS)
            let gps_uart = Uart::new(
                resources.uart0,
                resources.pin28, // TX
                resources.pin29, // RX
                Irqs,
                resources.dma_ch2,
                resources.dma_ch3,
                gps_uart_config(),
            );
            let gps_pps = Input::new(resources.pin30, Pull::None);

            // Telemetry MAVLink (UART1)
            let telem_uart = Uart::new(
                resources.uart1,
                resources.pin26, // TX
                resources.pin27, // RX
                Irqs,
                resources.dma_ch6,
                resources.dma_ch7,
                mavlink_uart_config(),
            );
            let (telem_tx, telem_rx) = telem_uart.split();

            // Battery Monitor (ADC6 / ADC7)
            let adc = Adc::new(resources.adc, Irqs, AdcConfig::default());
            let vbat_ch = AdcChannel::new_pin(resources.pin46, Pull::None);
            let ibat_ch = AdcChannel::new_pin(resources.pin47, Pull::None);

            // High-priority interrupt executor for IMU ingestion
            interrupt::SIO_IRQ_BELL.set_priority(Priority::P1);
            let imu_spawner = CORE1_IMU_EXECUTOR.start(interrupt::SIO_IRQ_BELL);
            imu_spawner.spawn(
                lsm6dsk320xtr::task(imu_spi, imu_csn, imu_drdy).expect("spawn IMU task"),
            );

            let executor = CORE1_EXECUTOR.init(Executor::new());
            mailbox.set_comms_state(21);
            executor.run(|spawner| {
                spawner.spawn(qspi_storage::task(flash).expect("spawn QSPI storage task"));
                spawner.spawn(uart_tx_task(telem_tx, mailbox).expect("spawn UART TX task"));
                spawner.spawn(uart_rx_task(telem_rx, mailbox).expect("spawn UART RX task"));
                spawner.spawn(bmp581::task(baro_i2c, baro_drdy).expect("spawn baro task"));
                spawner.spawn(mmc5983ma::task(mag_i2c, mag_drdy).expect("spawn mag task"));
                spawner.spawn(
                    dlhr::task(pitot_i2c, pitot_drdy, DLHR_FULL_SCALE_PA)
                        .expect("spawn pitot task"),
                );
                spawner.spawn(ubx::pps_task(gps_pps).expect("spawn PPS task"));
                spawner.spawn(ubx::task(gps_uart).expect("spawn GNSS task"));
                spawner.spawn(elrs::rx_task(rc_uart_rx).expect("spawn ELRS RX task"));
                spawner.spawn(elrs::tx_task(rc_uart_tx).expect("spawn ELRS TX task"));
                spawner.spawn(battery::task(adc, vbat_ch, ibat_ch).expect("spawn battery task"));
                mailbox.set_comms_state(22);
            })
        },
    );
}

fn hal_config() -> HalConfig {
    // 12 MHz CMOS external clock reference -> 150 MHz system clock
    HalConfig::new(ClockConfig::system_freq(150_000_000).unwrap())
}

fn init_world(board: Board, params: Params, pwm_driver: MarmotPwmDriver) -> MarmotWorld {
    let mixer = quadrotor::mixer(&params);
    let mut world = MarmotWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        quadrotor::Estimator::<MarmotReal>::default(),
        quadrotor::Controller::<MarmotReal>::default(),
        mixer,
        pwm_driver,
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(MARMOT_CONTROL_LOOP_HZ));
    world
}

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(hal_config());
    let mailbox = SHARED_MAVLINK_MAILBOX;

    let pwm_driver = MarmotPwmDriver::new(MarmotPwmPins {
        slice0: p.PWM_SLICE0,
        slice1: p.PWM_SLICE1,
        slice2: p.PWM_SLICE2,
        slice4: p.PWM_SLICE4,
        slice5: p.PWM_SLICE5,
        slice6: p.PWM_SLICE6,
        slice7: p.PWM_SLICE7,
        esc1: p.PIN_1,
        esc2: p.PIN_2,
        esc3: p.PIN_3,
        esc4: p.PIN_4,
        servo1: p.PIN_9,
        servo2: p.PIN_10,
        servo3: p.PIN_11,
        servo4: p.PIN_12,
        servo5: p.PIN_13,
        servo6: p.PIN_14,
    });

    let mut board = Board::new(BoardPins {
        led_red: Output::new(p.PIN_5, Level::High),
        led_green: Output::new(p.PIN_6, Level::High),
        led_blue: Output::new(p.PIN_7, Level::High),
        tp4: Output::new(p.PIN_8, Level::Low),
        tp5: Output::new(p.PIN_25, Level::Low),
        tp6: Output::new(p.PIN_33, Level::Low),
    });

    spawn_core1_services(
        Core1Resources {
            core1: p.CORE1,
            spi0: p.SPI0,
            i2c0: p.I2C0,
            i2c1: p.I2C1,
            pio0: p.PIO0,
            pio1: p.PIO1,
            uart0: p.UART0,
            uart1: p.UART1,
            adc: p.ADC,
            dma_ch0: p.DMA_CH0,
            dma_ch1: p.DMA_CH1,
            dma_ch2: p.DMA_CH2,
            dma_ch3: p.DMA_CH3,
            dma_ch4: p.DMA_CH4,
            dma_ch5: p.DMA_CH5,
            dma_ch6: p.DMA_CH6,
            dma_ch7: p.DMA_CH7,
            dma_ch8: p.DMA_CH8,
            dma_ch9: p.DMA_CH9,
            pin15: p.PIN_15,
            pin18: p.PIN_18,
            pin16: p.PIN_16,
            pin17: p.PIN_17,
            pin19: p.PIN_19,
            pin20: p.PIN_20,
            pin21: p.PIN_21,
            pin22: p.PIN_22,
            pin23: p.PIN_23,
            pin24: p.PIN_24,
            pin26: p.PIN_26,
            pin27: p.PIN_27,
            pin28: p.PIN_28,
            pin29: p.PIN_29,
            pin30: p.PIN_30,
            pin31: p.PIN_31,
            pin32: p.PIN_32,
            pin34: p.PIN_34,
            pin35: p.PIN_35,
            pin37: p.PIN_37,
            pin36: p.PIN_36,
            pin38: p.PIN_38,
            pin45: p.PIN_45,
            pin39: p.PIN_39,
            pin40: p.PIN_40,
            pin41: p.PIN_41,
            pin42: p.PIN_42,
            pin43: p.PIN_43,
            pin44: p.PIN_44,
            pin46: p.PIN_46,
            pin47: p.PIN_47,
        },
        mailbox,
    );

    let mut params = Params::default();
    if !board.read_params(&mut params) {
        veloxity_core::log_warn!("Unable to load parameters from QSPI flash; using defaults");
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    let mut world = init_world(board, params, pwm_driver);
    loop {
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                let _ = world.run_imu_control_tick();
            }
            RealtimeSchedulerStep::ControlUpdate => {
                let _ = world.run_control_update_tick();
            }
            RealtimeSchedulerStep::Service => {
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::continuous(MARMOT_TELEM_STREAMS_PER_SERVICE_PHASE),
                );
            }
            RealtimeSchedulerStep::Idle => {
                core::hint::spin_loop();
            }
        }
    }
}
