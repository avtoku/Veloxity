# Marmot (RP2354B) Implementation Plan

## Goals & Constraints
- Implement Veloxity firmware for the revised **Marmot (RP2354B)** board using **Option B**:
  - All new board, bus, sensor, QSPI parameter storage, PWM, and communication code is contained inside `boards/marmot/`.
  - Only root `Cargo.toml` (adding `"boards/marmot"` to `workspace.members`) and `xtask/src/main.rs` (adding `"marmot"` board target `thumbv8m.main-none-eabihf`) are modified outside `boards/marmot/`.
  - `veloxity_core`, `veloxity_mavlink`, and `platforms/rp2350` remain unmodified (`platforms/rp2350` enables `embassy-rp/rp235xa`, whereas `boards/marmot` enables `embassy-rp/rp235xb`).
- **MCU & Clocking**: RP2354B (48 GPIOs `PIN_0`..`PIN_47`, 2 MB internal stacked flash) with a **12 MHz CMOS external clock** (`embassy_rp::config::Config::default()`).
- **Floating-Point Precision**: Hardware single-precision `f32` throughout (`type MarmotReal = f32` for `World`), using `f64` only where required (`GNSSPacket::lat` and `lon`).
- **Temperature**: Temperature fields on IMU/Baro/Pitot/Mag are unused by `veloxity_core` flight pipelines and populated with `0.0` (saving bus time).
- **Parameter Storage**: Winbond `W25Q128JVPIM` 16 MB external NOR flash over `PIO0` QSPI acts as the nonvolatile parameter store (replacing the microSD card used on STM32), encoding/decoding ROSflight 2.0's `7,004`-byte `params_t` + `4`-byte IEEE CRC32 (`7,008` bytes total) in sectors `0x0000_0000` and `0x0000_1000`.

---

## Hardware Assignment Table (Revised RP2354B Board)

| Device / Subsystem | Signal | Package Pin | GPIO | Peripheral / Driver | DMA / IRQ / Notes |
| --- | --- | --- | --- | --- | --- |
| **IMU (`LSM6DSK320XTR` / `LSM6DSV320X`)** | `DRDY` | 39 | `PIN_31` | Input (rising edge) | `IO_IRQ_BANK0` |
| | `MISO` | 40 | `PIN_32` | `SPI0` RX | `DMA_CH0` (TX), `DMA_CH1` (RX) |
| | `SCK` | 43 | `PIN_34` | `SPI0` SCK | Mode 3, 4 MHz, 960 Hz HAODR |
| | `MOSI` | 44 | `PIN_35` | `SPI0` TX | |
| | `CS` | 46 | `PIN_37` | Output (`Level::High`) | Software CS |
| **Baro (`BMP581`)** | `DRDY` | 48 | `PIN_39` | Input (rising edge) | `IO_IRQ_BANK0` |
| | `SDA` | 49 | `PIN_40` | `I2C0` SDA | `I2C0_IRQ`, 400 kHz, addr `0x46` |
| | `SCL` | 52 | `PIN_41` | `I2C0` SCL | |
| **Mag (`MMC5983MA`)** | `SCL` | 53 | `PIN_42` | `PIO0_SM0` (`PioI2c`) | Shared 21-instr `PioI2cProgram` on `PIO0`, `PIO0_IRQ_0`, 400 kHz, addr `0x30` |
| | `SDA` | 54 | `PIN_43` | `PIO0_SM0` (`PioI2c`) | |
| | `DRDY` | 55 | `PIN_44` | Input | |
| **Pitot (`DLHR`)** | `SDA` | 45 | `PIN_36` | `PIO0_SM2` (`PioI2c`) | Shared `PioI2cProgram` on `PIO0`, `PIO0_IRQ_0`, 200 kHz, addr `0x29`, fs `1245.4445` Pa |
| | `SCL` | 47 | `PIN_38` | `PIO0_SM2` (`PioI2c`) | |
| | `DRDY` | 56 | `PIN_45` | Input | |
| **Flash (`W25Q128JVPIM`)** | `CS` | 19 | `PIN_19` | Output (`Level::High`) | Software CS |
| | `CLK` | 20 | `PIN_20` | `PIO0_SM1` (`PioQspi`) | 7-instr `PioQspiProgram` on `PIO0` (total 28/32 instrs) |
| | `D0`..`D3` | 21, 22, 23, 25 | `PIN_21`..`PIN_24` | `PIO0_SM1` (`PioQspi`) | `DMA_CH4` (TX), `DMA_CH5` (RX), 10 MHz |
| **GPS (`u-blox M10`)** | `TX` | 36 | `PIN_28` | `UART0` TX | `DMA_CH2` (TX), `DMA_CH3` (RX), `UART0_IRQ` |
| | `RX` | 37 | `PIN_29` | `UART0` RX | Auto-sync to 115200 baud, 10 Hz `UBX-NAV-PVT` |
| | `PPS` | 38 | `PIN_30` | Input (rising edge) | 1 Hz timepulse timestamp capture |
| **TELEM (MAVLink)** | `TX` | 27 | `PIN_26` | `UART1` TX | `DMA_CH6` (TX), `DMA_CH7` (RX), `UART1_IRQ` |
| | `RX` | 28 | `PIN_27` | `UART1` RX | 921600 baud (configurable) |
| **RC (`S.Bus`)** | `TX` | 16 | `PIN_16` | Reserved (`PIO1` TX TBD) | |
| | `RX` | 17 | `PIN_17` | `PIO1_SM0` (`PioSbusRx`) | Inverted `8E2` @ 100 kbaud (`PIO1` `GPIOBASE=0`) |
| **EXT I2C** | `SCL` | 14 | `PIN_15` | `I2C1` SCL | Initialized/reserved for external I2C |
| | `SDA` | 18 | `PIN_18` | `I2C1` SDA | |
| **Battery Monitor** | `V BAT` | 57 | `PIN_46` | `ADC` Ch6 | Polled 10 Hz via `embassy_rp::adc` |
| | `I BAT` | 58 | `PIN_47` | `ADC` Ch7 | |
| **ESC (`1`..`4`)** | `RX` | 77 | `PIN_0` | Reserved (TBD) | |
| | `ESC 1` | 78 | `PIN_1` | `PWM_SLICE0` B | Logical PWM Ch 0 |
| | `ESC 2` | 79 | `PIN_2` | `PWM_SLICE1` A | Logical PWM Ch 1 |
| | `ESC 3` | 80 | `PIN_3` | `PWM_SLICE1` B | Logical PWM Ch 2 |
| | `ESC 4` | 1 | `PIN_4` | `PWM_SLICE2` A | Logical PWM Ch 3 |
| **SERVO (`1`..`6`)** | `SERVO 1` | 7 | `PIN_9` | `PWM_SLICE4` B | Logical PWM Ch 4 |
| | `SERVO 2` | 8 | `PIN_10` | `PWM_SLICE5` A | Logical PWM Ch 5 |
| | `SERVO 3` | 9 | `PIN_11` | `PWM_SLICE5` B | Logical PWM Ch 6 |
| | `SERVO 4` | 11 | `PIN_12` | `PWM_SLICE6` A | Logical PWM Ch 7 |
| | `SERVO 5` | 12 | `PIN_13` | `PWM_SLICE6` B | Logical PWM Ch 8 |
| | `SERVO 6` | 13 | `PIN_14` | `PWM_SLICE7` A | Logical PWM Ch 9 |
| **LEDs** | `RED` | 2 | `PIN_5` | Output | `led1` / fault |
| | `GRN` | 3 | `PIN_6` | Output | `led0` / status |
| | `BLU` | 4 | `PIN_7` | Output | |
| **Test Points** | `TP 4` | 6 | `PIN_8` | Output | `BoardIo::set_test_pin_1` |
| | `TP 5` | 26 | `PIN_25` | Output | `BoardIo::set_test_pin_2` |
| | `TP 6` | 42 | `PIN_33` | Output | `BoardIo::set_test_pin_3` |

### Note on `PIO0` Upper GPIO Window (`GPIOBASE = 16`)
`PIO0` controls `Flash` (`PIN_20`..`PIN_24`), `Pitot` (`PIN_36`, `PIN_38`), and `Mag` (`PIN_42`, `PIN_43`). All of these pins lie in `PIN_16..=PIN_47`, so `PIO0` must use `GPIOBASE = 16` (`embassy-rp` RP2350B support) while `PIO1` (`PIN_17`) uses `GPIOBASE = 0`.
- **Workarounds for `embassy-rp 0.10.0` PIO `set_config`**:
  1. `StateMachine::set_config` subtracts `shift` (`16` when `GPIOBASE = 16`) from `pinctrl` bases when any pin is `>= 32`, but writes `config.exec.jmp_pin` unshifted to the 5-bit `EXECCTRL.JMP_PIN` register field; `PioI2c::new` therefore subtracts `16` from `cfg.get_exec().jmp_pin` whenever `sda` or `scl` is `>= 32`.
  2. `StateMachine::set_config` writes `PIO::PIO.gpiobase` based solely on that SM's pins, so `Flash` (`PIN_20..=24`, all `< 32`) would reset `PIO0.gpiobase = false` (`shift = 0`) and be broken when `Mag`/`Pitot` set `PIO0.gpiobase = true` (`shift = 16`); `PioQspi::new` therefore forces `PIO0.gpiobase = true` and shifts `Flash`'s `pinctrl` `in_base`/`out_base`/`sideset_base` by `- 16` before `sm.set_pins` / `sm.set_pin_dirs`.

---

## Architecture Summary
- **Core 0 (Flight Loop)**:
  - Initializes hardware (`embassy_rp::init(Config::default())`), spawns Core 1 services, sets up `Board` and `MarmotPwmDriver`.
  - Instantiates `MarmotWorld = World<Board, quadrotor::Estimator<f32>, quadrotor::Controller<f32>, quadrotor::Mixer<f32>, MavlinkInterface, MarmotPwmDriver, f32>` running at `400 Hz` (via `ControlLoopRates::fixed_rate_hz(400)`).
  - Runs the `world.realtime_scheduler_step()` loop:
    - `ImuControl`: drains `SHARED_IMU_QUEUE` (`ImuPacket<f32>`), runs IMU ingestion, calibration, accumulation, and control if due.
    - `ControlUpdate`: runs fixed-rate estimator/controller/mixer/PWM pipeline.
    - `Service`: drains service sensor queues (baro, mag, pitot, gnss, battery, rc), processes MAVLink RX/TX via `SharedMavlinkMailbox`, and runs budgeted telemetry.
- **Core 1 (Async Embassy Executors)**:
  - `CORE1_IMU_EXECUTOR` (`InterruptExecutor` on `SIO_IRQ_BELL` at `Priority::P1`): runs `lsm6dsk320xtr::task` pushing `ImuPacket<f32>` into `SHARED_IMU_QUEUE`.
  - `CORE1_EXECUTOR` (thread-mode `Executor`): runs `uart_tx_task`, `uart_rx_task`, `qspi_storage::task`, `bmp581::task`, `mmc5983ma::task`, `dlhr::task`, `ubx::pps_task`, `ubx::task`, `sbus::task`, and `battery::task`.

---

## File Checklist
- [x] `boards/marmot/PLAN.md`
- [x] `Cargo.toml` (add `"boards/marmot"` to `members`)
- [x] `xtask/src/main.rs` (add `"marmot"` target)
- [x] `boards/marmot/Cargo.toml`
- [x] `boards/marmot/build.rs`
- [x] `boards/marmot/memory.x`
- [x] `boards/marmot/src/lib.rs`
- [x] `boards/marmot/src/pio_i2c.rs`
- [x] `boards/marmot/src/pio_qspi.rs`
- [x] `boards/marmot/src/qspi_storage.rs`
- [x] `boards/marmot/src/comms.rs`
- [x] `boards/marmot/src/pwm.rs`
- [x] `boards/marmot/src/sensors/mod.rs`
- [x] `boards/marmot/src/sensors/lsm6dsk320xtr.rs`
- [x] `boards/marmot/src/sensors/bmp581.rs`
- [x] `boards/marmot/src/sensors/mmc5983ma.rs`
- [x] `boards/marmot/src/sensors/dlhr.rs`
- [x] `boards/marmot/src/sensors/sbus.rs`
- [x] `boards/marmot/src/sensors/ubx.rs`
- [x] `boards/marmot/src/sensors/battery.rs`
- [x] `boards/marmot/src/board.rs`
- [x] `boards/marmot/src/bin/veloxity.rs`

---

## Verification Commands
```bash
cargo xtask check-board marmot
cargo xtask check-host
cargo xtask check-board pico2w
```
