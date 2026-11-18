use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pac;
use embassy_rp::peripherals::{
    PIN_1, PIN_2, PIN_3, PIN_4, PIN_9, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PWM_SLICE0,
    PWM_SLICE1, PWM_SLICE2, PWM_SLICE4, PWM_SLICE5, PWM_SLICE6, PWM_SLICE7,
};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, PwmOutput, SetDutyCycle};
use veloxity_core::board::BoardIo;
use veloxity_core::pwm::{
    PwmDriver, PwmError, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate,
};

pub const NUM_PWM_CHANNELS: usize = 10;

// Hardware PWM slice index for each logical channel (ESC 1..4, SERVO 1..6).
const CHANNEL_SLICES: [usize; NUM_PWM_CHANNELS] = [0, 1, 1, 2, 4, 5, 5, 6, 6, 7];

pub struct MarmotPwmPins {
    pub slice0: Peri<'static, PWM_SLICE0>,
    pub slice1: Peri<'static, PWM_SLICE1>,
    pub slice2: Peri<'static, PWM_SLICE2>,
    pub slice4: Peri<'static, PWM_SLICE4>,
    pub slice5: Peri<'static, PWM_SLICE5>,
    pub slice6: Peri<'static, PWM_SLICE6>,
    pub slice7: Peri<'static, PWM_SLICE7>,
    pub esc1: Peri<'static, PIN_1>,
    pub esc2: Peri<'static, PIN_2>,
    pub esc3: Peri<'static, PIN_3>,
    pub esc4: Peri<'static, PIN_4>,
    pub servo1: Peri<'static, PIN_9>,
    pub servo2: Peri<'static, PIN_10>,
    pub servo3: Peri<'static, PIN_11>,
    pub servo4: Peri<'static, PIN_12>,
    pub servo5: Peri<'static, PIN_13>,
    pub servo6: Peri<'static, PIN_14>,
}

pub struct MarmotPwmDriver {
    channels: [PwmOutput<'static>; NUM_PWM_CHANNELS],
    enabled_mask: u16,
    rates_hz: [f32; NUM_PWM_CHANNELS],
    tops: [u16; NUM_PWM_CHANNELS],
}

impl MarmotPwmDriver {
    pub fn new(pins: MarmotPwmPins) -> Self {
        let clock_freq_hz = clk_sys_freq();
        let default_rate_hz = 50.0_f32;
        let default_top = rate_to_top(default_rate_hz);
        let divider_x16 = ((clock_freq_hz as u64 * 16) / 1_000_000) as u16;

        let mut cfg = PwmConfig::default();
        cfg.top = default_top;
        cfg.divider = fixed::FixedU16::<fixed::types::extra::U4>::from_bits(divider_x16);
        cfg.compare_a = 0;
        cfg.compare_b = 0;

        let (_, esc1) = Pwm::new_output_b(pins.slice0, pins.esc1, cfg.clone()).split();
        let (esc2, esc3) =
            Pwm::new_output_ab(pins.slice1, pins.esc2, pins.esc3, cfg.clone()).split();
        let (esc4, _) = Pwm::new_output_a(pins.slice2, pins.esc4, cfg.clone()).split();
        let (_, servo1) = Pwm::new_output_b(pins.slice4, pins.servo1, cfg.clone()).split();
        let (servo2, servo3) =
            Pwm::new_output_ab(pins.slice5, pins.servo2, pins.servo3, cfg.clone()).split();
        let (servo4, servo5) =
            Pwm::new_output_ab(pins.slice6, pins.servo4, pins.servo5, cfg.clone()).split();
        let (servo6, _) = Pwm::new_output_a(pins.slice7, pins.servo6, cfg).split();

        let mut driver = Self {
            channels: [
                esc1.unwrap(),
                esc2.unwrap(),
                esc3.unwrap(),
                esc4.unwrap(),
                servo1.unwrap(),
                servo2.unwrap(),
                servo3.unwrap(),
                servo4.unwrap(),
                servo5.unwrap(),
                servo6.unwrap(),
            ],
            enabled_mask: 0,
            rates_hz: [default_rate_hz; NUM_PWM_CHANNELS],
            tops: [default_top; NUM_PWM_CHANNELS],
        };
        driver.disable_all();
        driver
    }

    fn write_pulse_us(&mut self, channel: usize, pulse_us: u16) -> Result<(), PwmError> {
        let ch = self
            .channels
            .get_mut(channel)
            .ok_or(PwmError::ChannelOutOfRange)?;
        let clamped = pulse_us.min(self.tops[channel]);
        ch.set_duty_cycle(clamped)
            .map_err(|_| PwmError::GenericError)
    }
}

fn rate_to_top(rate_hz: f32) -> u16 {
    let period_us = (1_000_000.0 / rate_hz) as u32;
    period_us.saturating_sub(1).clamp(2_000, u16::MAX as u32) as u16
}

impl PwmDriver<f32> for MarmotPwmDriver {
    fn len(&self) -> usize {
        NUM_PWM_CHANNELS
    }

    fn is_enabled(&self) -> bool {
        self.enabled_mask == ((1 << NUM_PWM_CHANNELS) - 1)
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.enabled_mask |= 1 << channel;
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.enabled_mask &= !(1 << channel);
        self.write_pulse_us(channel, 0)
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        for channel in 0..NUM_PWM_CHANNELS {
            self.enable(channel)?;
        }
        Ok(())
    }

    fn disable_all(&mut self) {
        for channel in 0..NUM_PWM_CHANNELS {
            let _ = self.disable(channel);
        }
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        if (self.enabled_mask & (1 << channel)) == 0 {
            return self.write_pulse_us(channel, 0);
        }

        let normalized = (duty as f32 / u16::MAX as f32).clamp(0.0, 1.0);
        let pulse_us = (1000.0 + normalized * 1000.0) as u16;
        self.write_pulse_us(channel, pulse_us)
    }

    fn flush<B: BoardIo>(&mut self, _board: &mut B) {}

    fn configure_output_rates(&mut self, rates_hz: &[f32]) -> Result<(), PwmError> {
        for (channel, rate) in rates_hz.iter().copied().enumerate().take(NUM_PWM_CHANNELS) {
            match output_protocol_for_rate(rate)? {
                PwmOutputProtocol::StandardPwm => {}
                PwmOutputProtocol::Dshot => return Err(PwmError::UnsupportedProtocol),
            }
            let effective = effective_output_rate_hz(rate)?;
            let top = rate_to_top(effective);
            self.rates_hz[channel] = effective;
            self.tops[channel] = top;

            let slice = CHANNEL_SLICES[channel];
            pac::PWM.ch(slice).top().write_value(pac::pwm::regs::ChTop(top as u32));
        }
        Ok(())
    }

    fn output_protocol(&self, channel: usize) -> Result<PwmOutputProtocol, PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        Ok(PwmOutputProtocol::StandardPwm)
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        commands: &[f32],
    ) -> Result<(), PwmError> {
        for (channel, command) in commands.iter().copied().enumerate().take(NUM_PWM_CHANNELS) {
            if (self.enabled_mask & (1 << channel)) == 0 {
                self.write_pulse_us(channel, 0)?;
            } else {
                let pulse_us = (1000.0 + command.clamp(0.0, 1.0) * 1000.0) as u16;
                self.write_pulse_us(channel, pulse_us)?;
            }
        }
        self.flush(board);
        Ok(())
    }
}
