#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    time::Hertz,
    timer::{
        simple_pwm::{PwmPin, SimplePwm},
        Channel, CountingMode,
    },
};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let u_pwm = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let v_pwm = PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let w_pwm = PwmPin::new_ch3(p.PA10, OutputType::PushPull);

    let mut driver_pwm = SimplePwm::new(
        p.TIM1,
        Some(u_pwm),
        Some(v_pwm),
        Some(w_pwm),
        None,
        Hertz::khz(18),
        Default::default(),
    );

    driver_pwm.enable(Channel::Ch1);
    driver_pwm.enable(Channel::Ch2);
    driver_pwm.enable(Channel::Ch3);

    let mut u_enable = Output::new(p.PC10, Level::Low, Speed::Low);
    let mut v_enable = Output::new(p.PC11, Level::Low, Speed::Low);
    let mut w_enable = Output::new(p.PC12, Level::Low, Speed::Low);

    let hall_a = Input::new(p.PA15, Pull::None);
    let hall_b = Input::new(p.PB3, Pull::None);
    let hall_c = Input::new(p.PB10, Pull::None);

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let mut counter = 0;

    u_enable.set_high();
    v_enable.set_high();
    w_enable.set_high();

    let mut sin_table = [0.0; 360];
    for i in 0..360 {
        sin_table[i] = libm::sinf(i as f32);
    }

    loop {
        let angle_u = counter % 360;
        let angle_v = (counter + 120) % 360;
        let angle_w = (counter + 240) % 360;

        let duty_u = driver_pwm.get_max_duty() as f32 / 2.0 * (1.0 + sin_table[angle_u]);
        let duty_v = driver_pwm.get_max_duty() as f32 / 2.0 * (1.0 + sin_table[angle_v]);
        let duty_w = driver_pwm.get_max_duty() as f32 / 2.0 * (1.0 + sin_table[angle_w]);

        driver_pwm.set_duty(Channel::Ch1, duty_u as u16);
        driver_pwm.set_duty(Channel::Ch2, duty_v as u16);
        driver_pwm.set_duty(Channel::Ch3, duty_w as u16);

        led.toggle();

        counter += 1;

        Timer::after_millis(4).await;
    }
}
