#![no_std]
#![no_main]

mod fmt;

use core::f32::consts::PI;

use defmt::{info, println};
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
        Channel,
    },
};
use embassy_time::Instant;

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

    u_enable.set_high();
    v_enable.set_high();
    w_enable.set_high();

    let mut sin_table = [0.0; 1024];
    for i in 0..1024 {
        sin_table[i] = libm::sinf(i as f32 * (1024.0 / (2.0 * PI)));
    }

    let mut prev_time = Instant::now();
    let mut prev_hole_sensor_time = Instant::now();

    let mut prev_hole_a = hall_a.is_high();
    let mut prev_hole_b = hall_b.is_high();
    let mut prev_hole_c = hall_c.is_high();

    let mut hall_status;

    let mut theta: f32 = 0.0;

    let mut theta_speed: f32 = 0.0;

    loop {
        let now_time = Instant::now();
        let elapsed = now_time - prev_time;
        prev_time = now_time;

        if prev_hole_a != hall_a.is_high()
            || prev_hole_b != hall_b.is_high()
            || prev_hole_c != hall_c.is_high()
        {
            let elapsed_hole_sensor = now_time - prev_hole_sensor_time;
            prev_hole_sensor_time = now_time;

            // info!(
            //     "Hall A: {:?}, Hall B: {:?}, Hall C: {:?}",
            //     hall_a.is_high(),
            //     hall_b.is_high(),
            //     hall_c.is_high()
            // );

            prev_hole_a = hall_a.is_high();
            prev_hole_b = hall_b.is_high();
            prev_hole_c = hall_c.is_high();

            hall_status = match (hall_a.is_high(), hall_b.is_high(), hall_c.is_high()) {
                (true, false, true) => 0,
                (true, false, false) => 1,
                (true, true, false) => 2,
                (false, true, false) => 3,
                (false, true, true) => 4,
                (false, false, true) => 5,
                (true, true, true) => 7,    // bug
                (false, false, false) => 8, // bug
            };

            theta = hall_status as f32 * 60.0;

            let tmp_theta_speed = 60.0 * (elapsed_hole_sensor.as_micros() as f32 / 1000.0 / 1000.0);
            if tmp_theta_speed.is_normal() {
                theta_speed = (tmp_theta_speed + theta_speed) / 2.0;
            }

            // info!("hall status: {}", hall_status);
            // info!("theta: {}, theta_speed: {}", theta, theta_speed);
        } else {
            theta = theta + theta_speed * (elapsed.as_micros() as f32 / 1000.0 / 1000.0);
        }

        let d = 0.0;
        let q = 1.0;

        // D, Q -> a, b

        // a = D * cos(theta) - Q * sin(theta)
        // b = D * sin(theta) + Q * cos(theta)
        let a = d * libm::cosf(theta.to_radians()) - q * libm::sinf(theta.to_radians());
        let b = d * libm::sinf(theta.to_radians()) + q * libm::cosf(theta.to_radians());

        // a = 1/sqrt(6) * a
        // b = 1/sqrt(2) * b
        let a = 0.40824892046 * a;
        let b = 0.70710678118 * b;

        // u = 2.0 * a
        // v = -a + b
        // w = -a - b
        let u = 2.0 * a;
        let v = -a + b;
        let w = -a - b;

        // u, v, w -> duty
        let u_duty = (u * 0.5 + 0.5) * driver_pwm.get_max_duty() as f32;
        let v_duty = (v * 0.5 + 0.5) * driver_pwm.get_max_duty() as f32;
        let w_duty = (w * 0.5 + 0.5) * driver_pwm.get_max_duty() as f32;

        driver_pwm.set_duty(Channel::Ch1, u_duty as u16);
        driver_pwm.set_duty(Channel::Ch2, v_duty as u16);
        driver_pwm.set_duty(Channel::Ch3, w_duty as u16);

        led.toggle();

        // Timer::after_millis(5).await;
    }
}
