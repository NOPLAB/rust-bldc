#![no_std]
#![no_main]

mod fmt;

use core::f32::consts::PI;

use defmt::println;
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

    const ONCE_PER_REV: f32 = (2.0 * PI) / 3.0 / 2.0;

    let mut hall_status = 0;

    let mut theta = 0.0;

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

            // info!("hall status: {}", hall_status);
        }

        match hall_status {
            0 => {
                driver_pwm.set_duty(Channel::Ch1, driver_pwm.get_max_duty() as u16);
                driver_pwm.set_duty(Channel::Ch2, 0);
                driver_pwm.set_duty(Channel::Ch3, 0);

                u_enable.set_high();
                v_enable.set_high();
                w_enable.set_low();
            }
            1 => {
                driver_pwm.set_duty(Channel::Ch1, 0);
                driver_pwm.set_duty(Channel::Ch2, driver_pwm.get_max_duty() as u16);
                driver_pwm.set_duty(Channel::Ch3, 0);

                u_enable.set_high();
                v_enable.set_low();
                w_enable.set_high();
            }
            2 => {
                driver_pwm.set_duty(Channel::Ch1, 0);
                driver_pwm.set_duty(Channel::Ch2, driver_pwm.get_max_duty() as u16);
                driver_pwm.set_duty(Channel::Ch3, 0);

                u_enable.set_low();
                v_enable.set_high();
                w_enable.set_high();
            }
            3 => {
                driver_pwm.set_duty(Channel::Ch1, 0);
                driver_pwm.set_duty(Channel::Ch2, driver_pwm.get_max_duty() as u16);
                driver_pwm.set_duty(Channel::Ch3, 0);

                u_enable.set_high();
                v_enable.set_high();
                w_enable.set_low();
            }
            4 => {
                driver_pwm.set_duty(Channel::Ch1, 0);
                driver_pwm.set_duty(Channel::Ch2, 0);
                driver_pwm.set_duty(Channel::Ch3, driver_pwm.get_max_duty() as u16);

                u_enable.set_high();
                v_enable.set_low();
                w_enable.set_high();
            }
            5 => {
                driver_pwm.set_duty(Channel::Ch1, 0);
                driver_pwm.set_duty(Channel::Ch2, 0);
                driver_pwm.set_duty(Channel::Ch3, driver_pwm.get_max_duty() as u16);

                u_enable.set_low();
                v_enable.set_high();
                w_enable.set_high();
            }
            _ => {}
        }

        led.toggle();

        // Timer::after_millis(5).await;
    }
}
