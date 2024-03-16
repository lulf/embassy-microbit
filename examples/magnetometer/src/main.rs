#![no_std]
#![no_main]

use core::f32::consts::PI;
use defmt::{debug, info, Debug2Format};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use microbit_bsp::{
    display::{Brightness, Frame},
    embassy_nrf::{bind_interrupts, peripherals::TWISPI0, twim::InterruptHandler},
    lsm303agr,
    motion::new_lsm303agr,
    Microbit,
};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let board = Microbit::default();
    defmt::info!("Application started!");

    let mut display = board.display;
    display.set_brightness(Brightness::MAX);

    // Bind interrupt to the TWI/SPI peripheral.
    bind_interrupts!(
        struct InterruptRequests {
            SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => InterruptHandler<TWISPI0>;
        }
    );

    let irqs = InterruptRequests {};
    let mut sensor = new_lsm303agr(board.twispi0, irqs, board.p23, board.p22);
    sensor.init().unwrap();
    sensor.enable_mag_offset_cancellation().unwrap();
    sensor
        .set_mag_mode_and_odr(
            &mut embassy_time::Delay,
            lsm303agr::MagMode::HighResolution,
            lsm303agr::MagOutputDataRate::Hz50,
        )
        .unwrap();
    let Ok(mut sensor) = sensor.into_mag_continuous() else {
        panic!("Failed to set sensor to continuous mode");
    };

    let status = sensor.mag_status();
    info!("status: {:?}", Debug2Format(&status));

    Timer::after_secs(2).await;

    loop {
        let (x, y, z) = sensor.magnetic_field().unwrap().xyz_nt();
        debug!("x: {}, y: {}, z: {}", x, y, z);
        let (adjecent, opposite) = (y, x);
        #[allow(clippy::cast_precision_loss)]
        let frame = create_frame(opposite as f32, adjecent as f32);
        display.display(frame, Duration::from_millis(100)).await;
    }
}

fn create_frame(opposite: f32, adjecent: f32) -> Frame<5, 5> {
    // dir is in -pi..pi
    let dir = libm::atan2f(opposite, adjecent);
    // deg is in the range of 0..360
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let deg = (dir * 180.0 / PI + 180.0) as u16;
    debug!("dir: {}", deg);

    // map the current angle to the closest compass direction,
    // in the form of one of the LEDs on the display
    let (row, col) = if !(23..338).contains(&deg) {
        (0, 2)
    } else if deg < 68 {
        (1, 3)
    } else if deg < 113 {
        (2, 4)
    } else if deg < 158 {
        (3, 3)
    } else if deg < 203 {
        (4, 2)
    } else if deg < 248 {
        (3, 1)
    } else if deg < 293 {
        (2, 0)
    } else {
        (1, 1)
    };

    let mut frame = Frame::empty();
    frame.set(col, row);
    frame
}
