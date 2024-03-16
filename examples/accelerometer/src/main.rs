#![no_std]
#![no_main]

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_time::Duration;
use microbit_bsp::{
    display::{Brightness, Frame},
    embassy_nrf::{bind_interrupts, peripherals::TWISPI0, twim::InterruptHandler},
    motion::Sensor,
    LedMatrix, Microbit,
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
    let mut sensor = Sensor::new(board.twispi0, irqs, board.p23, board.p22).unwrap();

    let status = sensor.accel_status().unwrap();
    info!("status: {:?}", Debug2Format(&status));

    loop {
        let (x, y, z) = sensor.accel_data().unwrap().xyz_mg();
        #[allow(clippy::cast_precision_loss)]
        display_level(&mut display, Duration::from_millis(100), x as f32, y as f32, z as f32).await;
    }
}

async fn display_level(display: &mut LedMatrix, length: Duration, acc_x: f32, acc_y: f32, acc_z: f32) {
    let z = {
        // To avoid division by 0
        const LIMIT: f32 = 100.;
        if acc_z <= -LIMIT || acc_z >= LIMIT {
            acc_z
        } else if acc_z > 0. {
            LIMIT
        } else {
            -LIMIT
        }
    };
    let sin_x = acc_x / z;
    let sin_y = -acc_y / z;

    let offset_x = sin_to_offset(sin_x);
    let offset_y = sin_to_offset(sin_y);

    let frame = create_frame(offset_x, offset_y);

    display.display(frame, length).await;
}

fn create_frame(offset_x: i8, offset_y: i8) -> Frame<5, 5> {
    let mut frame = Frame::empty();
    frame.set(
        usize::try_from(2 + offset_x).unwrap(),
        usize::try_from(2 + offset_y).unwrap(),
    );
    frame
}

fn sin_to_offset(a: f32) -> i8 {
    const SIN_BAD: f32 = 0.175;
    const SIN_OK: f32 = 0.0874;
    const OFFSET_BAD: i8 = 2;
    const OFFSET_GOOD: i8 = 1;
    const OFFSET_PERFECT: i8 = 0;
    if a <= -SIN_BAD {
        return -OFFSET_BAD;
    }
    if a <= -SIN_OK {
        return -OFFSET_GOOD;
    }
    if a <= SIN_OK {
        return OFFSET_PERFECT;
    }
    if a <= SIN_BAD {
        return OFFSET_GOOD;
    }
    OFFSET_BAD
}
