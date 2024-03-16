#![no_std]
#![no_main]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
mod board;
pub use board::*;

pub mod display;
pub mod mic;
pub mod motion;
pub mod speaker;

// Re-exports

pub use embassy_nrf;
pub use lsm303agr;
pub use nb;
