#![no_main]
#![no_std]

extern crate panic_semihosting; // panic handler
extern crate aligned;
extern crate byteorder;
extern crate cast;
extern crate cobs;
extern crate cortex_m;
extern crate f3;
extern crate madgwick;
#[macro_use(block)]
extern crate nb;
extern crate embedded_hal;

mod f3_util;
mod madgwick_filter;

use cortex_m::itm;
use cortex_m_rt::entry;
 
use aligned::Aligned;
use byteorder::{ByteOrder, LE};

#[entry]
fn main() -> ! {
    let (mut itm, mut timer, mut leds, mut sensor_modules, mut madgwick_filter) = f3_util::init();

    // Turn on the LEDs after initializing
    for i in 0..8 {
        leds[i].on();
    }
    
    let mut tx_buf: Aligned<aligned::A4, [u8; 18]> = Aligned([0; 18]);
    let mut led_index = 0;
    loop {
        // Make the cycle of the loop constant
        f3_util::adjust_cycle(&mut timer);

        // show reflesh rate with LED
        {
            leds[led_index].off();
            led_index = (led_index + 1) % 8;
            leds[led_index].on();
        }
 
        let m = sensor_modules.lsm303dlhc.mag().unwrap();
        let ar = sensor_modules.l3gd20.gyro().unwrap();
        let g = sensor_modules.lsm303dlhc.accel().unwrap();
 
        // Run the filter
        let quat = madgwick_filter.run_filter(m, ar, g);
 
        // Serialize the quaternion
        let mut start = 0;
        let mut buf = [0; 16];
        LE::write_f32(&mut buf[start..start + 4], quat.0);
        start += 4;
        LE::write_f32(&mut buf[start..start + 4], quat.1);
        start += 4;
        LE::write_f32(&mut buf[start..start + 4], quat.2);
        start += 4;
        LE::write_f32(&mut buf[start..start + 4], quat.3);
 
        // Log data
        cobs::encode(&buf, &mut *tx_buf);
 
        itm::write_aligned(&mut itm.stim[0], &tx_buf);
    }
}