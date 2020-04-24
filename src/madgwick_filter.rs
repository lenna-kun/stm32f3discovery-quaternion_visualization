use core::f32::consts::PI;

use cast::{f32, i32};

use f3::{
    hal::{
        prelude::*,
        time::Hertz,
        timer::Timer,
        stm32f30x::TIM2,
    },
    l3gd20,
    lsm303dlhc,
    L3gd20,
};

use madgwick::{Quaternion, F32x3, Marg};

// Number of samples to use for gyroscope calibration
const NSAMPLES: i32 = 256;
 
// Magnetometer calibration parameters
// NOTE you need to use the right parameters for *your* magnetometer
// You can use the `log-sensors` example to calibrate your magnetometer. The producer is explained
// in https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
const M_BIAS_X: f32 = -34.;
const M_SCALE_X: f32 = 650.;
 
const M_BIAS_Y: f32 = -70.;
const M_SCALE_Y: f32 = 636.;
 
const M_BIAS_Z: f32 = -37.5;
const M_SCALE_Z: f32 = 589.5;
 
// Sensitivities of the accelerometer and gyroscope, respectively
const K_G: f32 = 2. / (1 << 15) as f32; // LSB -> g
const K_AR: f32 = 8.75e-3 * PI / 180.; // LSB -> rad/s
 
// Madgwick filter parameters
const SAMPLE_FREQ: u32 = 220;
const BETA: f32 = 1e-3;

pub fn calibrate_gyroscope(timer: &mut Timer<TIM2>, l3gd20: &mut L3gd20) -> (i16, i16, i16) {
    // Calibrate the gyroscope
    let mut ar_bias_x = 0;
    let mut ar_bias_y = 0;
    let mut ar_bias_z = 0;
    for _ in 0..NSAMPLES {
        block!(timer.wait()).unwrap();

        let ar = l3gd20.gyro().unwrap();

        ar_bias_x += i32(ar.x);
        ar_bias_y += i32(ar.y);
        ar_bias_z += i32(ar.z);
    }

    (
        (ar_bias_x / NSAMPLES) as i16,
        (ar_bias_y / NSAMPLES) as i16,
        (ar_bias_z / NSAMPLES) as i16
    )
}

pub struct MadgwickFilter {
    pub marg: Marg,
    pub ar_bias: (i16, i16, i16),
}

impl MadgwickFilter {
    pub fn new(pclk1_hz: Hertz, ar_bias: (i16, i16, i16)) -> Self {
        MadgwickFilter {
            marg: Marg::new(BETA, 1. / f32((SAMPLE_FREQ*pclk1_hz.0)/32)),
            ar_bias: ar_bias,
        }
    }
    pub fn run_filter(&mut self, m: lsm303dlhc::I16x3, ar: l3gd20::I16x3, g: lsm303dlhc::I16x3) -> Quaternion {
        let m_x = (f32(m.x) - M_BIAS_X) / M_SCALE_X;
        let m_y = (f32(m.y) - M_BIAS_Y) / M_SCALE_Y;
        let m_z = (f32(m.z) - M_BIAS_Z) / M_SCALE_Z;
 
        // Fix the X Y Z components of the magnetometer so they match the gyro axes
        let m = F32x3 {
            x: m_y,
            y: -m_x,
            z: m_z,
        };
 
        let ar_x = f32(ar.x - self.ar_bias.0) * K_AR;
        let ar_y = f32(ar.y - self.ar_bias.1) * K_AR;
        let ar_z = f32(ar.z - self.ar_bias.2) * K_AR;
        let ar = F32x3 {
            x: ar_x,
            y: ar_y,
            z: ar_z,
        };
 
        // Fix the X Y Z components of the accelerometer so they match the gyro axes
        let g_x = f32(g.x) * K_G;
        let g_y = f32(g.y) * K_G;
        let g_z = f32(g.z) * K_G;
        let g = F32x3 {
            x: g_y,
            y: -g_x,
            z: g_z,
        };
        self.marg.update(m, ar, g)
    }
}