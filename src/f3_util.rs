use super::madgwick_filter;

use f3::{
    hal::{
        prelude::*,
        i2c::I2c,
        spi::Spi,
        timer::Timer,
        stm32f30x::{self, ITM, TIM2},
    },
    led::Led,
    l3gd20::{self, Odr},
    lsm303dlhc::{AccelOdr, MagOdr},
    L3gd20,
    Lsm303dlhc,
};

pub struct SensorModules {
    pub l3gd20: L3gd20,
    pub lsm303dlhc: Lsm303dlhc,
}

pub fn init() -> (ITM, Timer<TIM2>, [Led; 8], SensorModules, madgwick_filter::MadgwickFilter) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();
    
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        l3gd20::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

    let mut nss = gpioe
        .pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    #[allow(deprecated)]
    nss.set_high();

    let mut l3gd20 = L3gd20::new(spi, nss).unwrap();
    l3gd20.set_odr(Odr::Hz380).unwrap();

    let mut lsm303dlhc = Lsm303dlhc::new(i2c).unwrap();

    lsm303dlhc.accel_odr(AccelOdr::Hz400).unwrap();
    lsm303dlhc.mag_odr(MagOdr::Hz220).unwrap();

    let mut timer = Timer::tim2(dp.TIM2, 380.hz(), clocks, &mut rcc.apb1);

    let gyro_bias = madgwick_filter::calibrate_gyroscope(&mut timer, &mut l3gd20);

    let madgwick_filter = madgwick_filter::MadgwickFilter::new(8.hz(), gyro_bias);
    let timer = Timer::tim2(timer.free(), (220*8/32).hz(), clocks, &mut rcc.apb1);
    let leds: [Led; 8] = [
        gpioe
            .pe8
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe9
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe10
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe11
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe12
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe13
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe14
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into(),
        gpioe
            .pe15
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
            .into()
    ];

    (
        cp.ITM,
        timer,
        leds,
        SensorModules {
            l3gd20: l3gd20,
            lsm303dlhc: lsm303dlhc,
        },
        madgwick_filter,
    )
}

pub fn adjust_cycle(timer: &mut Timer<TIM2>) {
    block!(timer.wait()).unwrap();
}