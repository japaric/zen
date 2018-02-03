//! # References
//!
//! - [Kalman filter][0]
//!
//! [0]: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

#![feature(proc_macro)]
#![no_std]

extern crate byteorder;
extern crate cast;
extern crate cobs;
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate crc16;
extern crate either;
extern crate m;
extern crate motor_driver;
extern crate mpu9250;
extern crate stm32f103xx_hal as hal;

use core::f32::consts::PI;

use byteorder::{ByteOrder, LE};
use cortex_m::peripheral::DWT;
use crc16::{State, ARC};
use either::Either;
use hal::delay::Delay;
use hal::dma::{self, Transfer, dma1, R, W};
use hal::gpio::gpioa::{PA0, PA1, PA4, PA5, PA6, PA7};
use hal::gpio::gpiob::{PB12, PB13, PB14, PB15, PB6, PB7};
use hal::gpio::{Alternate, Floating, Input, Output, PushPull};
use hal::prelude::*;
use hal::pwm::{C3, C4, Pwm};
use hal::qei::Qei;
use hal::serial::{Rx, Serial, Tx};
use hal::spi::Spi;
use hal::stm32f103xx;
use hal::timer::{self, Timer};
use m::Float;
use motor_driver::Motor;
use motor_driver::ic::TB6612FNG;
use mpu9250::{Imu, Mpu9250};
use rtfm::{app, Resource, Threshold};
use stm32f103xx::{SPI1, TIM2, TIM3, TIM4, USART1};

use kalman::Kalman;

mod kalman;

// CONNECTIONS
type MPU9250 = Mpu9250<
    Spi<
        SPI1,
        (
            PA5<Alternate<PushPull>>,
            PA6<Input<Floating>>,
            PA7<Alternate<PushPull>>,
        ),
    >,
    PA4<Output<PushPull>>,
    Imu,
>;
type M1 = Motor<PB12<Output<PushPull>>, PB13<Output<PushPull>>, Pwm<TIM3, C3>, TB6612FNG>;
type M2 = Motor<PB14<Output<PushPull>>, PB15<Output<PushPull>>, Pwm<TIM3, C4>, TB6612FNG>;
type QE1 = Qei<TIM2, (PA0<Input<Floating>>, PA1<Input<Floating>>)>;
type QE2 = Qei<TIM4, (PB6<Input<Floating>>, PB7<Input<Floating>>)>;
type TX = Tx<USART1>;
type RX = Rx<USART1>;
#[allow(non_camel_case_types)]
type TX_BUF = &'static mut [u8; TX_SZ];
#[allow(non_camel_case_types)]
type RX_BUF = &'static mut [u8; RX_SZ];

// PARAMETERS
const DT: f32 = 1. / FREQ as f32;
const FREQ: u32 = 512;
const CLAMP: f32 = 1.;

// don't log too fast
// const LOG_DIV: u32 = 4;

// gyroscope sensitivity
const K_G: f32 = 250. / (1 << 15) as f32;
const K_A: f32 = 2. / (1 << 15) as f32;

const TX_SZ: usize = 18;
const RX_SZ: usize = 20;

app! {
    device: stm32f103xx,

    resources: {
        static KALMAN: Kalman;
        static M1: M1;
        static M2: M2;
        static MPU9250: MPU9250;
        static QE1: QE1;
        static QE2: QE2;

        static SET_POINT: f32 = 0.;
        static K_P: f32 = 2e-2;
        static K_I: f32 = 2e-1;
        static K_D: f32 = 0.;

        // total number of samples
        static NSAMPLES: u32 = 0;
        // sample count
        static SAMPLE: u32 = 0;
        // is PID on?
        static ON: bool = false;
        static COUNT: u32 = 0;
        // previous angle
        static PREVIOUS: Option<f32> = None;
        // integral of the error
        static IE: f32 = 0.;
        // last position of the encoders
        static POS: Option<(u16, u16)> = None;
        // cycles spent sleeping
        static SLEEP: u32 = 0;
        static TX: Option<Either<(TX_BUF, dma1::C4, TX), Transfer<R, TX_BUF, dma1::C4, TX>>> = None;
        static RX_BUF: [u8; RX_SZ] = [0; RX_SZ];
        static RX: Option<Either<(RX_BUF, dma1::C5, RX), Transfer<W, RX_BUF, dma1::C5, RX>>> = None;
        static TX_BUF: [u8; TX_SZ] = [0; TX_SZ];
    },

    init: {
        resources: [RX_BUF, TX_BUF],
    },

    idle: {
        resources: [SLEEP],
    },

    tasks: {
        SYS_TICK: {
            path: tick,
            resources: [
                COUNT, IE, KALMAN, K_P, K_I, K_D, M1, M2, MPU9250, NSAMPLES, ON, POS, PREVIOUS, QE1,
                QE2, SAMPLE, SLEEP, TX, SET_POINT
            ],
        },

        DMA1_CHANNEL5: {
            path: rx,
            resources: [K_P, K_I, K_D, NSAMPLES, ON, RX, SAMPLE, SET_POINT],
        }
    },
}

fn init(mut p: init::Peripherals, r: init::Resources) -> init::LateResources {
    p.core.DWT.enable_cycle_counter();

    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();

    let clocks = rcc.cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);

    let mut afio = p.device.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.device.GPIOC.split(&mut rcc.apb2);
    let mut channels = p.device.DMA1.split(&mut rcc.ahb);

    let mut delay = Delay::new(p.core.SYST, clocks);

    // LED
    let mut pc13 = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Blink led twice on reset (to visualize brown-out resets)
    pc13.set_low();
    for _ in 0..4 {
        delay.delay_ms(250_u8);
    }
    pc13.set_high();
    for _ in 0..4 {
        delay.delay_ms(250_u8);
    }
    pc13.set_low();
    for _ in 0..4 {
        delay.delay_ms(250_u8);
    }
    pc13.set_high();
    for _ in 0..4 {
        delay.delay_ms(250_u8);
    }

    // LED off while initializing

    // SERIAL
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;

    let serial = Serial::usart1(
        p.device.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let (mut tx, rx) = serial.split();

    // start of COBS frame
    tx.write(0x00).ok().unwrap();

    *r.TX = Some(Either::Left((r.TX_BUF, channels.4, tx)));

    channels.5.listen(dma::Event::TransferComplete);
    *r.RX = Some(Either::Right(rx.read_exact(channels.5, r.RX_BUF)));

    // SPI
    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let spi = Spi::spi1(
        p.device.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        mpu9250::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    // MPU9250
    let mut mpu9250 = Mpu9250::imu(spi, nss, &mut delay).ok().unwrap();

    // CALIBRATION & KALMAN FILTER INITIALIZATION
    let (mut gx, mut ary, mut arz) = (0, 0, 0);
    const NSAMPLES: i32 = 128;
    for _ in 0..NSAMPLES {
        let (ary_, arz_, _, gx_) = mpu9250.aryz_t_gx().ok().unwrap();
        ary += ary_ as i32;
        arz += arz_ as i32;
        gx += gx_ as i32;
        delay.delay_ms(1_u8);
    }

    // average
    gx /= NSAMPLES;
    ary /= NSAMPLES;
    arz /= NSAMPLES;

    let gyro_bias = gx as f32 * K_G;
    let angle = (ary as f32 * K_A).atan2(arz as f32 * K_A) * 180. / PI;

    let kalman = Kalman::new(angle, gyro_bias);

    // PWM
    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let (pwm1, pwm2) = p.device.TIM3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        (8 * FREQ).hz(),
        clocks,
        &mut rcc.apb1,
    );

    // TB6612FNG
    let pb12 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let pb13 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let pb14 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
    let pb15 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);

    let m1 = Motor::tb6612fng(pb12, pb13, pwm1);
    let m2 = Motor::tb6612fng(pb14, pb15, pwm2);

    // QEI
    let qe1 = Qei::tim2(
        p.device.TIM2,
        (gpioa.pa0, gpioa.pa1),
        &mut afio.mapr,
        &mut rcc.apb1,
    );

    let qe2 = Qei::tim4(
        p.device.TIM4,
        (gpiob.pb6, gpiob.pb7),
        &mut afio.mapr,
        &mut rcc.apb1,
    );

    // LED on = READY
    pc13.set_low();

    Timer::syst(delay.free(), FREQ.hz(), clocks).listen(timer::Event::Update);

    init::LateResources {
        MPU9250: mpu9250,
        KALMAN: kalman,
        M1: m1,
        M2: m2,
        QE1: qe1,
        QE2: qe2,
    }
}

fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        rtfm::atomic(t, |t| {
            let before = DWT::get_cycle_count();
            rtfm::wfi();
            let after = DWT::get_cycle_count();
            *r.SLEEP.borrow_mut(t) += after.wrapping_sub(before);
        });
    }
}

fn rx(_t: &mut Threshold, mut r: DMA1_CHANNEL5::Resources) {
    let (buf, c, rx) = match r.RX.take().unwrap() {
        Either::Left((buf, c, rx)) => (buf, c, rx),
        Either::Right(transfer) => transfer.wait(),
    };

    *r.NSAMPLES = LE::read_u32(&buf[0..4]);
    *r.SET_POINT = LE::read_f32(&buf[4..8]);
    *r.K_P = LE::read_f32(&buf[8..12]);
    *r.K_I = LE::read_f32(&buf[12..16]);
    *r.K_D = LE::read_f32(&buf[16..20]);

    *r.RX = Some(Either::Right(rx.read_exact(c, buf)));

    *r.ON = true;
    *r.SAMPLE = 0;
}

fn tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // KALMAN
    let (ary, arz, _, gx) = r.MPU9250.aryz_t_gx().ok().unwrap();
    let omega = (gx as f32) * K_G;

    let angle = (ary as f32 * K_A).atan2(arz as f32 * K_A) * 180. / PI;

    let estimate = r.KALMAN.update(angle, omega);

    // PID
    // TODO factor out in its own crate
    let pid = if *r.ON {
        let e = *r.SET_POINT - estimate;
        *r.IE += e * DT;

        let pid = r.PREVIOUS.map(|previous| {
            let dangle_dt = (angle - previous) / DT;

            let p = *r.K_P * e;
            let i = *r.K_I * *r.IE;
            let d = *r.K_D * dangle_dt;

            let pid = p + i - d;

            // clamp
            let pidc = if pid < -CLAMP {
                -CLAMP
            } else if pid > CLAMP {
                CLAMP
            } else {
                pid
            };

            let duty = (r.M1.get_max_duty() as f32 * pidc.abs()) as u16;

            r.M1.duty(duty);
            r.M2.duty(duty);

            // TODO remove
            if pid > 0. {
                // backwards
                r.M1.ccw();
                r.M2.ccw();
            } else {
                // forward
                r.M1.cw();
                r.M2.cw();
            }

            pid
        });

        *r.PREVIOUS = Some(e);

        pid
    } else {
        *r.PREVIOUS = None;
        *r.IE = 0.;

        None
    };

    *r.COUNT += 1;

    // LOG
    // if *r.ON && *r.COUNT % LOG_DIV == 0 {
    *r.SAMPLE += 1;

    // motor speed
    // let curr1 = r.QE1.count();
    // let curr2 = r.QE2.count();

    // let speed = r.POS.map(|(prev1, prev2)| {
    //     let speed1 = curr1.wrapping_sub(prev1) as i16;
    //     let speed2 = curr2.wrapping_sub(prev2) as i16;

    //     (speed1, speed2)
    // });

    // *r.POS = Some((curr1, curr2));

    let mut data = [0; TX_SZ - 2];

    // for CPU usage measurement
    // LE::write_u32(&mut data[0..4], *r.SLEEP);
    *r.SLEEP = 0;

    LE::write_i16(&mut data[0..2], ary);
    LE::write_i16(&mut data[0..4], arz);

    LE::write_i16(&mut data[4..6], gx);

    LE::write_f32(&mut data[6..10], estimate);

    LE::write_f32(&mut data[10..14], pid.unwrap_or(0.));

    // let (_speed1, _speed2) = speed.unwrap_or((0, 0));
    // LE::write_i16(&mut data[8..10], gx);
    // LE::write_u16(&mut data[8..10], curr1);
    // LE::write_u16(&mut data[10..12], curr2);

    let crc = State::<ARC>::calculate(&data[..TX_SZ - 4]);
    LE::write_u16(&mut data[TX_SZ - 4..], crc);

    cobs::encode(&data, buf);

    // terminate the last DMA transfer
    let (buf, c, tx) = match r.TX.take().unwrap() {
        Either::Left((buf, c, tx)) => (buf, c, tx),
        Either::Right(trans) => trans.wait(),
    };

    if *r.ON {
        *r.TX = Some(Either::Right(tx.write_all(c, buf)));
    } else {
        *r.TX = Some(Either::Left((buf, c, tx)));
    }
    // }

    if *r.SAMPLE >= *r.NSAMPLES {
        *r.ON = false;
        r.M1.brake();
        r.M2.brake();
    }
}
