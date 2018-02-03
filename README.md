# `zen`

> A self-balancing robot coded in Rust

<p align="center">
  <img src="https://i.imgur.com/AOTNIem.jpg">
</p>

Currently the application is configured for manually tuning the PID controller. The robot expects a
20 byte serial frame that contains: (N = number of samples to run for: u32), (set point: f32), (K_p:
f32), (K_i: f32) and (K_d: f32). Upon receiving a frame the robot will run the PID control loop for
N iterations; while running the robot will log the following data: (acceleration in the Y axis: i16)
(acceleration in the Z axis: i16) (angular rate in the X axis: i16) (estimated tilt angle: f32) (PID
controller output = (non clamped) PWM duty cycle: f32). The data will be logged in 18 byte
(including the zero delimiter) COBS frames where the decoded frame contains a CRC16 (ARC) checksum
at the end.

## Part list

- [Self-balancing robot (kit with motors and
  chassis)](https://www.aliexpress.com/store/product/25GA370-Motor-with-encoder-Balanced-car-base-two-rounds-for-balance-toy-car-Two-wheel-self/1230304_32615777038.html)
- MPU9250
- HC-06
- Blue pill

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
