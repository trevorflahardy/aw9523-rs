//! Async implementation of the AW9523 driver.
//!
//! This module provides an async version of the AW9523 driver that uses
//! `embedded-hal-async` traits. Enable the `async` feature to use this module.
//!
//! All methods in this module are asynchronous and must be `.await`ed.
//!
//! # Example
//!
//! ```ignore
//! use aw9523::{r#async::Aw9523Async, PinMode, HIGH};
//! # let i2c = todo!(); // async I2C
//!
//! async fn configure_gpio() {
//!     let mut gpio = Aw9523Async::new(i2c, 0x58);
//!     gpio.init().await.unwrap();
//!     gpio.pin_mode(0, PinMode::Output).await.unwrap();
//!     gpio.digital_write(0, HIGH).await.unwrap();
//! }
//! ```

use embedded_hal::i2c::AddressMode;
use embedded_hal_async::i2c::I2c;

use crate::{
    Aw9523Error, PinMode, PinState, AW9523_REG_CONFIG0, AW9523_REG_CONFIG1, AW9523_REG_GCR,
    AW9523_REG_INPUT0, AW9523_REG_INPUT1, AW9523_REG_INTENABLE0, AW9523_REG_LEDMODE,
    AW9523_REG_OUTPUT0, AW9523_REG_SOFTRESET,
};

/// Async AW9523 driver instance
///
/// Manages asynchronous communication with an AW9523 GPIO expander over I2C.
/// All methods in this struct are async and must be `.await`ed.
///
/// # Type Parameters
///
/// * `A` - Address mode (typically `u8` for 7-bit I2C addresses)
/// * `I2C` - An async I2C implementation from `embedded-hal-async`
///
/// # Example
///
/// ```ignore
/// use aw9523::r#async::Aw9523Async;
/// # let i2c = todo!();
///
/// async fn setup() {
///     let mut gpio = Aw9523Async::new(i2c, 0x58);
///     gpio.init().await.unwrap();
/// }
/// ```
pub struct Aw9523Async<A: AddressMode, I2C: I2c<A>> {
    i2c: I2C,
    addr: A,
}

impl<A, I2C> Aw9523Async<A, I2C>
where
    A: AddressMode + Copy,
    I2C: I2c<A>,
{
    /// Creates a new async AW9523 driver instance.
    ///
    /// # Arguments
    ///
    /// * `i2c` - An async I2C bus implementation
    /// * `addr` - The I2C address of the device (typically 0x58)
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::r#async::Aw9523Async;
    /// # let i2c = todo!();
    /// let gpio = Aw9523Async::new(i2c, 0x58);
    /// ```
    pub fn new(i2c: I2C, addr: A) -> Self {
        Self { i2c, addr }
    }

    /// Consumes the driver and returns the I2C bus.
    ///
    /// Useful for testing and when you need to reclaim the I2C peripheral.
    #[cfg(test)]
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Initializes the AW9523 with a default configuration asynchronously.
    ///
    /// Sets up output values, pin directions, and enables push-pull mode.
    /// Call this after creating the device to ensure a known state.
    ///
    /// # Errors
    ///
    /// Returns an error if any I2C write operation fails.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::r#async::Aw9523Async;
    /// # let i2c = todo!();
    /// async fn setup() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     gpio.init().await.unwrap();
    /// }
    /// ```
    pub async fn init(&mut self) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_OUTPUT0, 0b00000101])
            .await?;
        self.write_register(&[AW9523_REG_OUTPUT0 + 1, 0b00000011])
            .await?;
        self.write_register(&[AW9523_REG_CONFIG0, 0b00011000])
            .await?;
        self.write_register(&[AW9523_REG_CONFIG0 + 1, 0b00001100])
            .await?;
        self.write_register(&[AW9523_REG_GCR, 0b00010000]).await?;
        self.write_register(&[AW9523_REG_LEDMODE + 1, 0b11111111])
            .await?;

        Ok(())
    }

    /// Writes data to a register on the device asynchronously.
    async fn write_register(&mut self, data: &[u8]) -> Result<(), Aw9523Error<I2C::Error>> {
        self.i2c
            .write(self.addr, data)
            .await
            .map_err(Aw9523Error::WriteError)
    }

    /// Performs a soft reset of the device asynchronously.
    ///
    /// All registers return to their default values after a reset.
    /// You should call [`init`](Self::init) again after reset.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C write fails.
    pub async fn reset(&mut self) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_SOFTRESET, 0x00]).await
    }

    /// Sets the output state for all 16 GPIO pins at once asynchronously.
    ///
    /// Each bit in the 16-bit value represents one pin's state (1 = high, 0 = low).
    /// Bit 0 corresponds to pin 0, bit 15 to pin 15.
    ///
    /// # Arguments
    ///
    /// * `pins` - 16-bit value where each bit sets the corresponding pin's output
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{r#async::Aw9523Async, AW9523_PIN_0, AW9523_PIN_3};
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     // Set pins 0 and 3 high, all others low
    ///     gpio.output_gpio(AW9523_PIN_0 | AW9523_PIN_3).await.unwrap();
    /// }
    /// ```
    pub async fn output_gpio(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_OUTPUT0, (pins & 0xFF) as u8])
            .await?;
        self.write_register(&[AW9523_REG_OUTPUT0 + 1, (pins >> 8) as u8])
            .await?;

        Ok(())
    }

    /// Reads the input state of all 16 GPIO pins at once asynchronously.
    ///
    /// Returns a 16-bit value where each bit represents one pin's state (1 = high, 0 = low).
    /// Bit 0 corresponds to pin 0, bit 15 to pin 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{r#async::Aw9523Async, AW9523_PIN_5};
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     let inputs = gpio.input_gpio().await.unwrap();
    ///     if inputs & AW9523_PIN_5 != 0 {
    ///         // Pin 5 is high
    ///     }
    /// }
    /// ```
    pub async fn input_gpio(&mut self) -> Result<u16, Aw9523Error<I2C::Error>> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[AW9523_REG_INPUT0], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;
        Ok(u16::from(buffer[0]) | (u16::from(buffer[1]) << 8))
    }

    /// Enables interrupt detection for all 16 GPIO pins at once asynchronously.
    ///
    /// Each bit in the 16-bit value enables (1) or disables (0) interrupt detection
    /// for the corresponding pin.
    ///
    /// # Arguments
    ///
    /// * `pins` - 16-bit value where each bit enables the corresponding pin's interrupt
    ///
    /// # Note
    ///
    /// The AW9523 uses inverted logic internally, but this is handled automatically.
    pub async fn interrupt_enable_gpio(
        &mut self,
        pins: u16,
    ) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_INTENABLE0, !(pins & 0xFF) as u8])
            .await?;
        self.write_register(&[AW9523_REG_INTENABLE0 + 1, !(pins >> 8) as u8])
            .await?;

        Ok(())
    }

    /// Configures pin direction for all 16 GPIO pins at once asynchronously.
    ///
    /// Each bit in the 16-bit value sets the direction for the corresponding pin:
    /// 1 = output, 0 = input.
    ///
    /// # Arguments
    ///
    /// * `pins` - 16-bit value where each bit sets the corresponding pin as output (1) or input (0)
    ///
    /// # Note
    ///
    /// The AW9523 uses inverted logic internally, but this is handled automatically.
    pub async fn configure_direction(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_CONFIG0, !(pins & 0xFF) as u8])
            .await?;
        self.write_register(&[AW9523_REG_CONFIG0 + 1, !(pins >> 8) as u8])
            .await?;

        Ok(())
    }

    /// Configures LED/constant-current mode for all 16 pins at once asynchronously.
    ///
    /// Each bit in the 16-bit value enables (1) or disables (0) LED mode
    /// for the corresponding pin. When enabled, the pin can be used with
    /// [`analog_write`](Self::analog_write) for LED dimming.
    ///
    /// # Arguments
    ///
    /// * `pins` - 16-bit value where each bit enables LED mode for the corresponding pin
    ///
    /// # Note
    ///
    /// The AW9523 uses inverted logic internally, but this is handled automatically.
    pub async fn configure_led_mode(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_LEDMODE, !(pins & 0xFF) as u8])
            .await?;
        self.write_register(&[AW9523_REG_LEDMODE + 1, !(pins >> 8) as u8])
            .await?;

        Ok(())
    }

    /// Sets the LED brightness/current level for a single pin asynchronously.
    ///
    /// The pin must be configured in LED mode first using [`pin_mode`](Self::pin_mode)
    /// or [`configure_led_mode`](Self::configure_led_mode).
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `val` - Brightness level (0 = off, 255 = maximum current)
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{r#async::Aw9523Async, PinMode};
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     gpio.pin_mode(0, PinMode::LedMode).await.unwrap();
    ///     gpio.analog_write(0, 128).await.unwrap(); // 50% brightness
    /// }
    /// ```
    pub async fn analog_write(&mut self, pin: u8, val: u8) -> Result<(), Aw9523Error<I2C::Error>> {
        let reg = match pin {
            0..=7 => 0x24 + pin,
            8..=11 => 0x20 + pin - 8,
            12..=15 => 0x2C + pin - 12,
            _ => return Err(Aw9523Error::InvalidArgument),
        };

        self.write_register(&[reg, val]).await
    }

    /// Sets the digital output state for a single pin asynchronously.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `state` - Output state ([`HIGH`](crate::HIGH) / [`LOW`](crate::LOW) or [`PinState::High`] / [`PinState::Low`])
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{r#async::Aw9523Async, HIGH};
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     gpio.digital_write(5, HIGH).await.unwrap(); // Set pin 5 high
    /// }
    /// ```
    pub async fn digital_write(
        &mut self,
        pin: u8,
        state: PinState,
    ) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let (reg, bit) = if pin < 8 {
            (AW9523_REG_OUTPUT0, pin)
        } else {
            (AW9523_REG_OUTPUT0 + 1, pin - 8)
        };

        let mut buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[reg], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        let new_val = match state {
            PinState::High => buffer[0] | (1 << bit),
            PinState::Low => buffer[0] & !(1 << bit),
        };

        self.write_register(&[reg, new_val]).await
    }

    /// Reads the digital input state for a single pin asynchronously.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    ///
    /// # Returns
    ///
    /// `true` if the pin is high, `false` if low
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::r#async::Aw9523Async;
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     let state = gpio.digital_read(8).await.unwrap();
    ///     if state {
    ///         // Pin 8 is high
    ///     }
    /// }
    /// ```
    pub async fn digital_read(&mut self, pin: u8) -> Result<bool, Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let (reg, bit) = if pin < 8 {
            (AW9523_REG_INPUT0, pin)
        } else {
            (AW9523_REG_INPUT1, pin - 8)
        };

        let mut buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[reg], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        Ok(buffer[0] & (1 << bit) != 0)
    }

    /// Enables or disables interrupt detection for a single pin asynchronously.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `enabled` - `true` to enable interrupts, `false` to disable
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Note
    ///
    /// The AW9523 uses inverted logic: writing 0 enables interrupts, 1 disables them.
    /// This method handles the inversion automatically.
    pub async fn enable_interrupt(
        &mut self,
        pin: u8,
        enabled: bool,
    ) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let (reg, bit) = if pin < 8 {
            (AW9523_REG_INTENABLE0, pin)
        } else {
            (AW9523_REG_INTENABLE0 + 1, pin - 8)
        };

        let mut buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[reg], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        // Inverted logic: 0 = enabled, 1 = disabled
        let new_val = if enabled {
            buffer[0] & !(1 << bit)
        } else {
            buffer[0] | (1 << bit)
        };

        self.write_register(&[reg, new_val]).await
    }

    /// Configures the mode for a single pin asynchronously.
    ///
    /// This method configures both direction (input/output) and whether the pin operates
    /// as a GPIO or LED driver.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `mode` - Pin configuration mode:
    ///   - [`PinMode::Input`] - Digital input
    ///   - [`PinMode::Output`] - Digital output
    ///   - [`PinMode::LedMode`] - LED driver with current control
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{r#async::Aw9523Async, PinMode};
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     gpio.pin_mode(0, PinMode::Output).await.unwrap();
    ///     gpio.pin_mode(1, PinMode::LedMode).await.unwrap();
    /// }
    /// ```
    pub async fn pin_mode(
        &mut self,
        pin: u8,
        mode: PinMode,
    ) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let (config_reg, led_reg, bit) = if pin < 8 {
            (AW9523_REG_CONFIG0, AW9523_REG_LEDMODE, pin)
        } else {
            (AW9523_REG_CONFIG1, AW9523_REG_LEDMODE + 1, pin - 8)
        };

        // Read current config register
        let mut config_buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[config_reg], &mut config_buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        // Read current LED mode register
        let mut led_buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[led_reg], &mut led_buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        // Configure based on mode
        let (new_config, new_led) = match mode {
            PinMode::Output => {
                // Output: CONFIG bit = 0, LED mode bit = 1 (GPIO mode)
                (config_buffer[0] & !(1 << bit), led_buffer[0] | (1 << bit))
            }
            PinMode::Input => {
                // Input: CONFIG bit = 1, LED mode bit = 1 (GPIO mode)
                (config_buffer[0] | (1 << bit), led_buffer[0] | (1 << bit))
            }
            PinMode::LedMode => {
                // LED mode: CONFIG bit = 0 (output), LED mode bit = 0
                (config_buffer[0] & !(1 << bit), led_buffer[0] & !(1 << bit))
            }
        };

        // Write updated values
        self.write_register(&[config_reg, new_config]).await?;
        self.write_register(&[led_reg, new_led]).await?;

        Ok(())
    }

    /// Configures Port 0 (pins 0-7) as open-drain or push-pull asynchronously.
    ///
    /// # Arguments
    ///
    /// * `open_drain` - `true` for open-drain mode, `false` for push-pull mode
    ///
    /// # Note
    ///
    /// This setting only affects Port 0. Port 1 (pins 8-15) always operates in push-pull mode.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::r#async::Aw9523Async;
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     gpio.open_drain_port0(true).await.unwrap(); // Enable open-drain for port 0
    /// }
    /// ```
    pub async fn open_drain_port0(
        &mut self,
        open_drain: bool,
    ) -> Result<(), Aw9523Error<I2C::Error>> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[AW9523_REG_GCR], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;

        // Bit 4: 0 = open-drain, 1 = push-pull
        let new_val = if open_drain {
            buffer[0] & !(1 << 4)
        } else {
            buffer[0] | (1 << 4)
        };

        self.write_register(&[AW9523_REG_GCR, new_val]).await
    }

    /// Reads the chip ID register asynchronously.
    ///
    /// Should return 0x23 for a valid AW9523 device.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::r#async::Aw9523Async;
    /// # let i2c = todo!();
    /// async fn example() {
    ///     let mut gpio = Aw9523Async::new(i2c, 0x58);
    ///     let chip_id = gpio.get_chip_id().await.unwrap();
    ///     assert_eq!(chip_id, 0x23);
    /// }
    /// ```
    pub async fn get_chip_id(&mut self) -> Result<u8, Aw9523Error<I2C::Error>> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.addr, &[crate::AW9523_REG_CHIPID], &mut buffer)
            .await
            .map_err(Aw9523Error::ReadError)?;
        Ok(buffer[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;
    use std::vec::Vec;

    /// Simple async I2C mock for testing
    /// Stores expected transactions and verifies them in order
    struct AsyncI2cMock {
        expectations: RefCell<Vec<I2cTransaction>>,
        index: RefCell<usize>,
    }

    enum I2cTransaction {
        Write {
            addr: u8,
            data: Vec<u8>,
        },
        WriteRead {
            addr: u8,
            write: Vec<u8>,
            read: Vec<u8>,
        },
    }

    impl AsyncI2cMock {
        fn new(expectations: Vec<I2cTransaction>) -> Self {
            Self {
                expectations: RefCell::new(expectations),
                index: RefCell::new(0),
            }
        }

        fn done(&self) {
            let index = *self.index.borrow();
            let total = self.expectations.borrow().len();
            assert_eq!(
                index, total,
                "Not all expected I2C transactions were executed: {}/{}",
                index, total
            );
        }
    }

    impl embedded_hal_async::i2c::ErrorType for AsyncI2cMock {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal_async::i2c::I2c for AsyncI2cMock {
        async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error> {
            let mut index = self.index.borrow_mut();
            let expectations = self.expectations.borrow();

            assert!(
                *index < expectations.len(),
                "Unexpected I2C write at index {}",
                *index
            );

            match &expectations[*index] {
                I2cTransaction::Write {
                    addr: expected_addr,
                    data: expected_data,
                } => {
                    assert_eq!(
                        addr, *expected_addr,
                        "Write address mismatch at index {}",
                        *index
                    );
                    assert_eq!(
                        data,
                        expected_data.as_slice(),
                        "Write data mismatch at index {}",
                        *index
                    );
                }
                _ => panic!("Expected Write transaction at index {}", *index),
            }

            *index += 1;
            Ok(())
        }

        async fn read(&mut self, _addr: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
            unimplemented!("read not used in tests")
        }

        async fn write_read(
            &mut self,
            addr: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            let mut index = self.index.borrow_mut();
            let expectations = self.expectations.borrow();

            assert!(
                *index < expectations.len(),
                "Unexpected I2C write_read at index {}",
                *index
            );

            match &expectations[*index] {
                I2cTransaction::WriteRead {
                    addr: expected_addr,
                    write: expected_write,
                    read: expected_read,
                } => {
                    assert_eq!(
                        addr, *expected_addr,
                        "WriteRead address mismatch at index {}",
                        *index
                    );
                    assert_eq!(
                        write,
                        expected_write.as_slice(),
                        "WriteRead write data mismatch at index {}",
                        *index
                    );
                    read.copy_from_slice(expected_read);
                }
                _ => panic!("Expected WriteRead transaction at index {}", *index),
            }

            *index += 1;
            Ok(())
        }

        async fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [embedded_hal_async::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            unimplemented!("transaction not used in tests")
        }
    }

    // Test helper macro for creating write transactions
    macro_rules! write_transaction {
        ($addr:expr, $reg:expr, $val:expr) => {
            I2cTransaction::Write {
                addr: $addr,
                data: vec![$reg, $val],
            }
        };
    }

    // Test helper macro for creating write-read transactions
    macro_rules! write_read_transaction {
        ($addr:expr, $reg:expr, $read:expr) => {
            I2cTransaction::WriteRead {
                addr: $addr,
                write: vec![$reg],
                read: vec![$read],
            }
        };
    }

    const TEST_ADDR: u8 = 0x58;

    /// Test async initialization
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_init_writes_default_config() {
        let expectations = vec![
            write_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0, 0b00000101),
            write_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0 + 1, 0b00000011),
            write_transaction!(TEST_ADDR, AW9523_REG_CONFIG0, 0b00011000),
            write_transaction!(TEST_ADDR, AW9523_REG_CONFIG0 + 1, 0b00001100),
            write_transaction!(TEST_ADDR, AW9523_REG_GCR, 0b00010000),
            write_transaction!(TEST_ADDR, AW9523_REG_LEDMODE + 1, 0b11111111),
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.init().await.unwrap();

        dev.i2c.done();
    }

    /// Test async reset
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_reset_writes_to_reset_register() {
        let expectations = vec![write_transaction!(TEST_ADDR, AW9523_REG_SOFTRESET, 0x00)];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.reset().await.unwrap();

        dev.i2c.done();
    }

    /// Test async digital write
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_digital_write_sets_pin_high() {
        let expectations = vec![
            write_read_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0, 0b00000000),
            write_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0, 0b00000001),
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.digital_write(0, PinState::High).await.unwrap();

        dev.i2c.done();
    }

    /// Test async digital read
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_digital_read_returns_pin_state() {
        let expectations = vec![write_read_transaction!(
            TEST_ADDR,
            AW9523_REG_INPUT0,
            0b00100000
        )];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        let state = dev.digital_read(5).await.unwrap();

        assert!(state);
        dev.i2c.done();
    }

    /// Test async pin mode configuration
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_pin_mode_output_configures_correctly() {
        let expectations = vec![
            write_read_transaction!(TEST_ADDR, AW9523_REG_CONFIG0, 0b11111111),
            write_read_transaction!(TEST_ADDR, AW9523_REG_LEDMODE, 0b00000000),
            write_transaction!(TEST_ADDR, AW9523_REG_CONFIG0, 0b11111011), // Clear bit 2
            write_transaction!(TEST_ADDR, AW9523_REG_LEDMODE, 0b00000100), // Set bit 2
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.pin_mode(2, PinMode::Output).await.unwrap();

        dev.i2c.done();
    }

    /// Test async LED mode configuration
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_pin_mode_led_configures_correctly() {
        let expectations = vec![
            write_read_transaction!(TEST_ADDR, AW9523_REG_CONFIG0 + 1, 0b11111111),
            write_read_transaction!(TEST_ADDR, AW9523_REG_LEDMODE + 1, 0b11111111),
            write_transaction!(TEST_ADDR, AW9523_REG_CONFIG0 + 1, 0b11101111), // Clear bit 4
            write_transaction!(TEST_ADDR, AW9523_REG_LEDMODE + 1, 0b11101111), // Clear bit 4
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.pin_mode(12, PinMode::LedMode).await.unwrap();

        dev.i2c.done();
    }

    /// Test async analog write
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_analog_write_sets_brightness() {
        let expectations = vec![write_transaction!(TEST_ADDR, 0x24, 128)]; // Pin 0, 50% brightness

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.analog_write(0, 128).await.unwrap();

        dev.i2c.done();
    }

    /// Test async bulk operations
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_output_gpio_sets_all_pins() {
        let expectations = vec![
            write_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0, 0x55),
            write_transaction!(TEST_ADDR, AW9523_REG_OUTPUT0 + 1, 0xAA),
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.output_gpio(0xAA55).await.unwrap();

        dev.i2c.done();
    }

    /// Test async input_gpio reads all pins
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_input_gpio_reads_all_pins() {
        let expectations = vec![I2cTransaction::WriteRead {
            addr: TEST_ADDR,
            write: vec![AW9523_REG_INPUT0],
            read: vec![0x12, 0x34],
        }];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        let inputs = dev.input_gpio().await.unwrap();

        assert_eq!(inputs, 0x3412);
        dev.i2c.done();
    }

    /// Test async interrupt enable
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_enable_interrupt_configures_pin() {
        let expectations = vec![
            write_read_transaction!(TEST_ADDR, AW9523_REG_INTENABLE0, 0xFF),
            write_transaction!(TEST_ADDR, AW9523_REG_INTENABLE0, 0xFE), // Clear bit 0 to enable
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.enable_interrupt(0, true).await.unwrap();

        dev.i2c.done();
    }

    /// Test async open drain configuration
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_open_drain_port0_enables_open_drain() {
        let expectations = vec![
            write_read_transaction!(TEST_ADDR, AW9523_REG_GCR, 0b00010000),
            write_transaction!(TEST_ADDR, AW9523_REG_GCR, 0b00000000), // Clear bit 4 for open-drain
        ];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        dev.open_drain_port0(true).await.unwrap();

        dev.i2c.done();
    }

    /// Test async chip ID read
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_get_chip_id_returns_correct_value() {
        let expectations = vec![write_read_transaction!(
            TEST_ADDR,
            crate::AW9523_REG_CHIPID,
            0x23
        )];

        let i2c = AsyncI2cMock::new(expectations);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        let chip_id = dev.get_chip_id().await.unwrap();

        assert_eq!(chip_id, 0x23);
        dev.i2c.done();
    }

    /// Test async error handling for invalid pin
    #[cfg(all(test, feature = "async"))]
    #[tokio::test]
    async fn async_digital_write_rejects_invalid_pin() {
        let i2c = AsyncI2cMock::new(vec![]);
        let mut dev = Aw9523Async::new(i2c, TEST_ADDR);

        let result = dev.digital_write(16, PinState::High).await;

        assert!(matches!(result, Err(Aw9523Error::InvalidArgument)));
    }

    #[test]
    fn async_module_compiles() {
        // This test just ensures the async module compiles correctly
        // when the async feature is not enabled
    }
}
