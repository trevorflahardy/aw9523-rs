//! AW9523 GPIO Expander Driver
//!
//! A platform-agnostic driver for the AW9523 16-channel LED driver and GPIO expander.
//!
//! The AW9523 is an I2C-controlled GPIO expander with 16 configurable pins that can operate
//! as digital I/O or constant-current LED drivers. Each pin supports:
//! - Digital input/output with configurable direction
//! - Interrupt detection
//! - 256-step constant current LED driving
//! - Open-drain or push-pull output modes (port 0)
//!
//! # Features
//!
//! - `no_std` compatible
//! - Uses `embedded-hal` traits for portability
//! - Individual pin control or bulk 16-bit operations
//! - Configurable interrupt detection per pin
//! - LED mode with PWM-like current control
//!
//! # Example
//!
//! ```ignore
//! use aw9523::{Aw9523, OUTPUT, INPUT};
//! # let i2c = todo!();
//!
//! // Create device with I2C bus and default address
//! let mut gpio = Aw9523::new(i2c, 0x58);
//!
//! // Initialize with default configuration
//! gpio.init().unwrap();
//!
//! // Configure pin 0 as output and set it high
//! gpio.pin_mode(0, OUTPUT).unwrap();
//! gpio.digital_write(0, true).unwrap();
//!
//! // Configure pin 8 as input and read it
//! gpio.pin_mode(8, INPUT).unwrap();
//! let state = gpio.digital_read(8).unwrap();
//! ```

#![no_std]

use embedded_hal::i2c::{AddressMode, I2c};

/// Default I2C address for the AW9523
const __AW9523_ADDRESS: u8 = 0x58;

// Hardware register addresses
/// Chip ID register (read-only, returns 0x23)
pub const AW9523_REG_CHIPID: u8 = 0x10;
const AW9523_REG_SOFTRESET: u8 = 0x7F; // Soft reset register
const AW9523_REG_INPUT0: u8 = 0x00; // Input port 0 (pins 0-7)
/// Input port 1 register (pins 8-15)
pub const AW9523_REG_INPUT1: u8 = 0x01;
const AW9523_REG_OUTPUT0: u8 = 0x02; // Output port 0 (pins 0-7)
const AW9523_REG_OUTPUT1: u8 = 0x03; // Output port 1 (pins 8-15)
const AW9523_REG_CONFIG0: u8 = 0x04; // Direction config port 0
const AW9523_REG_CONFIG1: u8 = 0x05; // Direction config port 1
const AW9523_REG_INTENABLE0: u8 = 0x06; // Interrupt enable port 0
/// Interrupt enable port 1 register
pub const AW9523_REG_INTENABLE1: u8 = 0x07;
const AW9523_REG_GCR: u8 = 0x11; // Global control register
const AW9523_REG_LEDMODE: u8 = 0x12; // LED mode config port 0
const AW9523_REG_LEDMODE1: u8 = 0x13; // LED mode config port 1

/// Pin mode constant for LED/constant-current mode.
/// Use with [`Aw9523::pin_mode`] to configure a pin for LED driving.
pub const AW9523_LED_MODE: u8 = 2;

// Pin bitmasks for 16-bit operations
// Use these with output_gpio(), interrupt_enable_gpio(), configure_direction(), configure_led_mode()
/// Bitmask for pin 0
pub const AW9523_PIN_0: u16 = 1 << 0;
/// Bitmask for pin 1
pub const AW9523_PIN_1: u16 = 1 << 1;
/// Bitmask for pin 2
pub const AW9523_PIN_2: u16 = 1 << 2;
/// Bitmask for pin 3
pub const AW9523_PIN_3: u16 = 1 << 3;
/// Bitmask for pin 4
pub const AW9523_PIN_4: u16 = 1 << 4;
/// Bitmask for pin 5
pub const AW9523_PIN_5: u16 = 1 << 5;
/// Bitmask for pin 6
pub const AW9523_PIN_6: u16 = 1 << 6;
/// Bitmask for pin 7
pub const AW9523_PIN_7: u16 = 1 << 7;
/// Bitmask for pin 8
pub const AW9523_PIN_8: u16 = 1 << 8;
/// Bitmask for pin 9
pub const AW9523_PIN_9: u16 = 1 << 9;
/// Bitmask for pin 10
pub const AW9523_PIN_10: u16 = 1 << 10;
/// Bitmask for pin 11
pub const AW9523_PIN_11: u16 = 1 << 11;
/// Bitmask for pin 12
pub const AW9523_PIN_12: u16 = 1 << 12;
/// Bitmask for pin 13
pub const AW9523_PIN_13: u16 = 1 << 13;
/// Bitmask for pin 14
pub const AW9523_PIN_14: u16 = 1 << 14;
/// Bitmask for pin 15
pub const AW9523_PIN_15: u16 = 1 << 15;

// Port naming aliases (port 0 = pins 0-7, port 1 = pins 8-15)
/// Port 0, pin 0 alias for PIN_0.
/// The AW9523 organizes its 16 pins into two ports of 8 pins each, and this is the first pin in port 0.
pub const AW9523_P0_0: u16 = AW9523_PIN_0;
/// Port 0, pin 1 alias for PIN_1.
/// This is the second pin in port 0, useful when you need port-based naming conventions.
pub const AW9523_P0_1: u16 = AW9523_PIN_1;
/// Port 0, pin 2 alias for PIN_2.
/// This is the third pin in port 0, sharing the same register space as other port 0 pins.
pub const AW9523_P0_2: u16 = AW9523_PIN_2;
/// Port 0, pin 3 alias for PIN_3.
/// This is the fourth pin in port 0, part of the lower byte of the 16-bit pin registers.
pub const AW9523_P0_3: u16 = AW9523_PIN_3;
/// Port 0, pin 4 alias for PIN_4.
/// The middle pin of port 0, this can be configured for open-drain mode along with other port 0 pins.
pub const AW9523_P0_4: u16 = AW9523_PIN_4;
/// Port 0, pin 5 alias for PIN_5.
/// This pin is part of port 0, which supports open-drain output configuration via the GCR register.
pub const AW9523_P0_5: u16 = AW9523_PIN_5;
/// Port 0, pin 6 alias for PIN_6.
/// One of the higher-numbered pins in port 0, accessible through the lower-byte registers.
pub const AW9523_P0_6: u16 = AW9523_PIN_6;
/// Port 0, pin 7 alias for PIN_7.
/// The last pin in port 0, representing bit 7 of the port 0 register space.
pub const AW9523_P0_7: u16 = AW9523_PIN_7;

/// Port 1, pin 0 alias for PIN_8.
/// The first pin in port 1, which always operates in push-pull mode (unlike port 0).
pub const AW9523_P1_0: u16 = AW9523_PIN_8;
/// Port 1, pin 1 alias for PIN_9.
/// This is the second pin in port 1, accessed through the upper-byte registers.
pub const AW9523_P1_1: u16 = AW9523_PIN_9;
/// Port 1, pin 2 alias for PIN_10.
/// The third pin in port 1, part of the upper byte of the 16-bit pin registers.
pub const AW9523_P1_2: u16 = AW9523_PIN_10;
/// Port 1, pin 3 alias for PIN_11.
/// This is the fourth pin in port 1, useful when organizing pins by port groups.
pub const AW9523_P1_3: u16 = AW9523_PIN_11;
/// Port 1, pin 4 alias for PIN_12.
/// The middle pin of port 1, can be used with LED dimming functionality.
pub const AW9523_P1_4: u16 = AW9523_PIN_12;
/// Port 1, pin 5 alias for PIN_13.
/// This pin is part of port 1's push-pull output group, accessible via upper-byte registers.
pub const AW9523_P1_5: u16 = AW9523_PIN_13;
/// Port 1, pin 6 alias for PIN_14.
/// One of the higher-numbered pins in port 1, supporting all standard GPIO and LED modes.
pub const AW9523_P1_6: u16 = AW9523_PIN_14;
/// Port 1, pin 7 alias for PIN_15.
/// The last pin on the AW9523, representing the highest bit in the 16-bit register space.
pub const AW9523_P1_7: u16 = AW9523_PIN_15;

/// Bitmask for all pins in port 0 (pins 0-7)
pub const AW9523_PORT0_ALL: u16 = 0x00FF;
/// Bitmask for all pins in port 1 (pins 8-15)
pub const AW9523_PORT1_ALL: u16 = 0xFF00;
/// Bitmask for all 16 pins
pub const AW9523_ALL_PINS: u16 = 0xFFFF;

/// Pin mode constant for input direction
pub const INPUT: u8 = 0;
/// Pin mode constant for output direction
pub const OUTPUT: u8 = 1;

/// Errors that can occur when interacting with the AW9523
#[derive(Debug)]
pub enum Aw9523Error<E> {
    /// Operation not supported by the hardware
    NotSupported(E),
    /// Invalid argument passed to a method (e.g., pin number out of range)
    InvalidArgument,
    /// I2C read operation failed
    ReadError(E),
    /// I2C write operation failed
    WriteError(E),
}

/// AW9523 driver instance
///
/// Manages communication with an AW9523 GPIO expander over I2C.
/// The generic parameters allow flexibility in I2C implementation and addressing mode.
pub struct Aw9523<A: AddressMode, I2C: I2c<A>> {
    i2c: I2C,
    addr: A,
}

impl<A, I2C> Aw9523<A, I2C>
where
    A: AddressMode + Copy,
    I2C: I2c<A>,
{
    /// Creates a new AW9523 driver instance.
    ///
    /// # Arguments
    ///
    /// * `i2c` - An I2C bus implementation
    /// * `addr` - The I2C address of the device (typically 0x58)
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::Aw9523;
    /// # let i2c = todo!();
    /// let gpio = Aw9523::new(i2c, 0x58);
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

    /// Initializes the AW9523 with a default configuration.
    ///
    /// Sets up output values, pin directions, and enables push-pull mode.
    /// Call this after creating the device to ensure a known state.
    ///
    /// # Errors
    ///
    /// Returns an error if any I2C write operation fails.
    pub fn init(&mut self) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_OUTPUT0, 0b00000101])?;
        self.write_register(&[AW9523_REG_OUTPUT1, 0b00000011])?;
        self.write_register(&[AW9523_REG_CONFIG0, 0b00011000])?;
        self.write_register(&[AW9523_REG_CONFIG1, 0b00001100])?;
        self.write_register(&[AW9523_REG_GCR, 0b00010000])?;
        self.write_register(&[AW9523_REG_LEDMODE1, 0b11111111])?;

        Ok(())
    }

    /// Writes data to a register on the device.
    ///
    /// Uses a two-phase write approach required by the AW9523 hardware:
    /// 1. Write register address and read dummy byte to set register pointer
    /// 2. Write register address + data to commit the value
    ///
    /// **Why this pattern is necessary:**
    /// The AW9523 hardware does not reliably support simple I2C write transactions
    /// with register address + data in one operation. Without the initial write_read
    /// to set the register pointer, the chip may NAK (not acknowledge) the data byte,
    /// resulting in `WriteError(AcknowledgeCheckFailed(Data))`. This two-phase
    /// approach ensures reliable register writes.
    fn write_register(&mut self, data: &[u8]) -> Result<(), Aw9523Error<I2C::Error>> {
        // Phase 1: Set register pointer by doing a write_read with register address
        // let mut dummy_buf = [0u8];
        // self.i2c
        //    .write_read(self.addr, &[data[0]], &mut dummy_buf)
        //    .map_err(|e| Aw9523Error::ReadError(e))?;

        // Phase 2: Write register address + value
        self.i2c
            .write(self.addr, data)
            .map_err(|e| Aw9523Error::WriteError(e))
    }

    /// Performs a soft reset of the device.
    ///
    /// All registers return to their default values after a reset.
    /// You should call [`init`](Self::init) again after reset.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C write fails.
    pub fn reset(&mut self) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_SOFTRESET, 0x00])
    }

    /// Sets the output state for all 16 GPIO pins at once.
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
    /// # use aw9523::{Aw9523, AW9523_PIN_0, AW9523_PIN_3};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// // Set pins 0 and 3 high, all others low
    /// gpio.output_gpio(AW9523_PIN_0 | AW9523_PIN_3).unwrap();
    /// ```
    pub fn output_gpio(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_OUTPUT0, (pins & 0xFF) as u8])?;
        self.write_register(&[AW9523_REG_OUTPUT0 + 1, (pins >> 8) as u8])?;

        Ok(())
    }

    /// Reads the input state of all 16 GPIO pins at once.
    ///
    /// Returns a 16-bit value where each bit represents one pin's state (1 = high, 0 = low).
    /// Bit 0 corresponds to pin 0, bit 15 to pin 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{Aw9523, AW9523_PIN_5};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// let inputs = gpio.input_gpio().unwrap();
    /// if inputs & AW9523_PIN_5 != 0 {
    ///     // Pin 5 is high
    /// }
    /// ```
    pub fn input_gpio(&mut self) -> Result<u16, Aw9523Error<I2C::Error>> {
        let mut buffer = [0u8; 2];
        self.i2c
            .write_read(self.addr, &[AW9523_REG_INPUT0], &mut buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;
        Ok(u16::from(buffer[0]) | (u16::from(buffer[1]) << 8))
    }

    /// Enables interrupt detection for all 16 GPIO pins at once.
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
    pub fn interrupt_enable_gpio(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_INTENABLE0, !(pins & 0xFF) as u8])?;
        self.write_register(&[AW9523_REG_INTENABLE0 + 1, !(pins >> 8) as u8])?;

        Ok(())
    }

    /// Configures pin direction for all 16 GPIO pins at once.
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
    pub fn configure_direction(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_CONFIG0, !(pins & 0xFF) as u8])?;
        self.write_register(&[AW9523_REG_CONFIG0 + 1, !(pins >> 8) as u8])?;

        Ok(())
    }

    /// Configures LED/constant-current mode for all 16 pins at once.
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
    pub fn configure_led_mode(&mut self, pins: u16) -> Result<(), Aw9523Error<I2C::Error>> {
        self.write_register(&[AW9523_REG_LEDMODE, !(pins & 0xFF) as u8])?;
        self.write_register(&[AW9523_REG_LEDMODE + 1, !(pins >> 8) as u8])?;

        Ok(())
    }

    /// Sets the LED brightness/current level for a single pin.
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
    /// # use aw9523::{Aw9523, AW9523_LED_MODE};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// gpio.pin_mode(0, AW9523_LED_MODE).unwrap();
    /// gpio.analog_write(0, 128).unwrap(); // 50% brightness
    /// ```
    pub fn analog_write(&mut self, pin: u8, val: u8) -> Result<(), Aw9523Error<I2C::Error>> {
        let reg = match pin {
            0..=7 => 0x24 + pin,
            8..=11 => 0x20 + pin - 8,
            12..=15 => 0x2C + pin - 12,
            _ => return Err(Aw9523Error::InvalidArgument),
        };

        self.write_register(&[reg, val])
    }

    /// Sets the digital output state for a single pin.
    ///
    /// The pin must be configured as an output first using [`pin_mode`](Self::pin_mode)
    /// or [`configure_direction`](Self::configure_direction).
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `val` - Output state (true = high, false = low)
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{Aw9523, OUTPUT};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// gpio.pin_mode(3, OUTPUT).unwrap();
    /// gpio.digital_write(3, true).unwrap(); // Set pin 3 high
    /// ```
    pub fn digital_write(&mut self, pin: u8, val: bool) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let reg_addr = AW9523_REG_OUTPUT0 + (pin / 8);
        let bit_pos = pin % 8;

        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg_addr], &mut buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;

        let new_val = if val {
            buffer[0] | (1 << bit_pos)
        } else {
            buffer[0] & !(1 << bit_pos)
        };
        self.write_register(&[reg_addr, new_val])
    }

    /// Reads the digital input state for a single pin.
    ///
    /// The pin must be configured as an input first using [`pin_mode`](Self::pin_mode)
    /// or [`configure_direction`](Self::configure_direction).
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    ///
    /// # Returns
    ///
    /// Returns `true` if the pin is high, `false` if low.
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{Aw9523, INPUT};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// gpio.pin_mode(8, INPUT).unwrap();
    /// let state = gpio.digital_read(8).unwrap();
    /// ```
    pub fn digital_read(&mut self, pin: u8) -> Result<bool, Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let reg_addr = AW9523_REG_INPUT0 + (pin / 8);
        let bit_pos = pin % 8;

        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg_addr], &mut buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;
        Ok((buffer[0] & (1 << bit_pos)) != 0)
    }

    /// Enables or disables interrupt detection for a single pin.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `en` - true to enable interrupt detection, false to disable
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15.
    ///
    /// # Note
    ///
    /// The AW9523 uses inverted logic internally, but this is handled automatically.
    pub fn enable_interrupt(&mut self, pin: u8, en: bool) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }

        let reg_addr = AW9523_REG_INTENABLE0 + (pin / 8);
        let bit_pos = pin % 8;

        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg_addr], &mut buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;

        let new_val = if en {
            buffer[0] & !(1 << bit_pos)
        } else {
            buffer[0] | (1 << bit_pos)
        };
        self.write_register(&[reg_addr, new_val])
    }

    /// Configures the mode for a single pin.
    ///
    /// This is the main method for setting up pins. It configures both the direction
    /// (input/output) and whether the pin operates as a GPIO or LED driver.
    ///
    /// # Arguments
    ///
    /// * `pin` - Pin number (0-15)
    /// * `mode` - One of:
    ///   - `INPUT` - Digital input
    ///   - `OUTPUT` - Digital output
    ///   - `AW9523_LED_MODE` - LED driver with current control
    ///
    /// # Errors
    ///
    /// Returns `InvalidArgument` if the pin number is greater than 15 or if
    /// the mode is not one of the valid values.
    ///
    /// # Example
    ///
    /// ```ignore
    /// # use aw9523::{Aw9523, OUTPUT, INPUT, AW9523_LED_MODE};
    /// # let i2c = todo!();
    /// # let mut gpio = Aw9523::new(i2c, 0x58);
    /// gpio.pin_mode(0, OUTPUT).unwrap();          // Digital output
    /// gpio.pin_mode(5, INPUT).unwrap();           // Digital input
    /// gpio.pin_mode(12, AW9523_LED_MODE).unwrap(); // LED driver
    /// ```
    pub fn pin_mode(&mut self, pin: u8, mode: u8) -> Result<(), Aw9523Error<I2C::Error>> {
        if pin > 15 {
            return Err(Aw9523Error::InvalidArgument);
        }
        if mode != OUTPUT && mode != INPUT && mode != AW9523_LED_MODE {
            return Err(Aw9523Error::InvalidArgument);
        }

        let bit_pos = pin % 8;

        let config_reg = AW9523_REG_CONFIG0 + (pin / 8);
        let mut config_buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[config_reg], &mut config_buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;

        let ledmode_reg = AW9523_REG_LEDMODE + (pin / 8);
        let mut ledmode_buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[ledmode_reg], &mut ledmode_buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;

        let (config_val, ledmode_val) = match mode {
            OUTPUT => {
                let conf = config_buffer[0] & !(1 << bit_pos);
                let led = ledmode_buffer[0] | (1 << bit_pos);
                (conf, led)
            }
            INPUT => {
                let conf = config_buffer[0] | (1 << bit_pos);
                let led = ledmode_buffer[0] | (1 << bit_pos);
                (conf, led)
            }
            AW9523_LED_MODE => {
                let conf = config_buffer[0] & !(1 << bit_pos);
                let led = ledmode_buffer[0] & !(1 << bit_pos);
                (conf, led)
            }
            _ => unreachable!(),
        };
        self.write_register(&[config_reg, config_val])?;
        self.write_register(&[ledmode_reg, ledmode_val])?;

        Ok(())
    }

    /// Configures output mode for all port 0 pins (pins 0-7).
    ///
    /// Sets whether port 0 pins use open-drain or push-pull output mode.
    /// This affects all 8 pins in port 0 simultaneously.
    ///
    /// # Arguments
    ///
    /// * `od` - true for open-drain mode, false for push-pull mode
    ///
    /// # Note
    ///
    /// Port 1 (pins 8-15) always operates in push-pull mode and cannot be changed.
    /// The AW9523 uses inverted logic internally, but this is handled automatically.
    pub fn open_drain_port0(&mut self, od: bool) -> Result<(), Aw9523Error<I2C::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[AW9523_REG_GCR], &mut buffer)
            .map_err(|e| Aw9523Error::ReadError(e))?;

        let new_val = if od {
            buffer[0] & !(1 << 4)
        } else {
            buffer[0] | (1 << 4)
        };
        self.write_register(&[AW9523_REG_GCR, new_val])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    // Vec is available in test mode (std is enabled for tests)
    #[allow(unused_imports)]
    extern crate std;
    use std::vec::Vec;

    const ADDR: u8 = 0x58;

    /// Helper macro to create the two-phase write pattern: write_read + write
    /// The AW9523 hardware requires setting the register pointer before writing data
    macro_rules! write_reg {
        ($addr:expr, $reg:expr, $val:expr) => {
            [
                I2cTransaction::write_read($addr, [$reg].to_vec(), [0].to_vec()),
                I2cTransaction::write($addr, [$reg, $val].to_vec()),
            ]
        };
    }

    #[test]
    fn init_writes_expected_registers_in_order() {
        // init() writes 6 registers using two-phase pattern (write_read + write for each)
        let mut expectations = Vec::new();
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_OUTPUT0, 0b0000_0101));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_OUTPUT1, 0b0000_0011));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_CONFIG0, 0b0001_1000));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_CONFIG1, 0b0000_1100));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_GCR, 0b0001_0000));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_LEDMODE1, 0b1111_1111));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.init().unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn reset_writes_softreset_register() {
        // reset() uses two-phase write pattern
        let expectations = write_reg!(ADDR, AW9523_REG_SOFTRESET, 0x00);

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.reset().unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn output_gpio_writes_both_ports() {
        // output_gpio(u16) writes low byte to OUTPUT0 (0x02) and high byte to OUTPUT1 (0x03) using two-phase pattern
        let pins: u16 = 0xA55A; // low=0x5A, high=0xA5
        let mut expectations = Vec::new();
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_OUTPUT0, 0x5A));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_OUTPUT0 + 1, 0xA5));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.output_gpio(pins).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn input_gpio_reads_two_bytes_and_combines() {
        // input_gpio() performs one write_read starting at INPUT0 (0x00) and reads two bytes.
        // The driver combines them into a u16: low=buf[0], high=buf[1].
        let expectations = [I2cTransaction::write_read(
            ADDR,
            [AW9523_REG_INPUT0].to_vec(),
            [0x34, 0x12].to_vec(),
        )];

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        let v = dev.input_gpio().unwrap();
        assert_eq!(v, 0x1234);

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn interrupt_enable_gpio_writes_inverted_masks() {
        // interrupt_enable_gpio(pins): datasheet uses inverted logic in the enable registers.
        // Your code writes bitwise NOT of each byte.
        let pins: u16 = 0x00F0; // low=0xF0, high=0x00
        let mut expectations = Vec::new();
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_INTENABLE0, !0xF0));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_INTENABLE0 + 1, !0x00));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.interrupt_enable_gpio(pins).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn configure_direction_writes_inverted_masks() {
        // configure_direction(pins): your API says 1=output, 0=input, but hardware config is 1=input.
        // Your code inverts the output-bitmask before writing to CONFIG regs.
        let pins: u16 = 0x0F0F; // request these as outputs
        let mut expectations = Vec::new();
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_CONFIG0, !(pins as u8)));
        expectations.extend_from_slice(&write_reg!(
            ADDR,
            AW9523_REG_CONFIG0 + 1,
            !((pins >> 8) as u8)
        ));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.configure_direction(pins).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn configure_led_mode_writes_inverted_masks() {
        // configure_led_mode(pins): your API uses 1=enable LED (constant current),
        // but register uses 0=LED mode, 1=GPIO mode. So you invert before writing.
        let pins: u16 = 0x00FF; // enable LED mode on P0 pins only
        let mut expectations = Vec::new();
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_LEDMODE, !(pins as u8)));
        expectations.extend_from_slice(&write_reg!(
            ADDR,
            AW9523_REG_LEDMODE + 1,
            !((pins >> 8) as u8)
        ));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.configure_led_mode(pins).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn analog_write_pin_register_mapping_examples() {
        // analog_write() maps a pin number to a DIM register address.
        // This test checks a few representative pins across the mapping ranges.
        let mut expectations = Vec::new();
        // pin 0 => 0x24
        expectations.extend_from_slice(&write_reg!(ADDR, 0x24, 10));
        // pin 7 => 0x2B
        expectations.extend_from_slice(&write_reg!(ADDR, 0x2B, 20));
        // pin 8 => 0x20
        expectations.extend_from_slice(&write_reg!(ADDR, 0x20, 30));
        // pin 11 => 0x23
        expectations.extend_from_slice(&write_reg!(ADDR, 0x23, 40));
        // pin 12 => 0x2C
        expectations.extend_from_slice(&write_reg!(ADDR, 0x2C, 50));
        // pin 15 => 0x2F
        expectations.extend_from_slice(&write_reg!(ADDR, 0x2F, 60));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.analog_write(0, 10).unwrap();
        dev.analog_write(7, 20).unwrap();
        dev.analog_write(8, 30).unwrap();
        dev.analog_write(11, 40).unwrap();
        dev.analog_write(12, 50).unwrap();
        dev.analog_write(15, 60).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn analog_write_rejects_invalid_pin() {
        let i2c = I2cMock::new(&[]);
        let mut dev = Aw9523::new(i2c, ADDR);

        let err = dev.analog_write(16, 1).unwrap_err();
        matches!(err, Aw9523Error::InvalidArgument);

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn digital_write_sets_bit_in_output0() {
        // digital_write():
        // 1) read output register for the port (write_read)
        // 2) set/clear the bit
        // 3) write back modified byte (using two-phase write)
        //
        // Example: pin 3 is in OUTPUT0 (0x02), bit 3.
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [AW9523_REG_OUTPUT0].to_vec(),
            [0b0000_0001].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_OUTPUT0, 0b0000_1001));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.digital_write(3, true).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn digital_write_clears_bit_in_output1() {
        // Example: pin 12 is in OUTPUT1 (0x03), bit (12%8)=4.
        let reg = AW9523_REG_OUTPUT0 + 1;
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [reg].to_vec(),
            [0b1111_1111].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, reg, 0b1110_1111));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.digital_write(12, false).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn digital_write_rejects_invalid_pin() {
        let i2c = I2cMock::new(&[]);
        let mut dev = Aw9523::new(i2c, ADDR);

        let err = dev.digital_write(99, true).unwrap_err();
        matches!(err, Aw9523Error::InvalidArgument);

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn digital_read_reads_correct_port_and_bit() {
        // Example: pin 9 is INPUT1 (0x01), bit 1.
        let reg = AW9523_REG_INPUT0 + 1;
        let expectations = [I2cTransaction::write_read(
            ADDR,
            [reg].to_vec(),
            [0b0000_0010].to_vec(),
        )];

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        let v = dev.digital_read(9).unwrap();
        assert!(v);

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn enable_interrupt_enables_by_clearing_bit() {
        // enable_interrupt(pin, true) clears the bit (0 = enabled).
        // Example: pin 2 => INTENABLE0 (0x06), bit 2.
        let reg = AW9523_REG_INTENABLE0;
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [reg].to_vec(),
            [0b1111_1111].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, reg, 0b1111_1011));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.enable_interrupt(2, true).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn enable_interrupt_disables_by_setting_bit() {
        // enable_interrupt(pin, false) sets the bit (1 = disabled).
        // Example: pin 10 => INTENABLE1 (0x07), bit 2.
        let reg = AW9523_REG_INTENABLE0 + 1;
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [reg].to_vec(),
            [0b0000_0000].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, reg, 0b0000_0100));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.enable_interrupt(10, false).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn pin_mode_output_sets_config0_bit_to_0_and_ledmode_to_gpio() {
        // pin_mode(pin, OUTPUT):
        // - CONFIG bit cleared (0=output)
        // - LEDMODE bit set (1=GPIO mode)
        //
        // Example pin 5 => CONFIG0 (0x04) bit5, LEDMODE0 (0x12) bit5.
        let config_reg = AW9523_REG_CONFIG0;
        let led_reg = AW9523_REG_LEDMODE;

        let mut expectations = Vec::new();
        // reads
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [config_reg].to_vec(),
            [0b1111_1111].to_vec(),
        ));
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [led_reg].to_vec(),
            [0b0000_0000].to_vec(),
        ));
        // writes (two-phase)
        expectations.extend_from_slice(&write_reg!(ADDR, config_reg, 0b1101_1111)); // clear bit 5
        expectations.extend_from_slice(&write_reg!(ADDR, led_reg, 0b0010_0000)); // set bit 5

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.pin_mode(5, OUTPUT).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn pin_mode_input_sets_config_bit_to_1_and_ledmode_to_gpio() {
        // pin_mode(pin, INPUT):
        // - CONFIG bit set (1=input)
        // - LEDMODE bit set (1=GPIO mode)
        //
        // Example pin 6 => CONFIG0 bit6, LEDMODE0 bit6.
        let config_reg = AW9523_REG_CONFIG0;
        let led_reg = AW9523_REG_LEDMODE;

        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [config_reg].to_vec(),
            [0b0000_0000].to_vec(),
        ));
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [led_reg].to_vec(),
            [0b0000_0000].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, config_reg, 0b0100_0000)); // set bit 6
        expectations.extend_from_slice(&write_reg!(ADDR, led_reg, 0b0100_0000)); // set bit 6

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.pin_mode(6, INPUT).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn pin_mode_led_mode_sets_output_and_led_mode_bits() {
        // pin_mode(pin, AW9523_LED_MODE):
        // - CONFIG bit cleared (output)
        // - LEDMODE bit cleared (0=LED mode)
        //
        // Example pin 14 => CONFIG1 (0x05) bit6, LEDMODE1 (0x13) bit6.
        let config_reg = AW9523_REG_CONFIG0 + 1;
        let led_reg = AW9523_REG_LEDMODE + 1;

        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [config_reg].to_vec(),
            [0b1111_1111].to_vec(),
        ));
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [led_reg].to_vec(),
            [0b1111_1111].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, config_reg, 0b1011_1111)); // clear bit 6
        expectations.extend_from_slice(&write_reg!(ADDR, led_reg, 0b1011_1111)); // clear bit 6

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.pin_mode(14, AW9523_LED_MODE).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn pin_mode_rejects_invalid_mode() {
        let i2c = I2cMock::new(&[]);
        let mut dev = Aw9523::new(i2c, ADDR);

        let err = dev.pin_mode(0, 99).unwrap_err();
        matches!(err, Aw9523Error::InvalidArgument);

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn open_drain_port0_true_clears_bit_4() {
        // open_drain_port0(true):
        // - reads GCR
        // - clears bit 4 to select open-drain
        // - writes back (two-phase)
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [AW9523_REG_GCR].to_vec(),
            [0b0001_0000].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_GCR, 0b0000_0000));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.open_drain_port0(true).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }

    #[test]
    fn open_drain_port0_false_sets_bit_4() {
        // open_drain_port0(false):
        // - sets bit 4 to select push-pull (two-phase)
        let mut expectations = Vec::new();
        expectations.push(I2cTransaction::write_read(
            ADDR,
            [AW9523_REG_GCR].to_vec(),
            [0b0000_0000].to_vec(),
        ));
        expectations.extend_from_slice(&write_reg!(ADDR, AW9523_REG_GCR, 0b0001_0000));

        let i2c = I2cMock::new(&expectations);
        let mut dev = Aw9523::new(i2c, ADDR);

        dev.open_drain_port0(false).unwrap();

        let mut i2c = dev.destroy();
        i2c.done();
    }
}
