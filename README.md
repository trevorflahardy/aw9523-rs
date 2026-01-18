# aw9523-embedded

![MSRV](https://img.shields.io/badge/MSRV-1.60-blue?style=flat-square)
[![Crates.io](https://img.shields.io/crates/v/aw9523-embedded.svg)](https://crates.io/crates/aw9523-embedded)
[![Documentation](https://docs.rs/aw9523-embedded/badge.svg)](https://docs.rs/aw9523-embedded)

A platform-agnostic embedded-hal driver for the **AW9523** 16-channel GPIO expander and LED driver.

## Features

- 🔌 **16 GPIO Pins** - Configurable as digital input/output
- 💡 **LED Driver Mode** - 256-step constant current control for LED dimming
- 🔧 **`no_std` Compatible** - Perfect for embedded systems
- 🎯 **Platform Agnostic** - Uses `embedded-hal` traits for maximum portability
- ⚡ **Flexible Control** - Individual pin control or bulk 16-bit operations
- 📡 **Interrupt Support** - Configurable interrupt detection per pin
- 🔄 **Dual Output Modes** - Port 0 supports open-drain or push-pull

## Hardware Overview

The AW9523 is an I2C-controlled GPIO expander with:
- 16 configurable I/O pins organized as two 8-bit ports
- Each pin supports digital I/O or constant-current LED driving
- I2C interface (default address: 0x58)
- Interrupt capability for input change detection
- Port 0 (pins 0-7): Configurable as open-drain or push-pull
- Port 1 (pins 8-15): Push-pull outputs only

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
aw9523-embedded = "0.1"
embedded-hal = "1.0"
```

## Usage Examples

### Basic GPIO Control

```rust
use aw9523_embedded::{Aw9523, OUTPUT, INPUT};

// Create device with I2C bus and default address
let mut gpio = Aw9523::new(i2c, 0x58);

// Initialize with default configuration
gpio.init().unwrap();

// Configure pin 0 as output and set it high
gpio.pin_mode(0, OUTPUT).unwrap();
gpio.digital_write(0, true).unwrap();

// Configure pin 8 as input and read its state
gpio.pin_mode(8, INPUT).unwrap();
let state = gpio.digital_read(8).unwrap();
```

### LED Dimming

```rust
use aw9523_embedded::{Aw9523, AW9523_LED_MODE};

// Create and initialize device
let mut gpio = Aw9523::new(i2c, 0x58);
gpio.init().unwrap();

// Configure pin 0 for LED mode
gpio.pin_mode(0, AW9523_LED_MODE).unwrap();

// Set LED brightness (0 = off, 255 = maximum)
gpio.analog_write(0, 128).unwrap(); // 50% brightness
gpio.analog_write(0, 255).unwrap(); // 100% brightness
```

### Bulk Operations

```rust
use aw9523_embedded::{Aw9523, AW9523_PIN_0, AW9523_PIN_3, AW9523_PIN_5};

let mut gpio = Aw9523::new(i2c, 0x58);
gpio.init().unwrap();

// Set multiple pins high at once using bitmasks
gpio.output_gpio(AW9523_PIN_0 | AW9523_PIN_3 | AW9523_PIN_5).unwrap();

// Read all 16 pins at once
let inputs = gpio.input_gpio().unwrap();
if inputs & AW9523_PIN_5 != 0 {
    // Pin 5 is high
}

// Configure multiple pins as outputs
gpio.configure_direction(AW9523_PIN_0 | AW9523_PIN_3).unwrap();

// Enable interrupts on multiple pins
gpio.interrupt_enable_gpio(AW9523_PIN_8 | AW9523_PIN_9).unwrap();
```

### Interrupt Detection

```rust
use aw9523_embedded::{Aw9523, INPUT};

let mut gpio = Aw9523::new(i2c, 0x58);
gpio.init().unwrap();

// Configure pin as input
gpio.pin_mode(10, INPUT).unwrap();

// Enable interrupt for pin 10
gpio.enable_interrupt(10, true).unwrap();

// Your code would monitor the hardware interrupt line
// and read the input state when triggered
```

## API Overview

### Pin Configuration
- `init()` - Initialize device with default configuration
- `reset()` - Software reset
- `pin_mode(pin, mode)` - Configure single pin (INPUT, OUTPUT, or AW9523_LED_MODE)

### Digital I/O
- `digital_write(pin, state)` - Set output pin high/low
- `digital_read(pin)` - Read input pin state
- `output_gpio(pins)` - Set all 16 pins at once (bit mask)
- `input_gpio()` - Read all 16 pins at once

### LED Control
- `analog_write(pin, brightness)` - Set LED brightness (0-255)
- `configure_led_mode(pins)` - Enable LED mode on multiple pins

### Interrupts
- `enable_interrupt(pin, enabled)` - Enable/disable interrupt for a pin
- `interrupt_enable_gpio(pins)` - Configure interrupts for multiple pins

### Bulk Configuration
- `configure_direction(pins)` - Set pin directions for all 16 pins

## Pin Constants

The crate provides constants for easy pin referencing:

```rust
use aw9523_embedded::{AW9523_PIN_0, AW9523_P0_0, AW9523_PORT0_ALL};

// Individual pins: AW9523_PIN_0 through AW9523_PIN_15
// Port notation: AW9523_P0_0 through AW9523_P1_7
// Port groups: AW9523_PORT0_ALL, AW9523_PORT1_ALL
// All pins: AW9523_ALL_PINS
```

## Resources

- [AW9523 Datasheet](https://cdn-shop.adafruit.com/product-files/4886/AW9523+English+Datasheet.pdf)
- [Adafruit AW9523 Breakout](https://www.adafruit.com/product/4886)
- [GitHub Repository](https://github.com/georgik/aw9523-rs)

## License

MIT

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
