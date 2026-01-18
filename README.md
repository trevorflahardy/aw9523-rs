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
use aw9523_embedded::{Aw9523, PinMode, AW9523_PIN_0, AW9523_PIN_8, HIGH, LOW};

// Create device with I2C bus and default address
let mut gpio = Aw9523::new(i2c, 0x58);

// Initialize with default configuration
gpio.init().unwrap();

// Configure pin 0 as output and set it high
gpio.pin_mode(AW9523_PIN_0, PinMode::Output).unwrap();
gpio.digital_write(AW9523_PIN_0, HIGH).unwrap();

// Configure pin 8 as input and read its state
gpio.pin_mode(AW9523_PIN_8, PinMode::Input).unwrap();
let state = gpio.digital_read(AW9523_PIN_8).unwrap();
```

### LED Dimming

```rust
use aw9523_embedded::{Aw9523, PinMode, AW9523_PIN_0};

// Create and initialize device
let mut gpio = Aw9523::new(i2c, 0x58);
gpio.init().unwrap();

// Configure pin 0 for LED mode
gpio.pin_mode(AW9523_PIN_0, PinMode::LedMode).unwrap();

// Set LED brightness (0 = off, 255 = maximum)
gpio.analog_write(AW9523_PIN_0, 128).unwrap(); // 50% brightness
gpio.analog_write(AW9523_PIN_0, 255).unwrap(); // 100% brightness
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
use aw9523_embedded::{Aw9523, PinMode, AW9523_PIN_10};

let mut gpio = Aw9523::new(i2c, 0x58);
gpio.init().unwrap();

// Configure pin as input
gpio.pin_mode(AW9523_PIN_10, PinMode::Input).unwrap();

// Enable interrupt for pin 10
gpio.enable_interrupt(AW9523_PIN_10, true).unwrap();

// Your code would monitor the hardware interrupt line
// and read the input state when triggered
```

## Async Support

The library provides full async support when the `async` feature is enabled. This allows non-blocking I2C operations using `embedded-hal-async` traits.

### Enabling Async

Add the `async` feature to your `Cargo.toml`:

```toml
[dependencies]
aw9523-embedded = { version = "0.1", features = ["async"] }
embedded-hal-async = "1.0"
```

### Async Usage Examples

#### Basic Async GPIO Control

```rust
use aw9523_embedded::{r#async::Aw9523Async, PinMode, AW9523_PIN_0, AW9523_PIN_8, HIGH};

async fn configure_gpio(i2c: impl embedded_hal_async::i2c::I2c) {
    // Create async device
    let mut gpio = Aw9523Async::new(i2c, 0x58);

    // Initialize with default configuration
    gpio.init().await.unwrap();

    // Configure pin 0 as output and set it high
    gpio.pin_mode(AW9523_PIN_0, PinMode::Output).await.unwrap();
    gpio.digital_write(AW9523_PIN_0, HIGH).await.unwrap();

    // Configure pin 8 as input and read its state
    gpio.pin_mode(AW9523_PIN_8, PinMode::Input).await.unwrap();
    let state = gpio.digital_read(AW9523_PIN_8).await.unwrap();
}
```

#### Async LED Control

```rust
use aw9523_embedded::{r#async::Aw9523Async, PinMode, AW9523_PIN_0};

async fn control_leds(i2c: impl embedded_hal_async::i2c::I2c) {
    let mut gpio = Aw9523Async::new(i2c, 0x58);
    gpio.init().await.unwrap();

    // Configure pin 0 for LED mode
    gpio.pin_mode(AW9523_PIN_0, PinMode::LedMode).await.unwrap();

    // Fade LED from off to full brightness
    for brightness in 0..=255 {
        gpio.analog_write(AW9523_PIN_0, brightness).await.unwrap();
        // Add your async delay here
    }
}
```

#### Async Bulk Operations

```rust
use aw9523_embedded::{r#async::Aw9523Async, AW9523_PIN_0, AW9523_PIN_3, AW9523_PIN_5};

async fn bulk_control(i2c: impl embedded_hal_async::i2c::I2c) {
    let mut gpio = Aw9523Async::new(i2c, 0x58);
    gpio.init().await.unwrap();

    // Set multiple pins high at once
    gpio.output_gpio(AW9523_PIN_0 | AW9523_PIN_3 | AW9523_PIN_5).await.unwrap();

    // Read all 16 pins at once
    let inputs = gpio.input_gpio().await.unwrap();
    if inputs & AW9523_PIN_5 != 0 {
        // Pin 5 is high
    }
}
```

### Async with Embassy Framework

The async implementation works seamlessly with Embassy and other async runtimes:

```rust
use embassy_executor::Spawner;
use aw9523_embedded::{r#async::Aw9523Async, AW9523_PIN_0, HIGH, LOW};

#[embassy_executor::task]
async fn gpio_task(i2c: impl embedded_hal_async::i2c::I2c) {
    let mut gpio = Aw9523Async::new(i2c, 0x58);
    gpio.init().await.unwrap();

    loop {
        // Toggle LED
        gpio.digital_write(AW9523_PIN_0, HIGH).await.unwrap();
        embassy_time::Timer::after_millis(500).await;
        gpio.digital_write(AW9523_PIN_0, LOW).await.unwrap();
        embassy_time::Timer::after_millis(500).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize your I2C peripheral
    let i2c = /* your async I2C setup */;

    spawner.spawn(gpio_task(i2c)).unwrap();
}
```

### Blocking vs Async API Comparison

| Feature | Blocking API | Async API |
|---------|-------------|-----------|
| Module | Root module | `aw9523_embedded::r#async` |
| Struct | `Aw9523` | `Aw9523Async` |
| Trait | `embedded_hal::i2c::I2c` | `embedded_hal_async::i2c::I2c` |
| Method calls | `gpio.init().unwrap()` | `gpio.init().await.unwrap()` |
| Runtime | Blocking execution | Async/await compatible |

All methods available in the blocking API have async equivalents with identical names and signatures (except for the `async` keyword and `.await`).

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

### Additional Methods (Async)
- `get_chip_id()` - Read chip ID register (should return 0x23)
- `open_drain_port0(enabled)` - Configure Port 0 as open-drain or push-pull

## Pin Constants

The crate provides constants for easy pin referencing:

```rust
use aw9523_embedded::{AW9523_PIN_0, AW9523_P0_0, AW9523_PORT0_ALL};

// Individual pins: AW9523_PIN_0 through AW9523_PIN_15
// Port notation: AW9523_P0_0 through AW9523_P1_7
// Port groups: AW9523_PORT0_ALL, AW9523_PORT1_ALL
// All pins: AW9523_ALL_PINS
```

All constants are re-exported in the async module for convenience.

## Testing

The library includes comprehensive test coverage for both blocking and async implementations:

```bash
# Run blocking tests
cargo test

# Run async tests
cargo test --features async

# Run all tests with all features
cargo test --all-features
```

## Resources

- [AW9523 Datasheet](https://cdn-shop.adafruit.com/product-files/4886/AW9523+English+Datasheet.pdf)
- [Adafruit AW9523 Breakout](https://www.adafruit.com/product/4886)
- [GitHub Repository](https://github.com/georgik/aw9523-rs)
- [embedded-hal Documentation](https://docs.rs/embedded-hal/)
- [embedded-hal-async Documentation](https://docs.rs/embedded-hal-async/)
