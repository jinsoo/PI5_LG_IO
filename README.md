# PI5_LG_IO

A Julia module that provides bindings to the lgpio C library for Raspberry Pi 5 hardware interfaces, including GPIO, I2C, SPI, and Serial communication.

## Overview

PI5_LG_IO enables Julia programs to interact with hardware connected to a Raspberry Pi 5 through various interfaces. It wraps the lgpio C library, providing a Julia-friendly API for hardware control.

## Features

- GPIO management (input, output, alerts)
- PWM and waveform generation
- I2C communication with comprehensive transaction support
- SPI communication
- Serial port access
- Hardware notifications and callbacks
- Thread management for parallel operations
- Utility functions (timing, error handling)
- Improved error handling and debugging
- Comprehensive inline documentation

## Requirements

- Julia 1.6 or higher
- Raspberry Pi with Linux OS
- lgpio library (`liblgpio.so`) installed

## Installation

1. Install the required C library:

```bash
sudo apt-get update
sudo apt-get install liblgpio-dev
```

2. Copy liblgpio.so to /usr/lib folder

3. Add the PI5_LG_IO module to your Julia project:

```julia
# You can add this package from a local path or GitHub repository
using Pkg
Pkg.add(path="https://github.com/jinsoo/PI5_LG_IO")

# Or import directly if the module is in your Julia load path
using PI5_LG_IO
```

## Raspberry Pi Configuration

To enable SPI interfaces, add the following to your `/boot/firmware/config.txt` file:

```
# Enable basic SPI
dtparam=spi=on
# SPI0 configuration (default pins: MOSI=10, MISO=9, SCLK=11, CS0=8, CS1=7)
dtoverlay=spi0-2cs
# SPI1 configuration (default pins: MOSI=20, MISO=19, SCLK=21, CS0=18, CS1=17)
dtoverlay=spi1-2cs
```

After updating the configuration, reboot your Raspberry Pi for the changes to take effect:

```bash
sudo reboot
```

## Usage Examples

### GPIO Example

```julia
using PI5_LG_IO

# Open a GPIO chip
chip = 0  # gpiochip0 on Raspberry Pi
handle = lg_gpiochip_open(chip)

if handle < 0
    println("Error opening GPIO chip: $(lg_error_text(handle))")
    exit(1)
end

# Configure GPIO pin as output
gpio = 17  # BCM pin 17
result = lg_gpio_claim_output(handle, 0, gpio, LG_LOW)

if result < 0
    println("Error claiming GPIO: $(lg_error_text(result))")
    lg_gpiochip_close(handle)
    exit(1)
end

# Blink LED
for i in 1:10
    println("LED on")
    lg_gpio_write(handle, gpio, LG_HIGH)
    lg_sleep(0.5)
    
    println("LED off")
    lg_gpio_write(handle, gpio, LG_LOW)
    lg_sleep(0.5)
end

# Clean up
lg_gpio_free(handle, gpio)
lg_gpiochip_close(handle)
```

### I2C Example

```julia
using PI5_LG_IO

# Open I2C device
i2c_dev = 1  # i2c-1 on Raspberry Pi
i2c_addr = 0x68  # Example device address (RTC DS3231)
handle = lg_i2c_open(i2c_dev, i2c_addr, 0)

if handle < 0
    println("Error opening I2C device: $(lg_error_text(handle))")
    exit(1)
end

# Read time from RTC
seconds_reg = 0x00
value = lg_i2c_read_byte_data(handle, seconds_reg)

if value >= 0
    println("Seconds: $(value & 0x7F)")  # Mask out clock halt bit
else
    println("Error reading from I2C device: $(lg_error_text(value))")
end

# Clean up
lg_i2c_close(handle)
```

### Advanced I2C with Multiple Transactions

```julia
using PI5_LG_IO

# Open I2C device
i2c_dev = 1
i2c_addr = 0x50  # Example EEPROM address
handle = lg_i2c_open(i2c_dev, i2c_addr, 0)

if handle < 0
    println("Error opening I2C device: $(lg_error_text(handle))")
    exit(1)
end

# Create I2C messages for a write-then-read operation
# First message: Write memory address to read from
addr_buf = [0x00, 0x10]  # Memory address 0x0010
write_msg = LGI2CMsg(i2c_addr, 0, length(addr_buf), addr_buf)

# Second message: Read 8 bytes from that address
read_buf = Vector{UInt8}(undef, 8)
read_msg = LGI2CMsg(i2c_addr, 1, length(read_buf), read_buf)  # Flag 1 = read operation

# Perform the combined transaction
messages = [write_msg, read_msg]
result = lg_i2c_segments(handle, messages)

if result >= 0
    println("Read $(result) bytes: $(read_buf)")
else
    println("Error in I2C transaction: $(lg_error_text(result))")
end

# Clean up
lg_i2c_close(handle)
```

### SPI Example

```julia
using PI5_LG_IO

# Open SPI device
spi_dev = 0  # SPI0 on Raspberry Pi
spi_chan = 0  # CE0
spi_baud = 1_000_000  # 1MHz
spi_flags = 0  # No special flags
handle = lg_spi_open(spi_dev, spi_chan, spi_baud, spi_flags)

if handle < 0
    println("Error opening SPI device: $(lg_error_text(handle))")
    exit(1)
end

# Send and receive data
tx_buf = [0x01, 0x02, 0x03, 0x04]
rx_buf = Vector{UInt8}(undef, length(tx_buf))

result = lg_spi_xfer(handle, tx_buf, rx_buf, length(tx_buf))

if result >= 0
    println("Received data: $rx_buf")
else
    println("Error in SPI transfer: $(lg_error_text(result))")
end

# Clean up
lg_spi_close(handle)
```

### PWM Example

```julia
using PI5_LG_IO

# Open a GPIO chip
chip = 0
handle = lg_gpiochip_open(chip)

if handle < 0
    println("Error opening GPIO chip: $(lg_error_text(handle))")
    exit(1)
end

# Configure GPIO pin as output
gpio = 18  # BCM pin 18
result = lg_gpio_claim_output(handle, 0, gpio, LG_LOW)

if result < 0
    println("Error claiming GPIO: $(lg_error_text(result))")
    lg_gpiochip_close(handle)
    exit(1)
end

# Generate PWM (e.g., for controlling LED brightness or motor speed)
frequency = 1000  # 1kHz
duty_cycle = 50.0  # 50%
cycles = 0  # Run continuously

result = lg_tx_pwm(handle, gpio, frequency, duty_cycle, 0, cycles)

if result < 0
    println("Error starting PWM: $(lg_error_text(result))")
else
    println("PWM running. Press Ctrl+C to stop.")
    try
        while true
            lg_sleep(1)
        end
    catch e
        if isa(e, InterruptException)
            println("Stopping PWM...")
        else
            rethrow(e)
        end
    end
end

# Clean up
lg_gpio_free(handle, gpio)
lg_gpiochip_close(handle)
```

### Thread Management Example

```julia
using PI5_LG_IO

# Define a function to run in a separate thread
function background_task(data)
    println("Background thread started with data: $data")
    for i in 1:10
        println("Thread iteration $i")
        lg_sleep(1)
    end
    println("Background thread completed")
end

# Start a thread with the function and some data
thread_data = "Sample data"
thread_id = lg_thread_start(background_task, thread_data)

if thread_id < 0
    println("Error starting thread: $(lg_error_text(thread_id))")
    exit(1)
end

println("Main thread continues while background thread runs...")

# Do some work in the main thread
for i in 1:5
    println("Main thread iteration $i")
    lg_sleep(0.5)
end

# Wait for background thread to finish
println("Waiting for background thread to complete...")
lg_sleep(6)

# Stop the thread (if it's still running)
lg_thread_stop(thread_id)

println("Done.")
```

## Utility Functions

The module provides several utility functions to help with hardware interaction:

- `print_gpio_info()`: Print information about available GPIO chips and lines
- `print_chip_info()`: Print information about a specific GPIO chip
- `print_line_info()`: Print information about a specific GPIO line
- `print_all_lines_info()`: Print information about all lines on a chip
- `print_all_gpio_chips_info()`: Print information about all available GPIO chips
- `get_gpio_chips()`: Get a list of available GPIO chips
- `is_liblgpio_usable()`: Check if the lgpio library is available and usable
- `check_package_configuration()`: Check the configuration of the package and library
- `lg_error_text()`: Get a human-readable description for an error code

Example:

```julia
using PI5_LG_IO

# Print information about all GPIO chips
print_all_gpio_chips_info()

# Print information about a specific chip
chip = 0
handle = lg_gpiochip_open(chip)
print_chip_info(handle)
lg_gpiochip_close(handle)

# Check if the library is properly loaded
if is_liblgpio_usable()
    println("lgpio library is available and usable")
else
    println("lgpio library is not properly installed or not available")
end

# Get detailed configuration information
config = check_package_configuration()
for (key, value) in config
    println("$key: $value")
end
```

## Error Handling

The module includes improved error handling with debug logging. When a function encounters an error, it will log the error message using Julia's logging system.

By default, only warnings and errors are shown. To see debug messages:

```julia
using Logging
global_logger(ConsoleLogger(stderr, Logging.Debug))
```

You can also check for errors in your code:

```julia
result = lg_gpio_claim_output(handle, 0, gpio, LG_LOW)
if result < 0
    println("Error: $(lg_error_text(result))")
end
```

## Resource Management

The module automatically manages resources and provides functions for proper cleanup:

```julia
# Close any open handles
lg_gpio_free(handle, gpio)
lg_gpiochip_close(handle)

# The module automatically handles cleanup when Julia exits
# but you can also call it explicitly
cleanup()
```

The improved version includes better callback registry management and proper cleanup of thread resources.

## Constants

The module defines several constants for use with the functions:

- `LG_LOW`, `LG_HIGH`: GPIO levels
- `LG_RISING_EDGE`, `LG_FALLING_EDGE`, `LG_BOTH_EDGES`: Edge detection for alerts
- `LG_SET_ACTIVE_LOW`, `LG_SET_OPEN_DRAIN`, etc.: GPIO flags

## Advanced Usage

### GPIO Callbacks

```julia
using PI5_LG_IO

# Open a GPIO chip
chip = 0
handle = lg_gpiochip_open(chip)

# Configure GPIO pin for input with alert
gpio = 17
notify_handle = lg_notify_open()
result = lg_gpio_claim_alert(handle, 0, LG_RISING_EDGE, gpio, notify_handle)

# Set up callback function
function gpio_callback(count, alerts, userdata)
    for i in 1:count
        alert = alerts[i]
        println("GPIO $(alert.report.gpio) changed to $(alert.report.level) at $(alert.report.timestamp)")
    end
end

# Register callback
lg_gpio_set_alerts_func(handle, gpio, gpio_callback, nothing)

# Wait for events
println("Waiting for GPIO events (press Ctrl+C to exit)...")
try
    while true
        lg_sleep(1)
    end
catch e
    if isa(e, InterruptException)
        println("Exiting...")
    else
        rethrow(e)
    end
finally
    # Clean up
    lg_gpio_free(handle, gpio)
    lg_notify_close(notify_handle)
    lg_gpiochip_close(handle)
end
```

### Checking Library Availability

```julia
using PI5_LG_IO

# Check if the library is usable
if !is_liblgpio_usable()
    println("liblgpio is not available or not properly installed")
    exit(1)
end

# Check package configuration
config = check_package_configuration()
println("Library loaded: $(config["library_loaded"])")
println("Julia version: $(config["julia_version"])")

# If there was an error loading the library
if !config["library_loaded"]
    println("Error: $(config["library_error"])")
end
```

## License

This project is licensed under the MIT License:

```
MIT License

Copyright (c) 2024 Jin-Soo Kim, Expikx company

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.