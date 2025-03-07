module PI5_LG_IO

using Libdl
using CEnum
using Printf

export lg_gpiochip_open, lg_gpiochip_close, lg_gpio_get_chip_info, lg_gpio_get_line_info,
  lg_gpio_get_mode, lg_gpio_set_user,
  lg_gpio_claim_input, lg_gpio_claim_output, lg_gpio_claim_alert, lg_gpio_free,
  lg_group_claim_input, lg_group_claim_output, lg_group_free,
  lg_gpio_read, lg_gpio_write, lg_group_read, lg_group_write,
  lg_gpio_set_alerts_func, lg_gpio_set_samples_func,
  lg_gpio_set_debounce, lg_gpio_set_watchdog,
  lg_tx_pulse, lg_tx_pwm, lg_tx_servo, lg_tx_wave, lg_tx_busy, lg_tx_room,
  lg_notify_open, lg_notify_close, lg_notify_pause, lg_notify_resume,
  lg_i2c_open, lg_i2c_close, lg_i2c_write_quick, lg_i2c_write_byte, lg_i2c_read_byte,
  lg_i2c_write_byte_data, lg_i2c_read_byte_data, lg_i2c_write_word_data, lg_i2c_read_word_data,
  lg_i2c_process_call, lg_i2c_write_block_data, lg_i2c_read_block_data, lg_i2c_block_process_call,
  lg_i2c_read_i2c_block_data, lg_i2c_write_i2c_block_data, lg_i2c_segments, lg_i2c_zip,
  lg_spi_open, lg_spi_close, lg_spi_read, lg_spi_write, lg_spi_xfer,
  lg_serial_open, lg_serial_close, lg_serial_read_byte, lg_serial_write_byte,
  lg_serial_read, lg_serial_write, lg_serial_data_available,
  lg_thread_start, lg_thread_stop,
  lg_error_text, lg_timestamp, lg_time, lg_sleep, lg_version,
  lg_get_internal, lg_set_internal, lg_sbc_name,
  lg_set_work_dir, lg_get_work_dir,
  get_gpio_chips, print_all_gpio_chips_info, print_chip_info, print_line_info,
  print_all_lines_info, print_gpio_info, cleanup, is_liblgpio_usable, check_package_configuration, load_lgpio_library

# Define constants first for better organization
# GPIO Constants
const LG_LOW = 0
const LG_HIGH = 1
const LG_TIMEOUT = 2

const LG_TX_PWM = 0
const LG_TX_WAVE = 1

const LG_RISING_EDGE = 1
const LG_FALLING_EDGE = 2
const LG_BOTH_EDGES = 3

const LG_SET_ACTIVE_LOW = 4
const LG_SET_OPEN_DRAIN = 8
const LG_SET_OPEN_SOURCE = 16
const LG_SET_PULL_UP = 32
const LG_SET_PULL_DOWN = 64
const LG_SET_PULL_NONE = 128

# Library state management
mutable struct LibraryState
  handle::Ptr{Nothing}
  loaded::Bool
  error::String
end

const LGPIO_STATE = LibraryState(C_NULL, false, "")
const LGPIO_LIB_LOCK = ReentrantLock()

"""
  load_lgpio_library()

Loads the lgpio library and returns a handle to it.
The function ensures that the library is loaded only once.

Returns:
- A pointer to the loaded library, or C_NULL if loading failed
"""
function load_lgpio_library()
  # Only attempt to load once
  if LGPIO_STATE.loaded
    return LGPIO_STATE.handle
  end

  lock(LGPIO_LIB_LOCK) do
    # Check again after acquiring the lock
    if LGPIO_STATE.loaded
      return LGPIO_STATE.handle
    end

    @static if Sys.islinux()
      lib_path = "/usr/lib/liblgpio.so"
      try
        handle = Libdl.dlopen(lib_path; throw_error=false)
        if handle == C_NULL
          LGPIO_STATE.error = "Failed to load $lib_path (null handle)"
        else
          LGPIO_STATE.handle = handle
        end
      catch e
        LGPIO_STATE.error = "Error loading $lib_path: $e"
      end
    else
      LGPIO_STATE.error = "lgpio is only available on Linux systems"
    end

    LGPIO_STATE.loaded = true
    return LGPIO_STATE.handle
  end
end

"""
  get_func(func_name)

Retrieves a function pointer from the loaded lgpio library.

Arguments:
- func_name: Name of the function to retrieve

Returns:
- A pointer to the requested function

Throws:
- Error if library is not loaded or function is not found
"""
function get_func(func_name)
  handle = load_lgpio_library()
  if handle == C_NULL
    error("LGPIO library not loaded: $(LGPIO_STATE.error)")
  end

  lock(LGPIO_LIB_LOCK) do
    try
      sym = Libdl.dlsym_e(handle, func_name)
      if sym == C_NULL
        error("Function $func_name not found in liblgpio.so")
      end
      return sym
    catch e
      error("Error accessing function $func_name: $e")
    end
  end
end

"""
  check_package_configuration()

Checks the configuration of the package and the lgpio library.

Returns:
- A dictionary with configuration information
"""
function check_package_configuration()
  results = Dict()

  # Check if we can load the library
  handle = load_lgpio_library()
  results["library_loaded"] = handle != C_NULL
  results["library_error"] = LGPIO_STATE.error

  # Check Julia environment
  results["julia_version"] = string(VERSION)
  results["libc_path"] = Base.BinaryPlatforms.libc_lib

  # Check if some basic functions are available
  if handle != C_NULL
    test_functions = [:lguVersion, :lgGpiochipOpen, :lgGpioRead]
    for func in test_functions
      try
        sym = Libdl.dlsym_e(handle, func)
        results["func_$(func)"] = sym != C_NULL
      catch
        results["func_$(func)"] = false
      end
    end
  end

  return results
end

function __init__()
  # Initialize the library when module is loaded
  handle = load_lgpio_library()
  if handle == C_NULL
    @warn "Failed to load lgpio library: $(LGPIO_STATE.error)"
  end

  # Make sure to cleanup when Julia exits
  atexit() do
    # Clean up library resources
    if LGPIO_STATE.handle != C_NULL
      try
        Libdl.dlclose(LGPIO_STATE.handle)
        LGPIO_STATE.handle = C_NULL
        LGPIO_STATE.loaded = false
      catch
        # Ignore errors during cleanup
      end
    end

    # Clean up callback registry
    lock(callback_registry_lock) do
      empty!(callback_registry)
    end
  end
end

# Enums and Structs
@cenum LGError begin
  LG_OKAY = 0
  LG_INIT_FAILED = -1
  LG_BAD_MICROS = -2
  LG_BAD_PATHNAME = -3
  LG_NO_HANDLE = -4
  LG_BAD_HANDLE = -5
  LG_BAD_SOCKET_PORT = -6
  LG_NOT_PERMITTED = -7
  LG_SOME_PERMITTED = -8
  LG_BAD_SCRIPT = -9
  LG_BAD_TX_TYPE = -10
  LG_GPIO_IN_USE = -11
  # More error codes can be added as needed
end

# Structs
struct LGChipInfo
  lines::UInt32
  name::NTuple{32,Cchar}
  label::NTuple{32,Cchar}
end
struct LGLineInfo
  offset::UInt32
  lFlags::UInt32
  name::NTuple{32,Cchar}
  user::NTuple{32,Cchar}
end
struct LGGpioReport
  timestamp::UInt64
  chip::UInt8
  gpio::UInt8
  level::UInt8
  flags::UInt8
end
struct LGGpioAlert
  report::LGGpioReport
  nfyHandle::Cint
end
struct LGPulse
  bits::UInt64
  mask::UInt64
  delay::Int64
end
struct LGI2CMsg
  addr::UInt16
  flags::UInt16
  len::UInt16
  buf::Ptr{UInt8}
end

# Helper functions for string conversion
function cstr_to_array(ntup::NTuple{N,Cchar}) where {N}
  idx = findfirst(ntup .== 0) # Find the null terminator
  if idx === nothing
    return collect(ntup)
  else
    return collect(ntup)[1:idx-1]
  end
end
function cstr_to_string(ntup::NTuple{N,Cchar}) where {N}
  return String(cstr_to_array(ntup))
end

# Function to safely pad strings for C interop
function safe_pad_str(s::AbstractString, len::Int)
  result = fill(Cchar(0), len)
  for (i, c) in enumerate(s)
    if i <= len
      result[i] = Cchar(c)
    else
      break
    end
  end
  return ntuple(i -> result[i], len)
end

# Constructor for LGChipInfo
function LGChipInfo(lines::Integer=0; name::AbstractString="", label::AbstractString="")
  name_tuple = safe_pad_str(name, 32)
  label_tuple = safe_pad_str(label, 32)
  LGChipInfo(UInt32(lines), name_tuple, label_tuple)
end
# Constructor for LGLineInfo
function LGLineInfo(offset::Integer=0, lFlags::Integer=0; name::AbstractString="", user::AbstractString="")
  name_tuple = safe_pad_str(name, 32)
  user_tuple = safe_pad_str(user, 32)
  LGLineInfo(UInt32(offset), UInt32(lFlags), name_tuple, user_tuple)
end
# Constructor for LGGpioReport
function LGGpioReport(timestamp::Integer=0, chip::Integer=0, gpio::Integer=0, level::Integer=0, flags::Integer=0)
  LGGpioReport(UInt64(timestamp), UInt8(chip), UInt8(gpio), UInt8(level), UInt8(flags))
end
# Constructor for LGGpioAlert
function LGGpioAlert(report::LGGpioReport=LGGpioReport(), nfyHandle::Integer=0)
  LGGpioAlert(report, Cint(nfyHandle))
end
# Constructor for LGPulse
function LGPulse(bits::Integer=0, mask::Integer=0, delay::Integer=0)
  LGPulse(UInt64(bits), UInt64(mask), Int64(delay))
end
# Constructor for LGI2CMsg
function LGI2CMsg(addr::Integer=0, flags::Integer=0, len::Integer=0, buf::Union{Ptr{UInt8},Vector{UInt8}}=Ptr{UInt8}(0))
  if buf isa Vector{UInt8}
    buf_ptr = pointer(buf)
  else
    buf_ptr = buf
  end
  LGI2CMsg(UInt16(addr), UInt16(flags), UInt16(len), buf_ptr)
end

# GPIO Chip utility functions
"""
  get_gpio_chips()

Gets a list of available GPIO chips.

Returns:
- A list of tuples (chip_number, handle)
"""
function get_gpio_chips()
  chips = Tuple{Int,Int}[]
  chip_num = 0

  while true
    handle = lg_gpiochip_open(chip_num)
    if handle < 0
      # No more chips found
      break
    end

    push!(chips, (chip_num, handle))
    chip_num += 1
  end

  return chips
end
# Function to print information for all available GPIO chips
function print_all_gpio_chips_info()
  chips = get_gpio_chips()

  if isempty(chips)
    println("No GPIO chips found.")
    return
  end

  for (chip_num, handle) in chips
    println("GPIO Chip $chip_num:")
    print_chip_info(handle)
    print_all_lines_info(handle)
    println("-"^40)  # Separator line
    lg_gpiochip_close(handle)
  end
end
# Function to pretty print chip information
function print_chip_info(handle::Integer)
  # Initialize with default values
  chip_info = Ref(LGChipInfo())
  result = lg_gpio_get_chip_info(handle, Base.unsafe_convert(Ptr{LGChipInfo}, chip_info))

  if result < 0
    println("Error getting chip info: $(lg_error_text(result))")
    return
  end

  println("Chip Information:")
  println("  Lines:     $(chip_info[].lines)")
  println("  Name:      $(cstr_to_string(chip_info[].name))")
  println("  Label:     $(cstr_to_string(chip_info[].label))")
end
# Function to pretty print line information
function print_line_info(handle::Integer, gpio::Integer)
  # Initialize with default values
  line_info = Ref(LGLineInfo())
  result = lg_gpio_get_line_info(handle, gpio, Base.unsafe_convert(Ptr{LGLineInfo}, line_info))

  if result < 0
    println("Error getting line info for GPIO $gpio: $(lg_error_text(result))")
    return
  end

  println("Line Information for GPIO $gpio:")
  println("  Offset:    $(line_info[].offset)")
  println("  Flags:     0x$(@sprintf("%08X", line_info[].lFlags))")
  println("  Name:      $(cstr_to_string(line_info[].name))")
  println("  User:      $(cstr_to_string(line_info[].user))")
end
# Function to print all line information for a chip
function print_all_lines_info(handle::Integer)
  chip_info = Ref(LGChipInfo())
  result = lg_gpio_get_chip_info(handle, Base.unsafe_convert(Ptr{LGChipInfo}, chip_info))

  if result < 0
    println("Error getting chip info: $(lg_error_text(result))")
    return
  end

  println("Information for all lines:")
  for gpio in 0:(chip_info[].lines-1)
    print_line_info(handle, Cint(gpio))
  end
end
# Function to print all chip and line information
function print_gpio_info(gpiochip::Integer=0)
  handle = lg_gpiochip_open(gpiochip)
  if handle < 0
    println("Error opening gpiochip$gpiochip: $(lg_error_text(handle))")
    return
  end

  print_chip_info(handle)
  print_all_lines_info(handle)

  lg_gpiochip_close(handle)
end

# GPIO Functions
"""
  lg_gpiochip_open(gpioDev)

Opens a GPIO chip.

Arguments:
- gpioDev: GPIO device number

Returns:
- Handle to the GPIO chip or negative error code
"""
function lg_gpiochip_open(gpioDev::Integer)
  result = ccall(get_func(:lgGpiochipOpen), Cint, (Cint,), Cint(gpioDev))
  if result < 0
    @debug "Error opening GPIO chip $gpioDev: $(lg_error_text(result))"
  end
  return result
end

"""
  lg_gpiochip_close(handle)

Closes a GPIO chip.

Arguments:
- handle: Handle to the GPIO chip

Returns:
- 0 on success or negative error code
"""
function lg_gpiochip_close(handle::Integer)
  result = ccall(get_func(:lgGpiochipClose), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error closing GPIO chip: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_get_chip_info(handle::Integer, chipInfo::Ptr{LGChipInfo})
  result = ccall(get_func(:lgGpioGetChipInfo), Cint, (Cint, Ptr{LGChipInfo}), Cint(handle), chipInfo)
  if result < 0
    @debug "Error getting chip info: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_get_line_info(handle::Integer, gpio::Integer, lineInfo::Ptr{LGLineInfo})
  result = ccall(get_func(:lgGpioGetLineInfo), Cint, (Cint, Cint, Ptr{LGLineInfo}), Cint(handle), Cint(gpio), lineInfo)
  if result < 0
    @debug "Error getting line info for GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_get_mode(handle::Integer, gpio::Integer)
  result = ccall(get_func(:lgGpioGetMode), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
  if result < 0
    @debug "Error getting mode for GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_set_user(handle::Integer, gpiouser::AbstractString)
  c_gpiouser = Base.unsafe_convert(Cstring, Base.cconvert(Cstring, gpiouser))
  GC.@preserve gpiouser begin
    result = ccall(get_func(:lgGpioSetUser), Cint, (Cint, Cstring), Cint(handle), c_gpiouser)
    if result < 0
      @debug "Error setting GPIO user: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_gpio_claim_input(handle::Integer, lFlags::Integer, gpio::Integer)
  result = ccall(get_func(:lgGpioClaimInput), Cint, (Cint, Cint, Cint), Cint(handle), Cint(lFlags), Cint(gpio))
  if result < 0
    @debug "Error claiming GPIO $gpio as input: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_claim_output(handle::Integer, lFlags::Integer, gpio::Integer, level::Integer)
  result = ccall(get_func(:lgGpioClaimOutput), Cint, (Cint, Cint, Cint, Cint), Cint(handle), Cint(lFlags), Cint(gpio), Cint(level))
  if result < 0
    @debug "Error claiming GPIO $gpio as output: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_claim_alert(handle::Integer, lFlags::Integer, eFlags::Integer, gpio::Integer, nfyHandle::Integer)
  result = ccall(get_func(:lgGpioClaimAlert), Cint, (Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(lFlags), Cint(eFlags), Cint(gpio), Cint(nfyHandle))
  if result < 0
    @debug "Error claiming GPIO $gpio for alerts: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_free(handle::Integer, gpio::Integer)
  result = ccall(get_func(:lgGpioFree), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
  if result < 0
    @debug "Error freeing GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_group_claim_input(handle::Integer, lFlags::Integer, count::Integer, gpios::Vector{<:Integer})
  GC.@preserve gpios begin
    result = ccall(get_func(:lgGroupClaimInput), Cint, (Cint, Cint, Cint, Ptr{Cint}),
      Cint(handle), Cint(lFlags), Cint(count), pointer(convert(Vector{Cint}, gpios)))
    if result < 0
      @debug "Error claiming GPIO group as input: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_group_claim_output(handle::Integer, lFlags::Integer, count::Integer, gpios::Vector{<:Integer}, levels::Vector{<:Integer})
  GC.@preserve gpios levels begin
    result = ccall(get_func(:lgGroupClaimOutput), Cint, (Cint, Cint, Cint, Ptr{Cint}, Ptr{Cint}),
      Cint(handle), Cint(lFlags), Cint(count), pointer(convert(Vector{Cint}, gpios)),
      pointer(convert(Vector{Cint}, levels)))
    if result < 0
      @debug "Error claiming GPIO group as output: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_group_free(handle::Integer, gpio::Integer)
  result = ccall(get_func(:lgGroupFree), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
  if result < 0
    @debug "Error freeing GPIO group: $(lg_error_text(result))"
  end
  return result
end

"""
  lg_gpio_read(handle, gpio)

Reads the level of a GPIO pin.

Arguments:
- handle: Handle to the GPIO chip
- gpio: GPIO pin number

Returns:
- 0 (low), 1 (high), or negative error code
"""
function lg_gpio_read(handle::Integer, gpio::Integer)
  result = ccall(get_func(:lgGpioRead), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
  if result < 0
    @debug "Error reading GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

"""
  lg_gpio_write(handle, gpio, level)

Writes a level to a GPIO pin.

Arguments:
- handle: Handle to the GPIO chip
- gpio: GPIO pin number
- level: Level to write (0 for low, 1 for high)

Returns:
- 0 on success or negative error code
"""
function lg_gpio_write(handle::Integer, gpio::Integer, level::Integer)
  result = ccall(get_func(:lgGpioWrite), Cint, (Cint, Cint, Cint), Cint(handle), Cint(gpio), Cint(level))
  if result < 0
    @debug "Error writing to GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_group_read(handle::Integer, gpio::Integer, groupBits::Ref{UInt64})
  result = ccall(get_func(:lgGroupRead), Cint, (Cint, Cint, Ptr{UInt64}),
    Cint(handle), Cint(gpio), Base.unsafe_convert(Ptr{UInt64}, groupBits))
  if result < 0
    @debug "Error reading GPIO group: $(lg_error_text(result))"
  end
  return result
end

function lg_group_write(handle::Integer, gpio::Integer, groupBits::UInt64, groupMask::UInt64)
  result = ccall(get_func(:lgGroupWrite), Cint, (Cint, Cint, UInt64, UInt64),
    Cint(handle), Cint(gpio), groupBits, groupMask)
  if result < 0
    @debug "Error writing to GPIO group: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_set_debounce(handle::Integer, gpio::Integer, debounce_us::Integer)
  result = ccall(get_func(:lgGpioSetDebounce), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(debounce_us))
  if result < 0
    @debug "Error setting debounce for GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_gpio_set_watchdog(handle::Integer, gpio::Integer, watchdog_us::Integer)
  result = ccall(get_func(:lgGpioSetWatchdog), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(watchdog_us))
  if result < 0
    @debug "Error setting watchdog for GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

# Define the callback function type and global storage
const LGGpioAlertsFunc = Ptr{Cvoid}
# We'll use a more robust callback mechanism with weak references to prevent memory leaks
mutable struct CallbackData
  handle::Cint
  gpio::Cint
  func::Function
  userdata::Any
end
# Global dictionary to store callback data
const callback_registry = Dict{UInt,CallbackData}()
const callback_registry_lock = ReentrantLock()
# C callback function that will be passed to the C library
function global_c_callback(e::Cint, evt::Ptr{LGGpioAlert}, data::Ptr{Cvoid})::Cvoid
  key = reinterpret(UInt, data)

  lock(callback_registry_lock) do
    if haskey(callback_registry, key)
      cb_data = callback_registry[key]

      # Convert C array to Julia array
      alerts = unsafe_wrap(Array, evt, e)

      # Call the Julia callback function
      cb_data.func(e, alerts, cb_data.userdata)
    end
  end

  return nothing
end
# The C-compatible callback reference
const c_callback_ref = Ref{Ptr{Cvoid}}(C_NULL)
# Initialize the C callback only once
function get_c_callback()
  if c_callback_ref[] == C_NULL
    c_callback_ref[] = @cfunction(global_c_callback, Cvoid, (Cint, Ptr{LGGpioAlert}, Ptr{Cvoid}))
  end
  return c_callback_ref[]
end
# Register a callback function
function register_callback(handle::Integer, gpio::Integer, func::Function, userdata::Any)
  # Create a unique key for this callback
  key = UInt(hash((handle, gpio, objectid(func))))
  data_ptr = reinterpret(Ptr{Cvoid}, key)

  cb_data = CallbackData(Cint(handle), Cint(gpio), func, userdata)

  lock(callback_registry_lock) do
    callback_registry[key] = cb_data
  end

  return data_ptr
end
# Unregister a callback function
function unregister_callback(handle::Integer, gpio::Integer)
  to_remove = UInt[]

  lock(callback_registry_lock) do
    for (key, data) in callback_registry
      if data.handle == handle && data.gpio == gpio
        push!(to_remove, key)
      end
    end

    for key in to_remove
      delete!(callback_registry, key)
    end
  end
end
# Julia function to set up the alert callback
function lg_gpio_set_alerts_func(handle::Integer, gpio::Integer, cbf::Function, userdata::Any)
  # First unregister any existing callback for this GPIO
  unregister_callback(handle, gpio)

  # Register the new callback
  data_ptr = register_callback(handle, gpio, cbf, userdata)

  # Get the C-compatible callback
  c_cbf = get_c_callback()

  # Call the C function
  result = ccall(
    get_func(:lgGpioSetAlertsFunc),
    Cint,
    (Cint, Cint, LGGpioAlertsFunc, Ptr{Cvoid}),
    Cint(handle), Cint(gpio), c_cbf, data_ptr
  )

  if result < 0
    @debug "Error setting alerts function for GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end
# Function to set a samples callback (for all GPIO)
function lg_gpio_set_samples_func(cbf::Function, userdata::Any)
  # Use a special key for the samples callback
  key = UInt(hash(("samples_callback", objectid(cbf))))
  data_ptr = reinterpret(Ptr{Cvoid}, key)

  cb_data = CallbackData(Cint(-1), Cint(-1), cbf, userdata)

  lock(callback_registry_lock) do
    callback_registry[key] = cb_data
  end

  # Get the C-compatible callback
  c_cbf = get_c_callback()

  # Call the C function
  result = ccall(
    get_func(:lgGpioSetSamplesFunc),
    Cint,
    (LGGpioAlertsFunc, Ptr{Cvoid}),
    c_cbf, data_ptr
  )

  if result < 0
    @debug "Error setting samples function: $(lg_error_text(result))"
  end
  return result
end

# PWM and Wave Functions
function lg_tx_pulse(handle::Integer, gpio::Integer, pulseOn::Integer, pulseOff::Integer, pulseOffset::Integer, pulseCycles::Integer)
  result = ccall(get_func(:lgTxPulse), Cint, (Cint, Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(pulseOn), Cint(pulseOff), Cint(pulseOffset), Cint(pulseCycles))
  if result < 0
    @debug "Error transmitting pulse on GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

"""
  lg_tx_pwm(handle, gpio, pwmFrequency, pwmDutyCycle, pwmOffset, pwmCycles)

Starts PWM on a GPIO pin.

Arguments:
- handle: Handle to the GPIO chip
- gpio: GPIO pin number
- pwmFrequency: PWM frequency in Hz
- pwmDutyCycle: PWM duty cycle (0-100)
- pwmOffset: Offset in microseconds
- pwmCycles: Number of cycles (0 for continuous)

Returns:
- 0 on success or negative error code
"""
function lg_tx_pwm(handle::Integer, gpio::Integer, pwmFrequency::Real, pwmDutyCycle::Real, pwmOffset::Integer, pwmCycles::Integer)
  result = ccall(get_func(:lgTxPwm), Cint, (Cint, Cint, Cfloat, Cfloat, Cint, Cint),
    Cint(handle), Cint(gpio), Cfloat(pwmFrequency), Cfloat(pwmDutyCycle),
    Cint(pwmOffset), Cint(pwmCycles))
  if result < 0
    @debug "Error transmitting PWM on GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_tx_servo(handle::Integer, gpio::Integer, pulseWidth::Integer, servoFrequency::Integer, servoOffset::Integer, servoCycles::Integer)
  result = ccall(get_func(:lgTxServo), Cint, (Cint, Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(pulseWidth), Cint(servoFrequency),
    Cint(servoOffset), Cint(servoCycles))
  if result < 0
    @debug "Error transmitting servo pulses on GPIO $gpio: $(lg_error_text(result))"
  end
  return result
end

function lg_tx_wave(handle::Integer, gpio::Integer, count::Integer, pulses::Union{Vector{LGPulse},Ptr{LGPulse}})
  if pulses isa Vector{LGPulse}
    GC.@preserve pulses begin
      result = ccall(get_func(:lgTxWave), Cint, (Cint, Cint, Cint, Ptr{LGPulse}),
        Cint(handle), Cint(gpio), Cint(count), pointer(pulses))
      if result < 0
        @debug "Error transmitting wave on GPIO $gpio: $(lg_error_text(result))"
      end
      return result
    end
  else
    result = ccall(get_func(:lgTxWave), Cint, (Cint, Cint, Cint, Ptr{LGPulse}),
      Cint(handle), Cint(gpio), Cint(count), pulses)
    if result < 0
      @debug "Error transmitting wave on GPIO $gpio: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_tx_busy(handle::Integer, gpio::Integer, kind::Integer)
  result = ccall(get_func(:lgTxBusy), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(kind))
  return result
end

function lg_tx_room(handle::Integer, gpio::Integer, kind::Integer)
  result = ccall(get_func(:lgTxRoom), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(kind))
  return result
end

# Notification Functions
function lg_notify_open()
  result = ccall(get_func(:lgNotifyOpen), Cint, ())
  if result < 0
    @debug "Error opening notification: $(lg_error_text(result))"
  end
  return result
end

function lg_notify_close(handle::Integer)
  result = ccall(get_func(:lgNotifyClose), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error closing notification: $(lg_error_text(result))"
  end
  return result
end

function lg_notify_pause(handle::Integer)
  result = ccall(get_func(:lgNotifyPause), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error pausing notification: $(lg_error_text(result))"
  end
  return result
end

function lg_notify_resume(handle::Integer)
  result = ccall(get_func(:lgNotifyResume), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error resuming notification: $(lg_error_text(result))"
  end
  return result
end

# I2C Functions
"""
  lg_i2c_open(i2cDev, i2cAddr, i2cFlags)

Opens a connection to an I2C device.

Arguments:
- i2cDev: I2C device number
- i2cAddr: I2C device address
- i2cFlags: I2C flags

Returns:
- Handle to the I2C device or negative error code
"""
function lg_i2c_open(i2cDev::Integer, i2cAddr::Integer, i2cFlags::Integer)
  result = ccall(get_func(:lgI2cOpen), Cint, (Cint, Cint, Cint),
    Cint(i2cDev), Cint(i2cAddr), Cint(i2cFlags))
  if result < 0
    @debug "Error opening I2C device $i2cDev at address $i2cAddr: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_close(handle::Integer)
  result = ccall(get_func(:lgI2cClose), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error closing I2C device: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_write_quick(handle::Integer, bitVal::Integer)
  result = ccall(get_func(:lgI2cWriteQuick), Cint, (Cint, Cint),
    Cint(handle), Cint(bitVal))
  if result < 0
    @debug "Error in I2C write quick: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_write_byte(handle::Integer, byteVal::Integer)
  result = ccall(get_func(:lgI2cWriteByte), Cint, (Cint, Cint),
    Cint(handle), Cint(byteVal))
  if result < 0
    @debug "Error in I2C write byte: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_read_byte(handle::Integer)
  result = ccall(get_func(:lgI2cReadByte), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error in I2C read byte: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_write_byte_data(handle::Integer, i2cReg::Integer, byteVal::Integer)
  result = ccall(get_func(:lgI2cWriteByteData), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(byteVal))
  if result < 0
    @debug "Error in I2C write byte data to register $i2cReg: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_read_byte_data(handle::Integer, i2cReg::Integer)
  result = ccall(get_func(:lgI2cReadByteData), Cint, (Cint, Cint),
    Cint(handle), Cint(i2cReg))
  if result < 0
    @debug "Error in I2C read byte data from register $i2cReg: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_write_word_data(handle::Integer, i2cReg::Integer, wordVal::Integer)
  result = ccall(get_func(:lgI2cWriteWordData), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(wordVal))
  if result < 0
    @debug "Error in I2C write word data to register $i2cReg: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_read_word_data(handle::Integer, i2cReg::Integer)
  result = ccall(get_func(:lgI2cReadWordData), Cint, (Cint, Cint),
    Cint(handle), Cint(i2cReg))
  if result < 0
    @debug "Error in I2C read word data from register $i2cReg: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_process_call(handle::Integer, i2cReg::Integer, wordVal::Integer)
  result = ccall(get_func(:lgI2cProcessCall), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(wordVal))
  if result < 0
    @debug "Error in I2C process call to register $i2cReg: $(lg_error_text(result))"
  end
  return result
end

function lg_i2c_write_block_data(handle::Integer, i2cReg::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    result = ccall(get_func(:lgI2cWriteBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(data), Cint(count))
    if result < 0
      @debug "Error in I2C write block data to register $i2cReg: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_i2c_read_block_data(handle::Integer, i2cReg::Integer, rxBuf::Vector{UInt8})
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgI2cReadBlockData), Cint, (Cint, Cint, Ptr{Cchar}),
      Cint(handle), Cint(i2cReg), pointer(rxBuf))
    if result < 0
      @debug "Error in I2C read block data from register $i2cReg: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_i2c_block_process_call(handle::Integer, i2cReg::Integer, ioBuf::Vector{UInt8}, count::Integer)
  GC.@preserve ioBuf begin
    result = ccall(get_func(:lgI2cBlockProcessCall), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(ioBuf), Cint(count))
    if result < 0
      @debug "Error in I2C block process call to register $i2cReg: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_i2c_read_i2c_block_data(handle::Integer, i2cReg::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgI2cReadI2CBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(rxBuf), Cint(count))
    if result < 0
      @debug "Error in I2C read I2C block data from register $i2cReg: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_i2c_write_i2c_block_data(handle::Integer, i2cReg::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    result = ccall(get_func(:lgI2cWriteI2CBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(data), Cint(count))
    if result < 0
      @debug "Error in I2C write I2C block data to register $i2cReg: $(lg_error_text(result))"
    end
    return result
  end
end

# Implement the missing I2C functions
"""
  lg_i2c_segments(handle, segs)

Performs multiple I2C transactions.

Arguments:
- handle: Handle to the I2C device
- segs: Array of I2C segments

Returns:
- Number of bytes transferred or negative error code
"""
function lg_i2c_segments(handle::Integer, segs::Vector{LGI2CMsg})
  GC.@preserve segs begin
    result = ccall(get_func(:lgI2cSegments), Cint, (Cint, Ptr{LGI2CMsg}, Cint),
      Cint(handle), pointer(segs), Cint(length(segs)))
    if result < 0
      @debug "Error in I2C segments: $(lg_error_text(result))"
    end
    return result
  end
end

"""
  lg_i2c_zip(handle, inBuf, inLen, outBuf, outLen)

Executes a sequence of I2C operations.

Arguments:
- handle: Handle to the I2C device
- inBuf: Input buffer with commands
- inLen: Length of input buffer
- outBuf: Output buffer for results
- outLen: Maximum length of output buffer

Returns:
- Number of bytes placed in outBuf or negative error code
"""
function lg_i2c_zip(handle::Integer, inBuf::Union{Vector{UInt8},String}, inLen::Integer,
  outBuf::Vector{UInt8}, outLen::Integer)
  if inBuf isa String
    in_data = Vector{UInt8}(inBuf)
  else
    in_data = inBuf
  end

  GC.@preserve in_data outBuf begin
    result = ccall(get_func(:lgI2cZip), Cint,
      (Cint, Ptr{Cchar}, Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(in_data), Cint(inLen), pointer(outBuf), Cint(outLen))
    if result < 0
      @debug "Error in I2C zip: $(lg_error_text(result))"
    end
    return result
  end
end

# SPI Functions
"""
  lg_spi_open(spiDev, spiChan, spiBaud, spiFlags)

Opens a connection to an SPI device.

Arguments:
- spiDev: SPI device number
- spiChan: SPI channel
- spiBaud: SPI baud rate
- spiFlags: SPI flags

Returns:
- Handle to the SPI device or negative error code
"""
function lg_spi_open(spiDev::Integer, spiChan::Integer, spiBaud::Integer, spiFlags::Integer)
  result = ccall(get_func(:lgSpiOpen), Cint, (Cint, Cint, Cint, Cint),
    Cint(spiDev), Cint(spiChan), Cint(spiBaud), Cint(spiFlags))
  if result < 0
    @debug "Error opening SPI device $spiDev channel $spiChan: $(lg_error_text(result))"
  end
  return result
end

function lg_spi_close(handle::Integer)
  result = ccall(get_func(:lgSpiClose), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error closing SPI device: $(lg_error_text(result))"
  end
  return result
end

function lg_spi_read(handle::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgSpiRead), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(rxBuf), Cint(count))
    if result < 0
      @debug "Error in SPI read: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_spi_write(handle::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    result = ccall(get_func(:lgSpiWrite), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(data), Cint(count))
    if result < 0
      @debug "Error in SPI write: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_spi_xfer(handle::Integer, txBuf::Union{Vector{UInt8},String}, rxBuf::Vector{UInt8}, count::Integer)
  if txBuf isa String
    tx_data = Vector{UInt8}(txBuf)
  else
    tx_data = txBuf
  end

  GC.@preserve tx_data rxBuf begin
    result = ccall(get_func(:lgSpiXfer), Cint, (Cint, Ptr{Cchar}, Ptr{Cchar}, Cint),
      Cint(handle), pointer(tx_data), pointer(rxBuf), Cint(count))
    if result < 0
      @debug "Error in SPI transfer: $(lg_error_text(result))"
    end
    return result
  end
end

# Serial Functions
function lg_serial_open(serDev::AbstractString, serBaud::Integer, serFlags::Integer)
  c_serDev = Base.unsafe_convert(Cstring, Base.cconvert(Cstring, serDev))
  GC.@preserve serDev begin
    result = ccall(get_func(:lgSerialOpen), Cint, (Cstring, Cint, Cint),
      c_serDev, Cint(serBaud), Cint(serFlags))
    if result < 0
      @debug "Error opening serial device $serDev: $(lg_error_text(result))"
    end
    return result
  end
end

# Keep the pointer version for backward compatibility
function lg_serial_open(serDev::Ptr{Cchar}, serBaud::Integer, serFlags::Integer)
  result = ccall(get_func(:lgSerialOpen), Cint, (Ptr{Cchar}, Cint, Cint),
    serDev, Cint(serBaud), Cint(serFlags))
  if result < 0
    @debug "Error opening serial device: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_close(handle::Integer)
  result = ccall(get_func(:lgSerialClose), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error closing serial device: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_read_byte(handle::Integer)
  result = ccall(get_func(:lgSerialReadByte), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error in serial read byte: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_write_byte(handle::Integer, byteVal::Integer)
  result = ccall(get_func(:lgSerialWriteByte), Cint, (Cint, Cint), Cint(handle), Cint(byteVal))
  if result < 0
    @debug "Error in serial write byte: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_read(handle::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgSerialRead), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(rxBuf), Cint(count))
    if result < 0
      @debug "Error in serial read: $(lg_error_text(result))"
    end
    return result
  end
end

function lg_serial_write(handle::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    result = ccall(get_func(:lgSerialWrite), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(data), Cint(count))
    if result < 0
      @debug "Error in serial write: $(lg_error_text(result))"
    end
    return result
  end
end

# Keep pointer versions for backward compatibility
function lg_serial_read(handle::Integer, rxBuf::Ptr{Cchar}, count::Integer)
  result = ccall(get_func(:lgSerialRead), Cint, (Cint, Ptr{Cchar}, Cint),
    Cint(handle), rxBuf, Cint(count))
  if result < 0
    @debug "Error in serial read: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_write(handle::Integer, txBuf::Ptr{Cchar}, count::Integer)
  result = ccall(get_func(:lgSerialWrite), Cint, (Cint, Ptr{Cchar}, Cint),
    Cint(handle), txBuf, Cint(count))
  if result < 0
    @debug "Error in serial write: $(lg_error_text(result))"
  end
  return result
end

function lg_serial_data_available(handle::Integer)
  result = ccall(get_func(:lgSerialDataAvailable), Cint, (Cint,), Cint(handle))
  if result < 0
    @debug "Error checking serial data available: $(lg_error_text(result))"
  end
  return result
end

# Implement the missing thread functions
"""
  lg_thread_start(func, userdata)

Starts a new thread.

Arguments:
- func: Function to run in the new thread
- userdata: User data to pass to the function

Returns:
- Thread ID or negative error code
"""
function lg_thread_start(func::Function, userdata::Any)
  # We'll need a similar mechanism to callbacks for thread functions
  key = UInt(hash(("thread", objectid(func))))
  data_ptr = reinterpret(Ptr{Cvoid}, key)

  # Store the function and userdata
  lock(callback_registry_lock) do
    callback_registry[key] = CallbackData(Cint(-1), Cint(-1), func, userdata)
  end

  # Julia function that will be called by C
  thread_func_wrapper = @cfunction(
    (data::Ptr{Cvoid}) -> begin
      k = reinterpret(UInt, data)
      lock(callback_registry_lock) do
        if haskey(callback_registry, k)
          cb_data = callback_registry[k]
          cb_data.func(cb_data.userdata)
        end
      end
      return Ptr{Cvoid}(0)
    end,
    Ptr{Cvoid}, (Ptr{Cvoid},)
  )

  result = ccall(
    get_func(:lgThreadStart),
    Cint,
    (Ptr{Cvoid}, Ptr{Cvoid}),
    thread_func_wrapper, data_ptr
  )

  if result < 0
    @debug "Error starting thread: $(lg_error_text(result))"
    # Clean up on error
    lock(callback_registry_lock) do
      delete!(callback_registry, key)
    end
  end

  return result
end

"""
  lg_thread_stop(threadId)

Stops a thread.

Arguments:
- threadId: Thread ID to stop

Returns:
- 0 on success or negative error code
"""
function lg_thread_stop(threadId::Integer)
  result = ccall(get_func(:lgThreadStop), Cint, (Cint,), Cint(threadId))
  if result < 0
    @debug "Error stopping thread $threadId: $(lg_error_text(result))"
  end
  return result
end

# Utility Functions
"""
  lg_error_text(error)

Gets a human-readable error message for an error code.

Arguments:
- error: Error code

Returns:
- Error message string
"""
function lg_error_text(error::Integer)
  unsafe_string(ccall(get_func(:lguErrorText), Cstring, (Cint,), Cint(error)))
end

function lg_timestamp()
  ccall(get_func(:lguTimestamp), UInt64, ())
end

function lg_time()
  ccall(get_func(:lguTime), Cdouble, ())
end

"""
  lg_sleep(sleepSecs)

Sleeps for the specified number of seconds.

Arguments:
- sleepSecs: Number of seconds to sleep
"""
function lg_sleep(sleepSecs::Real)
  ccall(get_func(:lguSleep), Cvoid, (Cdouble,), Cdouble(sleepSecs))
end

function lg_version()
  ccall(get_func(:lguVersion), Cint, ())
end

function lg_sbc_name(max_chars::Integer=1024)
  buffer = Vector{UInt8}(undef, max_chars)
  GC.@preserve buffer begin
    result = ccall(get_func(:lguSbcName), Cint, (Ptr{Cchar}, Cint),
      pointer(buffer), Cint(max_chars))
    if result >= 0
      return String(buffer[1:result])
    else
      return lg_error_text(result)
    end
  end
end

function lg_get_internal(cfgId::Integer)
  cfgVal = Ref{UInt64}(0)
  result = ccall(get_func(:lguGetInternal), Cint, (Cint, Ptr{UInt64}),
    Cint(cfgId), Base.unsafe_convert(Ptr{UInt64}, cfgVal))
  return result, cfgVal[]
end

function lg_set_internal(cfgId::Integer, cfgVal::Integer)
  result = ccall(get_func(:lguSetInternal), Cint, (Cint, UInt64),
    Cint(cfgId), UInt64(cfgVal))
  if result < 0
    @debug "Error setting internal configuration $cfgId: $(lg_error_text(result))"
  end
  return result
end

function lg_set_work_dir(dirPath::AbstractString)
  c_dirPath = Base.unsafe_convert(Cstring, Base.cconvert(Cstring, dirPath))
  GC.@preserve dirPath begin
    ccall(get_func(:lguSetWorkDir), Cvoid, (Cstring,), c_dirPath)
  end
end

function lg_get_work_dir()
  ptr = ccall(get_func(:lguGetWorkDir), Cstring, ())
  return unsafe_string(ptr)
end

"""
  is_liblgpio_usable()

Checks if the lgpio library is available and usable.

Returns:
- true if the library is usable, false otherwise
"""
function is_liblgpio_usable()
  try
    handle = load_lgpio_library()
    if handle == C_NULL
      return false
    end
    sym = Libdl.dlsym_e(handle, :lguVersion)
    return sym != C_NULL
  catch
    return false
  end
end

"""
  cleanup()

Cleans up resources used by the module.
"""
function cleanup()
  # Ensure that we only close the library if it's open
  if LGPIO_STATE.handle != C_NULL
    lock(LGPIO_LIB_LOCK) do
      try
        Libdl.dlclose(LGPIO_STATE.handle)
        LGPIO_STATE.handle = C_NULL
        LGPIO_STATE.loaded = false
      catch e
        @warn "Error during cleanup: $e"
      end
    end
  end

  # Clean up callback registry
  lock(callback_registry_lock) do
    empty!(callback_registry)
  end
end
end # module PI5_LG_IO