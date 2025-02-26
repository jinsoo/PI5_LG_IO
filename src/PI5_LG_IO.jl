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
  lg_i2c_read_device, lg_i2c_write_device,
  lg_spi_open, lg_spi_close, lg_spi_read, lg_spi_write, lg_spi_xfer,
  lg_serial_open, lg_serial_close, lg_serial_read_byte, lg_serial_write_byte,
  lg_serial_read, lg_serial_write, lg_serial_data_available,
  lg_thread_start, lg_thread_stop,
  lg_error_text, lg_timestamp, lg_time, lg_sleep, lg_version,
  lg_get_internal, lg_set_internal, lg_sbc_name,
  lg_set_work_dir, lg_get_work_dir,
  get_gpio_chips, print_all_gpio_chips_info, print_chip_info, print_line_info,
  print_all_lines_info, print_gpio_info, cleanup

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

# Load the lgpio library
const LGPIO_LIB_LOCK = ReentrantLock()

# Use a wrapper function that safely loads the library
function get_lgpio_lib()
  @static if Sys.islinux()
    lib_path = "/usr/lib/liblgpio.so"
    try
      return Libdl.dlopen(lib_path)
    catch e
      error("Could not load library $lib_path: $e. Make sure it's installed on your system.")
    end
  else
    error("lgpio is only available on Linux systems")
  end
end
const LGPIO_LIB = get_lgpio_lib()
# Function to get a pointer to a function in the library
function get_func(func_name)
  lock(LGPIO_LIB_LOCK) do
    sym = Libdl.dlsym(LGPIO_LIB, func_name)
    if sym == C_NULL
      error("Function $func_name not found in liblgpio.so")
    end
    return sym
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
function cstr_to_array(ntup::NTuple{N,Cchar}) where N
  idx = findfirst(ntup .== 0) # Find the null terminator
  if idx === nothing
    return collect(ntup)
  else
    return collect(ntup)[1:idx-1]
  end
end
function cstr_to_string(ntup::NTuple{N,Cchar}) where N
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
function lg_gpiochip_open(gpioDev::Integer)
  ccall(get_func(:lgGpiochipOpen), Cint, (Cint,), Cint(gpioDev))
end
function lg_gpiochip_close(handle::Integer)
  ccall(get_func(:lgGpiochipClose), Cint, (Cint,), Cint(handle))
end
function lg_gpio_get_chip_info(handle::Integer, chipInfo::Ptr{LGChipInfo})
  ccall(get_func(:lgGpioGetChipInfo), Cint, (Cint, Ptr{LGChipInfo}), Cint(handle), chipInfo)
end
function lg_gpio_get_line_info(handle::Integer, gpio::Integer, lineInfo::Ptr{LGLineInfo})
  ccall(get_func(:lgGpioGetLineInfo), Cint, (Cint, Cint, Ptr{LGLineInfo}), Cint(handle), Cint(gpio), lineInfo)
end
function lg_gpio_get_mode(handle::Integer, gpio::Integer)
  ccall(get_func(:lgGpioGetMode), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
end
function lg_gpio_set_user(handle::Integer, gpiouser::AbstractString)
  c_gpiouser = Base.unsafe_convert(Cstring, Base.cconvert(Cstring, gpiouser))
  GC.@preserve gpiouser begin
    ccall(get_func(:lgGpioSetUser), Cint, (Cint, Cstring), Cint(handle), c_gpiouser)
  end
end
function lg_gpio_claim_input(handle::Integer, lFlags::Integer, gpio::Integer)
  ccall(get_func(:lgGpioClaimInput), Cint, (Cint, Cint, Cint), Cint(handle), Cint(lFlags), Cint(gpio))
end
function lg_gpio_claim_output(handle::Integer, lFlags::Integer, gpio::Integer, level::Integer)
  ccall(get_func(:lgGpioClaimOutput), Cint, (Cint, Cint, Cint, Cint), Cint(handle), Cint(lFlags), Cint(gpio), Cint(level))
end
function lg_gpio_claim_alert(handle::Integer, lFlags::Integer, eFlags::Integer, gpio::Integer, nfyHandle::Integer)
  ccall(get_func(:lgGpioClaimAlert), Cint, (Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(lFlags), Cint(eFlags), Cint(gpio), Cint(nfyHandle))
end
function lg_gpio_free(handle::Integer, gpio::Integer)
  ccall(get_func(:lgGpioFree), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
end
function lg_group_claim_input(handle::Integer, lFlags::Integer, count::Integer, gpios::Vector{<:Integer})
  GC.@preserve gpios begin
    ccall(get_func(:lgGroupClaimInput), Cint, (Cint, Cint, Cint, Ptr{Cint}),
      Cint(handle), Cint(lFlags), Cint(count), pointer(convert(Vector{Cint}, gpios)))
  end
end
function lg_group_claim_output(handle::Integer, lFlags::Integer, count::Integer, gpios::Vector{<:Integer}, levels::Vector{<:Integer})
  GC.@preserve gpios levels begin
    ccall(get_func(:lgGroupClaimOutput), Cint, (Cint, Cint, Cint, Ptr{Cint}, Ptr{Cint}),
      Cint(handle), Cint(lFlags), Cint(count), pointer(convert(Vector{Cint}, gpios)),
      pointer(convert(Vector{Cint}, levels)))
  end
end
function lg_group_free(handle::Integer, gpio::Integer)
  ccall(get_func(:lgGroupFree), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
end
function lg_gpio_read(handle::Integer, gpio::Integer)
  ccall(get_func(:lgGpioRead), Cint, (Cint, Cint), Cint(handle), Cint(gpio))
end
function lg_gpio_write(handle::Integer, gpio::Integer, level::Integer)
  ccall(get_func(:lgGpioWrite), Cint, (Cint, Cint, Cint), Cint(handle), Cint(gpio), Cint(level))
end
function lg_group_read(handle::Integer, gpio::Integer, groupBits::Ref{UInt64})
  ccall(get_func(:lgGroupRead), Cint, (Cint, Cint, Ptr{UInt64}),
    Cint(handle), Cint(gpio), Base.unsafe_convert(Ptr{UInt64}, groupBits))
end
function lg_group_write(handle::Integer, gpio::Integer, groupBits::UInt64, groupMask::UInt64)
  ccall(get_func(:lgGroupWrite), Cint, (Cint, Cint, UInt64, UInt64),
    Cint(handle), Cint(gpio), groupBits, groupMask)
end
function lg_gpio_set_debounce(handle::Integer, gpio::Integer, debounce_us::Integer)
  ccall(get_func(:lgGpioSetDebounce), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(debounce_us))
end
function lg_gpio_set_watchdog(handle::Integer, gpio::Integer, watchdog_us::Integer)
  ccall(get_func(:lgGpioSetWatchdog), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(watchdog_us))
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

  return result
end

# PWM and Wave Functions
function lg_tx_pulse(handle::Integer, gpio::Integer, pulseOn::Integer, pulseOff::Integer, pulseOffset::Integer, pulseCycles::Integer)
  ccall(get_func(:lgTxPulse), Cint, (Cint, Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(pulseOn), Cint(pulseOff), Cint(pulseOffset), Cint(pulseCycles))
end
function lg_tx_pwm(handle::Integer, gpio::Integer, pwmFrequency::Real, pwmDutyCycle::Real, pwmOffset::Integer, pwmCycles::Integer)
  ccall(get_func(:lgTxPwm), Cint, (Cint, Cint, Cfloat, Cfloat, Cint, Cint),
    Cint(handle), Cint(gpio), Cfloat(pwmFrequency), Cfloat(pwmDutyCycle),
    Cint(pwmOffset), Cint(pwmCycles))
end
function lg_tx_servo(handle::Integer, gpio::Integer, pulseWidth::Integer, servoFrequency::Integer, servoOffset::Integer, servoCycles::Integer)
  ccall(get_func(:lgTxServo), Cint, (Cint, Cint, Cint, Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(pulseWidth), Cint(servoFrequency),
    Cint(servoOffset), Cint(servoCycles))
end
function lg_tx_wave(handle::Integer, gpio::Integer, count::Integer, pulses::Union{Vector{LGPulse},Ptr{LGPulse}})
  if pulses isa Vector{LGPulse}
    GC.@preserve pulses begin
      ccall(get_func(:lgTxWave), Cint, (Cint, Cint, Cint, Ptr{LGPulse}),
        Cint(handle), Cint(gpio), Cint(count), pointer(pulses))
    end
  else
    ccall(get_func(:lgTxWave), Cint, (Cint, Cint, Cint, Ptr{LGPulse}),
      Cint(handle), Cint(gpio), Cint(count), pulses)
  end
end
function lg_tx_busy(handle::Integer, gpio::Integer, kind::Integer)
  ccall(get_func(:lgTxBusy), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(kind))
end
function lg_tx_room(handle::Integer, gpio::Integer, kind::Integer)
  ccall(get_func(:lgTxRoom), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(gpio), Cint(kind))
end

# Notification Functions
function lg_notify_open()
  ccall(get_func(:lgNotifyOpen), Cint, ())
end
function lg_notify_close(handle::Integer)
  ccall(get_func(:lgNotifyClose), Cint, (Cint,), Cint(handle))
end
function lg_notify_pause(handle::Integer)
  ccall(get_func(:lgNotifyPause), Cint, (Cint,), Cint(handle))
end
function lg_notify_resume(handle::Integer)
  ccall(get_func(:lgNotifyResume), Cint, (Cint,), Cint(handle))
end

# I2C Functions
function lg_i2c_open(i2cDev::Integer, i2cAddr::Integer, i2cFlags::Integer)
  ccall(get_func(:lgI2cOpen), Cint, (Cint, Cint, Cint),
    Cint(i2cDev), Cint(i2cAddr), Cint(i2cFlags))
end
function lg_i2c_close(handle::Integer)
  ccall(get_func(:lgI2cClose), Cint, (Cint,), Cint(handle))
end
function lg_i2c_write_quick(handle::Integer, bitVal::Integer)
  ccall(get_func(:lgI2cWriteQuick), Cint, (Cint, Cint),
    Cint(handle), Cint(bitVal))
end
function lg_i2c_write_byte(handle::Integer, byteVal::Integer)
  ccall(get_func(:lgI2cWriteByte), Cint, (Cint, Cint),
    Cint(handle), Cint(byteVal))
end
function lg_i2c_read_byte(handle::Integer)
  ccall(get_func(:lgI2cReadByte), Cint, (Cint,), Cint(handle))
end
function lg_i2c_write_byte_data(handle::Integer, i2cReg::Integer, byteVal::Integer)
  ccall(get_func(:lgI2cWriteByteData), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(byteVal))
end
function lg_i2c_read_byte_data(handle::Integer, i2cReg::Integer)
  ccall(get_func(:lgI2cReadByteData), Cint, (Cint, Cint),
    Cint(handle), Cint(i2cReg))
end
function lg_i2c_write_word_data(handle::Integer, i2cReg::Integer, wordVal::Integer)
  ccall(get_func(:lgI2cWriteWordData), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(wordVal))
end
function lg_i2c_read_word_data(handle::Integer, i2cReg::Integer)
  ccall(get_func(:lgI2cReadWordData), Cint, (Cint, Cint),
    Cint(handle), Cint(i2cReg))
end
function lg_i2c_process_call(handle::Integer, i2cReg::Integer, wordVal::Integer)
  ccall(get_func(:lgI2cProcessCall), Cint, (Cint, Cint, Cint),
    Cint(handle), Cint(i2cReg), Cint(wordVal))
end
function lg_i2c_write_block_data(handle::Integer, i2cReg::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    ccall(get_func(:lgI2cWriteBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(data), Cint(count))
  end
end
function lg_i2c_read_block_data(handle::Integer, i2cReg::Integer, rxBuf::Vector{UInt8})
  GC.@preserve rxBuf begin
    ccall(get_func(:lgI2cReadBlockData), Cint, (Cint, Cint, Ptr{Cchar}),
      Cint(handle), Cint(i2cReg), pointer(rxBuf))
  end
end
function lg_i2c_block_process_call(handle::Integer, i2cReg::Integer, ioBuf::Vector{UInt8}, count::Integer)
  GC.@preserve ioBuf begin
    ccall(get_func(:lgI2cBlockProcessCall), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(ioBuf), Cint(count))
  end
end
function lg_i2c_read_i2c_block_data(handle::Integer, i2cReg::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    ccall(get_func(:lgI2cReadI2CBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(rxBuf), Cint(count))
  end
end
function lg_i2c_write_i2c_block_data(handle::Integer, i2cReg::Integer, txBuf::Union{Vector{UInt8},String}, count::Integer)
  if txBuf isa String
    data = Vector{UInt8}(txBuf)
  else
    data = txBuf
  end

  GC.@preserve data begin
    ccall(get_func(:lgI2cWriteI2CBlockData), Cint, (Cint, Cint, Ptr{Cchar}, Cint),
      Cint(handle), Cint(i2cReg), pointer(data), Cint(count))
  end
end

# SPI Functions
function lg_spi_open(spiDev::Integer, spiChan::Integer, spiBaud::Integer, spiFlags::Integer)
  ccall(get_func(:lgSpiOpen), Cint, (Cint, Cint, Cint, Cint),
    Cint(spiDev), Cint(spiChan), Cint(spiBaud), Cint(spiFlags))
end
function lg_spi_close(handle::Integer)
  ccall(get_func(:lgSpiClose), Cint, (Cint,), Cint(handle))
end
function lg_spi_read(handle::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgSpiRead), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(rxBuf), Cint(count))
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
    return result
  end
end

# Serial Functions
function lg_serial_open(serDev::AbstractString, serBaud::Integer, serFlags::Integer)
  c_serDev = Base.unsafe_convert(Cstring, Base.cconvert(Cstring, serDev))
  GC.@preserve serDev begin
    result = ccall(get_func(:lgSerialOpen), Cint, (Cstring, Cint, Cint),
      c_serDev, Cint(serBaud), Cint(serFlags))
    return result
  end
end
# 원래 포인터를 직접 받는 버전도 백워드 호환성을 위해 유지할 수 있습니다
function lg_serial_open(serDev::Ptr{Cchar}, serBaud::Integer, serFlags::Integer)
  ccall(get_func(:lgSerialOpen), Cint, (Ptr{Cchar}, Cint, Cint),
    serDev, Cint(serBaud), Cint(serFlags))
end
function lg_serial_close(handle::Integer)
  ccall(get_func(:lgSerialClose), Cint, (Cint,), Cint(handle))
end
function lg_serial_read_byte(handle::Integer)
  ccall(get_func(:lgSerialReadByte), Cint, (Cint,), Cint(handle))
end
function lg_serial_write_byte(handle::Integer, byteVal::Integer)
  ccall(get_func(:lgSerialWriteByte), Cint, (Cint, Cint), Cint(handle), Cint(byteVal))
end
function lg_serial_read(handle::Integer, rxBuf::Vector{UInt8}, count::Integer)
  GC.@preserve rxBuf begin
    result = ccall(get_func(:lgSerialRead), Cint, (Cint, Ptr{Cchar}, Cint),
      Cint(handle), pointer(rxBuf), Cint(count))
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
    return result
  end
end

# 원래 포인터를 직접 받는 버전도 유지할 수 있습니다
function lg_serial_read(handle::Integer, rxBuf::Ptr{Cchar}, count::Integer)
  ccall(get_func(:lgSerialRead), Cint, (Cint, Ptr{Cchar}, Cint),
    Cint(handle), rxBuf, Cint(count))
end
function lg_serial_write(handle::Integer, txBuf::Ptr{Cchar}, count::Integer)
  ccall(get_func(:lgSerialWrite), Cint, (Cint, Ptr{Cchar}, Cint),
    Cint(handle), txBuf, Cint(count))
end
function lg_serial_data_available(handle::Integer)
  ccall(get_func(:lgSerialDataAvailable), Cint, (Cint,), Cint(handle))
end

# Utility Functions
function lg_error_text(error::Integer)
  unsafe_string(ccall(get_func(:lguErrorText), Cstring, (Cint,), Cint(error)))
end
function lg_timestamp()
  ccall(get_func(:lguTimestamp), UInt64, ())
end
function lg_time()
  ccall(get_func(:lguTime), Cdouble, ())
end
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
  ccall(get_func(:lguSetInternal), Cint, (Cint, UInt64),
    Cint(cfgId), UInt64(cfgVal))
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
# Clean up function with improved resource management
function cleanup()
  # Ensure that we only close the library if it's open
  if LGPIO_LIB != C_NULL
    lock(LGPIO_LIB_LOCK) do
      Libdl.dlclose(LGPIO_LIB)
    end
  end
end
end # module pi5_lg_io
