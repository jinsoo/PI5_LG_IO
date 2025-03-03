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
    print_all_lines_info, print_gpio_info, cleanup, is_liblgpio_usable, check_package_configuration, load_lgpio_library,
    get_error_description, print_error_description, print_all_errors,
    LG_OKAY, LG_INIT_FAILED, LG_BAD_MICROS, LG_BAD_PATHNAME  # exporting necessary constants 


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

  # Replace the existing library loading code with this
  mutable struct LibraryState
    handle::Ptr{Nothing}
    loaded::Bool
    error::String
  end

  const LGPIO_STATE = LibraryState(C_NULL, false, "")
  const LGPIO_LIB_LOCK = ReentrantLock()

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

  # Replace the existing get_func with this safer version
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
    load_lgpio_library()

    # Make sure to cleanup when Julia exits
    atexit() do
      if LGPIO_STATE.handle != C_NULL
        try
          Libdl.dlclose(LGPIO_STATE.handle)
          LGPIO_STATE.handle = C_NULL
          LGPIO_STATE.loaded = false
        catch
          # Ignore errors during cleanup
        end
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
  function is_liblgpio_usable()
    try
      sym = Libdl.dlsym_e(LGPIO_LIB, :lguVersion)
      return sym != C_NULL
    catch
      return false
    end
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

  # 에러 코드 상수 정의
  const LG_OKAY = 0   # No error
  const LG_INIT_FAILED = -1   # initialisation failed
  const LG_BAD_MICROS = -2   # micros not 0-999999
  const LG_BAD_PATHNAME = -3   # can not open pathname
  const LG_NO_HANDLE = -4   # no handle available
  const LG_BAD_HANDLE = -5   # unknown handle
  const LG_BAD_SOCKET_PORT = -6   # socket port not 1024-32000
  const LG_NOT_PERMITTED = -7   # GPIO operation not permitted
  const LG_SOME_PERMITTED = -8   # one or more GPIO not permitted
  const LG_BAD_SCRIPT = -9   # invalid script
  const LG_BAD_TX_TYPE = -10  # bad tx type for GPIO and group
  const LG_GPIO_IN_USE = -11  # GPIO already in use
  const LG_BAD_PARAM_NUM = -12  # script parameter id not 0-9
  const LG_DUP_TAG = -13  # script has duplicate tag
  const LG_TOO_MANY_TAGS = -14  # script has too many tags
  const LG_BAD_SCRIPT_CMD = -15  # illegal script command
  const LG_BAD_VAR_NUM = -16  # script variable id not 0-149
  const LG_NO_SCRIPT_ROOM = -17  # no more room for scripts
  const LG_NO_MEMORY = -18  # can not allocate temporary memory
  const LG_SOCK_READ_FAILED = -19  # socket read failed
  const LG_SOCK_WRIT_FAILED = -20  # socket write failed
  const LG_TOO_MANY_PARAM = -21  # too many script parameters (> 10)
  const LG_SCRIPT_NOT_READY = -22  # script initialising
  const LG_BAD_TAG = -23  # script has unresolved tag
  const LG_BAD_MICS_DELAY = -24  # bad MICS delay (too large)
  const LG_BAD_MILS_DELAY = -25  # bad MILS delay (too large)
  const LG_I2C_OPEN_FAILED = -26  # can not open I2C device
  const LG_SERIAL_OPEN_FAILED = -27  # can not open serial device
  const LG_SPI_OPEN_FAILED = -28  # can not open SPI device
  const LG_BAD_I2C_BUS = -29  # bad I2C bus
  const LG_BAD_I2C_ADDR = -30  # bad I2C address
  const LG_BAD_SPI_CHANNEL = -31  # bad SPI channel
  const LG_BAD_I2C_FLAGS = -32  # bad I2C open flags
  const LG_BAD_SPI_FLAGS = -33  # bad SPI open flags
  const LG_BAD_SERIAL_FLAGS = -34  # bad serial open flags
  const LG_BAD_SPI_SPEED = -35  # bad SPI speed
  const LG_BAD_SERIAL_DEVICE = -36  # bad serial device name
  const LG_BAD_SERIAL_SPEED = -37  # bad serial baud rate
  const LG_BAD_FILE_PARAM = -38  # bad file parameter
  const LG_BAD_I2C_PARAM = -39  # bad I2C parameter
  const LG_BAD_SERIAL_PARAM = -40  # bad serial parameter
  const LG_I2C_WRITE_FAILED = -41  # i2c write failed
  const LG_I2C_READ_FAILED = -42  # i2c read failed
  const LG_BAD_SPI_COUNT = -43  # bad SPI count
  const LG_SERIAL_WRITE_FAILED = -44  # ser write failed
  const LG_SERIAL_READ_FAILED = -45  # ser read failed
  const LG_SERIAL_READ_NO_DATA = -46  # ser read no data available
  const LG_UNKNOWN_COMMAND = -47  # unknown command
  const LG_SPI_XFER_FAILED = -48  # spi xfer/read/write failed
  const LG_BAD_POINTER = -49  # bad (NULL) pointer
  const LG_MSG_TOOBIG = -50  # socket/pipe message too big
  const LG_BAD_MALLOC_MODE = -51  # bad memory allocation mode
  const LG_TOO_MANY_SEGS = -52  # too many I2C transaction segments
  const LG_BAD_I2C_SEG = -53  # an I2C transaction segment failed
  const LG_BAD_SMBUS_CMD = -54  # SMBus command not supported by driver
  const LG_BAD_I2C_WLEN = -55  # bad I2C write length
  const LG_BAD_I2C_RLEN = -56  # bad I2C read length
  const LG_BAD_I2C_CMD = -57  # bad I2C command
  const LG_FILE_OPEN_FAILED = -58  # file open failed
  const LG_BAD_FILE_MODE = -59  # bad file mode
  const LG_BAD_FILE_FLAG = -60  # bad file flag
  const LG_BAD_FILE_READ = -61  # bad file read
  const LG_BAD_FILE_WRITE = -62  # bad file write
  const LG_FILE_NOT_ROPEN = -63  # file not open for read
  const LG_FILE_NOT_WOPEN = -64  # file not open for write
  const LG_BAD_FILE_SEEK = -65  # bad file seek
  const LG_NO_FILE_MATCH = -66  # no files match pattern
  const LG_NO_FILE_ACCESS = -67  # no permission to access file
  const LG_FILE_IS_A_DIR = -68  # file is a directory
  const LG_BAD_SHELL_STATUS = -69  # bad shell return status
  const LG_BAD_SCRIPT_NAME = -70  # bad script name
  const LG_CMD_INTERRUPTED = -71  # Python socket command interrupted
  const LG_BAD_EVENT_REQUEST = -72  # bad event request
  const LG_BAD_GPIO_NUMBER = -73  # bad GPIO number
  const LG_BAD_GROUP_SIZE = -74  # bad group size
  const LG_BAD_LINEINFO_IOCTL = -75  # bad lineinfo IOCTL
  const LG_BAD_READ = -76  # bad GPIO read
  const LG_BAD_WRITE = -77  # bad GPIO write
  const LG_CANNOT_OPEN_CHIP = -78  # can not open gpiochip
  const LG_GPIO_BUSY = -79  # GPIO busy
  const LG_GPIO_NOT_ALLOCATED = -80  # GPIO not allocated
  const LG_NOT_A_GPIOCHIP = -81  # not a gpiochip
  const LG_NOT_ENOUGH_MEMORY = -82  # not enough memory
  const LG_POLL_FAILED = -83  # GPIO poll failed
  const LG_TOO_MANY_GPIOS = -84  # too many GPIO
  const LG_UNEGPECTED_ERROR = -85  # unexpected error
  const LG_BAD_PWM_MICROS = -86  # bad PWM micros
  const LG_NOT_GROUP_LEADER = -87  # GPIO not the group leader
  const LG_SPI_IOCTL_FAILED = -88  # SPI iOCTL failed
  const LG_BAD_GPIOCHIP = -89  # bad gpiochip
  const LG_BAD_CHIPINFO_IOCTL = -90  # bad chipinfo IOCTL
  const LG_BAD_CONFIG_FILE = -91  # bad configuration file
  const LG_BAD_CONFIG_VALUE = -92  # bad configuration value
  const LG_NO_PERMISSIONS = -93  # no permission to perform action
  const LG_BAD_USERNAME = -94  # bad user name
  const LG_BAD_SECRET = -95  # bad secret for user
  const LG_TX_QUEUE_FULL = -96  # TX queue full
  const LG_BAD_CONFIG_ID = -97  # bad configuration id
  const LG_BAD_DEBOUNCE_MICS = -98  # bad debounce microseconds
  const LG_BAD_WATCHDOG_MICS = -99  # bad watchdog microseconds
  const LG_BAD_SERVO_FREQ = -100 # bad servo frequency
  const LG_BAD_SERVO_WIDTH = -101 # bad servo pulsewidth
  const LG_BAD_PWM_FREQ = -102 # bad PWM frequency
  const LG_BAD_PWM_DUTY = -103 # bad PWM dutycycle
  const LG_GPIO_NOT_AN_OUTPUT = -104 # GPIO not set as an output
  const LG_INVALID_GROUP_ALERT = -105 # can not set a group to alert

  # Dictionary mapping between error codes and descriptions
  const ERROR_DESCRIPTIONS = Dict{Int,String}(
    LG_OKAY => "No error",
    LG_INIT_FAILED => "initialisation failed",
    LG_BAD_MICROS => "micros not 0-999999",
    LG_BAD_PATHNAME => "can not open pathname",
    LG_NO_HANDLE => "no handle available",
    LG_BAD_HANDLE => "unknown handle",
    LG_BAD_SOCKET_PORT => "socket port not 1024-32000",
    LG_NOT_PERMITTED => "GPIO operation not permitted",
    LG_SOME_PERMITTED => "one or more GPIO not permitted",
    LG_BAD_SCRIPT => "invalid script",
    LG_BAD_TX_TYPE => "bad tx type for GPIO and group",
    LG_GPIO_IN_USE => "GPIO already in use",
    LG_BAD_PARAM_NUM => "script parameter id not 0-9",
    LG_DUP_TAG => "script has duplicate tag",
    LG_TOO_MANY_TAGS => "script has too many tags",
    LG_BAD_SCRIPT_CMD => "illegal script command",
    LG_BAD_VAR_NUM => "script variable id not 0-149",
    LG_NO_SCRIPT_ROOM => "no more room for scripts",
    LG_NO_MEMORY => "can not allocate temporary memory",
    LG_SOCK_READ_FAILED => "socket read failed",
    LG_SOCK_WRIT_FAILED => "socket write failed",
    LG_TOO_MANY_PARAM => "too many script parameters (> 10)",
    LG_SCRIPT_NOT_READY => "script initialising",
    LG_BAD_TAG => "script has unresolved tag",
    LG_BAD_MICS_DELAY => "bad MICS delay (too large)",
    LG_BAD_MILS_DELAY => "bad MILS delay (too large)",
    LG_I2C_OPEN_FAILED => "can not open I2C device",
    LG_SERIAL_OPEN_FAILED => "can not open serial device",
    LG_SPI_OPEN_FAILED => "can not open SPI device",
    LG_BAD_I2C_BUS => "bad I2C bus",
    LG_BAD_I2C_ADDR => "bad I2C address",
    LG_BAD_SPI_CHANNEL => "bad SPI channel",
    LG_BAD_I2C_FLAGS => "bad I2C open flags",
    LG_BAD_SPI_FLAGS => "bad SPI open flags",
    LG_BAD_SERIAL_FLAGS => "bad serial open flags",
    LG_BAD_SPI_SPEED => "bad SPI speed",
    LG_BAD_SERIAL_DEVICE => "bad serial device name",
    LG_BAD_SERIAL_SPEED => "bad serial baud rate",
    LG_BAD_FILE_PARAM => "bad file parameter",
    LG_BAD_I2C_PARAM => "bad I2C parameter",
    LG_BAD_SERIAL_PARAM => "bad serial parameter",
    LG_I2C_WRITE_FAILED => "i2c write failed",
    LG_I2C_READ_FAILED => "i2c read failed",
    LG_BAD_SPI_COUNT => "bad SPI count",
    LG_SERIAL_WRITE_FAILED => "ser write failed",
    LG_SERIAL_READ_FAILED => "ser read failed",
    LG_SERIAL_READ_NO_DATA => "ser read no data available",
    LG_UNKNOWN_COMMAND => "unknown command",
    LG_SPI_XFER_FAILED => "spi xfer/read/write failed",
    LG_BAD_POINTER => "bad (NULL) pointer",
    LG_MSG_TOOBIG => "socket/pipe message too big",
    LG_BAD_MALLOC_MODE => "bad memory allocation mode",
    LG_TOO_MANY_SEGS => "too many I2C transaction segments",
    LG_BAD_I2C_SEG => "an I2C transaction segment failed",
    LG_BAD_SMBUS_CMD => "SMBus command not supported by driver",
    LG_BAD_I2C_WLEN => "bad I2C write length",
    LG_BAD_I2C_RLEN => "bad I2C read length",
    LG_BAD_I2C_CMD => "bad I2C command",
    LG_FILE_OPEN_FAILED => "file open failed",
    LG_BAD_FILE_MODE => "bad file mode",
    LG_BAD_FILE_FLAG => "bad file flag",
    LG_BAD_FILE_READ => "bad file read",
    LG_BAD_FILE_WRITE => "bad file write",
    LG_FILE_NOT_ROPEN => "file not open for read",
    LG_FILE_NOT_WOPEN => "file not open for write",
    LG_BAD_FILE_SEEK => "bad file seek",
    LG_NO_FILE_MATCH => "no files match pattern",
    LG_NO_FILE_ACCESS => "no permission to access file",
    LG_FILE_IS_A_DIR => "file is a directory",
    LG_BAD_SHELL_STATUS => "bad shell return status",
    LG_BAD_SCRIPT_NAME => "bad script name",
    LG_CMD_INTERRUPTED => "Python socket command interrupted",
    LG_BAD_EVENT_REQUEST => "bad event request",
    LG_BAD_GPIO_NUMBER => "bad GPIO number",
    LG_BAD_GROUP_SIZE => "bad group size",
    LG_BAD_LINEINFO_IOCTL => "bad lineinfo IOCTL",
    LG_BAD_READ => "bad GPIO read",
    LG_BAD_WRITE => "bad GPIO write",
    LG_CANNOT_OPEN_CHIP => "can not open gpiochip",
    LG_GPIO_BUSY => "GPIO busy",
    LG_GPIO_NOT_ALLOCATED => "GPIO not allocated",
    LG_NOT_A_GPIOCHIP => "not a gpiochip",
    LG_NOT_ENOUGH_MEMORY => "not enough memory",
    LG_POLL_FAILED => "GPIO poll failed",
    LG_TOO_MANY_GPIOS => "too many GPIO",
    LG_UNEGPECTED_ERROR => "unexpected error",
    LG_BAD_PWM_MICROS => "bad PWM micros",
    LG_NOT_GROUP_LEADER => "GPIO not the group leader",
    LG_SPI_IOCTL_FAILED => "SPI iOCTL failed",
    LG_BAD_GPIOCHIP => "bad gpiochip",
    LG_BAD_CHIPINFO_IOCTL => "bad chipinfo IOCTL",
    LG_BAD_CONFIG_FILE => "bad configuration file",
    LG_BAD_CONFIG_VALUE => "bad configuration value",
    LG_NO_PERMISSIONS => "no permission to perform action",
    LG_BAD_USERNAME => "bad user name",
    LG_BAD_SECRET => "bad secret for user",
    LG_TX_QUEUE_FULL => "TX queue full",
    LG_BAD_CONFIG_ID => "bad configuration id",
    LG_BAD_DEBOUNCE_MICS => "bad debounce microseconds",
    LG_BAD_WATCHDOG_MICS => "bad watchdog microseconds",
    LG_BAD_SERVO_FREQ => "bad servo frequency",
    LG_BAD_SERVO_WIDTH => "bad servo pulsewidth",
    LG_BAD_PWM_FREQ => "bad PWM frequency",
    LG_BAD_PWM_DUTY => "bad PWM dutycycle",
    LG_GPIO_NOT_AN_OUTPUT => "GPIO not set as an output",
    LG_INVALID_GROUP_ALERT => "can not set a group to alert"
  )

  """
      get_error_description(error_no::Int)

  It takes an error number and returns the corresponding error description. If there is no error number, it returns "Unknown error code".
  """
  function get_error_description(error_no::Int)
    if haskey(ERROR_DESCRIPTIONS, error_no)
      return "Error $(error_no): $(ERROR_DESCRIPTIONS[error_no])"
    else
      return "Unknown error code: $(error_no)"
    end
  end

  """
      print_error_description(error_no::Int)

  It takes an error number and prints out the corresponding error description.
  """
  function print_error_description(error_no::Int)
    println(get_error_description(error_no))
  end

  # a function printing all error codes and descriptions
  function print_all_errors()
    println("library error code list:")
    println("========================")

    sorted_codes = sort(collect(keys(ERROR_DESCRIPTIONS)))

    for code in sorted_codes
      println("$(code): $(ERROR_DESCRIPTIONS[code])")
    end
  end
end # module pi5_lg_io
