# Bobby Car Controller

Here the code for the high-level Bobby Car Controller is located.

The Controller has the following features:

- It reads Wii Nunchuk input
- It transforms throttle/brake input into hoverboard UART commands
- It sends speed commands to the motor driver board

## PlatformIO

This folder is now a PlatformIO project.

- `platformio.ini`: Platform and board environments
- `src/main.cpp`: migrated controller code (from Arduino IDE tabs)
- `include/nunchuk.h`: local Nunchuk library header

## Build and Upload

From this folder:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

If your Nano uses the new bootloader, select:

```bash
pio run -e nanoatmega328new
```

## Notes

- `compile_commands.json` is an auto-generated C/C++ compilation database used by tools like `clangd` and `clang-tidy`.
- It is machine/path-specific build output and is intentionally ignored in git.
