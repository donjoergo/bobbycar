# Repository Guidelines

## Project Structure & Module Organization
- `Bobby_Car_Controller/` contains the active firmware project (PlatformIO).
- `Bobby_Car_Controller/src/main.cpp` is the main controller logic (Nunchuk input, drive mode, UART commands).
- `Bobby_Car_Controller/include/nunchuk.h` holds the local Nunchuk helper library.
- `test/` contains hardware-oriented Arduino sketches for manual validation (`Bobby_Car_Test`, `Nunchuk_Test`, etc.).
- `schematics/` stores wiring references; `docs/pictures/` stores project images used in documentation.

## Build, Test, and Development Commands
Run commands from `Bobby_Car_Controller/`:

```bash
pio run                         # Build default env (nanoatmega328)
pio run -e nanoatmega328new     # Build for Nano new bootloader
pio run -t upload               # Flash firmware to connected board
pio device monitor -b 115200    # Open serial monitor
```

Use a full build before opening a PR to catch compile-time issues.

## Coding Style & Naming Conventions
- Language: Arduino C++ (AVR/PlatformIO).
- Match existing style in touched files: 2-space indentation, concise comments, and brace style already present in the file.
- Use `UPPER_SNAKE_CASE` for macros/constants (`START_FRAME`, `TIME_SEND`) and `camelCase` for functions (`detectDrivingMode`, `sendToHoverboard`).
- Keep hardware pin, baud, and protocol constants centralized near the top of `main.cpp`.

## Testing Guidelines
- There is no automated unit-test suite yet; validation is build + hardware checks.
- Minimum check: compile both controller environments (`nanoatmega328`, `nanoatmega328new`).
- Use sketches in `test/` for targeted hardware verification (Nunchuk input, controller behavior) before merging risky control changes.
- Document what hardware was tested (board, motor controller firmware, Nunchuk variant) in PR notes.

## Commit & Pull Request Guidelines
- Recent history follows Conventional Commit style: `feat:`, `fix(controller):`, `docs:`, `refactor:`, `chore:`.
- Prefer small, focused commits with imperative subjects (e.g., `fix(controller): debounce mode selection`).
- PRs should include: summary of behavior changes, test/build evidence, related issue/TODO reference, and photos/log snippets for hardware-facing changes.
