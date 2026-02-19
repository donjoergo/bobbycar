# TODO List

## V1 - Macher Festival 2025 🔥⚒️

- [x] Get Arduino Controller to working condition
  - [x] Control motor board with nunchuck
  - [x] Take a look at memory usage
- [x] Add different speedmodes
  - The mode should be selected based on the nunchucks joystick position at startup
  - Each mode should have the following properties
    - Max Speed Forward
    - Max Speed Backward
    - Acceleration Forward
    - Acceleration Backward
  - [x] The modes are:
    - [x] SuperSlow/Indoor/Children (max 3km/h)
    - [x] Slow/STVO (max 6km/h)
    - [x] Fast (max 17km/h / full 1000 throttle)
  - [x] Fix mode selection
    - [x] Mode selection should be always possible
- [ ] Make V1 code nicer
- [ ] upload all source files

## V2

- [ ] Add led ring for status feedback, like
  - [ ] battery percentage
  - [ ] errors, etc.
- [ ] Add fuel cap and on/off switch
- [ ] Fix beep beep horn
- [ ] Fix base plate
- [ ] lights
  - [ ] headlights
    - [ ] Dim the lights to simulate low beam and high beam
  - [ ] backlights
  - [ ] lights inside the body
- [ ] Add Racing Wheel
  - [ ] Communicate with racing wheel
  - [ ] Play racing wheel sounds remotely

## V3

- [ ] Add display
  - [ ] Current speed
  - [ ] Odometer
  - [ ] Trip counter
  - [ ] Battery percentage
  - [ ] Remaining km to 0%
  - [ ] Options for changing settings via the display
- [ ] Make battery larger
- [ ] Add gold connectors to the motors
- [ ] Add 4WD
- [ ] Implement Kamikaze mode (max > 30km/h / field weakening is used)
- [ ] Add a real and loud 12V electrical car horn :)

## VX - Macher Festival 2026 🔥⚒️

- [ ]  Make project sign for paddock area to showcase the features
  - [ ] Print QR code with link to Github project
- [ ] Implement hotspot mode so that features can be controlled remotely (also by visitors)
  - [ ] Light mode of front led stripe: off, Knight Rider, police Germany, police USA, rainbow, solid color (with color picker)
  - [ ] Light mode of front light: off, low beam, high beam, hazard lights, pulse
  - [ ] Light mode of back light: off, on (only brakelight), steady on, hazard lights, pulse
  - [ ] Light mode of inner body light: off, on, pulse
  - [ ] Quick settings for light modes: off, on (driving operation), on (showcase)
- [ ] Drive mode selection (only when not driving)
- [ ] Play sounds: honk the horn, etc.

- [ ] Implement alarm system
  - [ ] An alarm should sound when the car is armed and moved

## Vnext

- [ ] GPS logging with automatic lap time detection for race tracks
  - [ ] It shold be possible to set a position as the finish line. Every time this point is crossed, a lap is recorded.
