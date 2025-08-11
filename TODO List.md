## V1 - Macher Festival
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
  - [ ] OPtions for changing settings via the display
- [ ] Make battery larger
- [ ] Add gold connectors to the motors
- [ ] Add 4WD
- [ ] Implement Kamikaze mode (max > 30km/h / field weakening is used)
- [ ] Add a real and loud 12V electrical car horn :)
