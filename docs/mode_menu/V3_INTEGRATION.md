# v3 seven-mode PA0 selector ported from giomuadongbacv2

Only the seven-mode selection and Bluetooth Classic SPP handshake are integrated.
Mode confirmation is RAM-only; it deliberately never triggers calibration,
exploration, speedrun, map deletion, or a motor test.

## PA0
- Short press: cycle to the next mode (first is Calibration).
- Hold at least 400 ms: select once. Debounce is 30 ms.
- A key held at boot is ignored until release.
- Long press before first short press does not select.

## Menu labels
1. Calibration
2. Run Slow
3. Run Faster
4. A* Run
5. Speedrun
6. Reset Map
7. System Test

## Bluetooth Classic SPP
Connect to the outgoing JDY-33 COM port with:
    py tools/classic_spp_dashboard.py --port COM16
The dashboard sends '?' and firmware replies 'SPP CONNECTED'.
Each short press emits one 'Mode N: Name' line; each long hold emits
one 'Selected mode N: Name' line. Periodic MPU/IR sensor readings
remain independent from these menu events.

## Important: motor safety and behavior
PA0 previously started a 720mm Wall PID run in v3. That start action
is disabled to prevent short-press menu navigation from starting motors.
Wall PID core remains in source. No mode action is mapped yet, so a
separate explicit motor start design is required in a future change.
Motor outputs remain locked. Existing gyro calibration during boot remains.

## Verify
    py tests/regression/mode_menu_native_test.py
    pio run -e wall_pid_4cell_96m
    pio run -e wall_pid_4cell_96m -t upload
Test physical PA0 and SPP with VM motor power disconnected.

## KEY initialized after calibration

The menu state machine now initializes only after `mpu.calibrated` is true
and the driver reaches `cal_state==3`. Holding PA0 during calibration must
not select a mode. The held key must be released before any new action.
After `W,MENU_READY`, a short press advances one step, and holding the
button >=400 ms confirms the focused mode only once. No mode action runs.
