# MPU Test restored from giomuadongbacv2

## Source and scope

Source branch: giomuadongbacv2, commit 72c9867d982e809ebf0009236c011aa31738f3ac.
Target branch: giomuadongbac (Motion Control).

The target firmware does not have v2's Mode 7 menu. The MPU Log/manual yaw
feature is therefore restored as a dedicated "MPU Test" tab in the current
SPP dashboard, without replacing existing Motion Control profiles. This
does not include motorized Pivot 90/180 test routines from v3.

## Operation

1. Pull giomuadongbac and build/upload its default genericSTM32F411CE
   PlatformIO profile with ST-Link.
2. Keep the robot still while MPU gyro calibration completes.
3. Start tools/classic_spp_dashboard.py and connect the JDY-33 outgoing COM.
4. Open the "MPU Test" tab; select "Start MPU Log".
5. After the MCU acknowledges the Start command, the host opens a new CSV.
6. Rotate the robot by hand and observe yaw, gyro-Z, sample age, and 2D yaw.
7. Use "Zero yaw" to reset the heading, and "Stop" to close and save the CSV.
8. Each Start/Stop pair writes a separate timestamped file in
   logs/spp_runs/mode 7 - mpu test - log receiver/

## Protocol and CSV

Structured commands: $id,MPU_TEST_START; $id,MPU_TEST_STOP;
$id,MPU_TEST_ZERO. Each command is newline terminated.

Samples:
  MPU,ms=N,valid=1,age_us=N,yaw_cd=N,gz_cd=N
  MPU,ms=N,valid=0,age_us=N

CSV fields: ms, valid, age_us, yaw_deg, gz_dps.
Yaw and gyro-Z fields are output in hundredths of a degree and hundredths
of a degree per second respectively, then converted by the host.
ms is elapsed MCU milliseconds since the Start command.
Yaw integration can drift with gyro bias.

## Safety and limitations

Start is rejected if a motion command is active or MPU is not calibrated
and fresh. Entering MPU Test calls Motion_Stop() and blocks SPP motion
commands and PA0 quick step until stopped. STOP remains available.
This commands PWM zero but is not a hardware motor driver safety latch.
Other sensor acquisition continues in the normal motion-control ISR.
The actual robot's motor power and radio throughput must be checked on
a fixture with both wheels raised and a physical motor-power cutoff.
Firmware compilation and on-device behavior require local verification.
