# Sharp Face Robotics #30450 — BioBuzz (2026–27)

Robot code for FTC team **Sharp Face Robotics #30450**. Built on the FTC SDK 11.2,
[Pedro Pathing 3](https://pedropathing.com) for driving and localization, and
[Panels](https://panels.bylazar.com) for live tuning.

> The stock FTC SDK readme now lives in [doc/FTC_SDK_README.md](doc/FTC_SDK_README.md).

## Layout

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── robot/                  subsystems: one class per mechanism
│   ├── RobotHardware       every motor/servo/sensor + Pedro follower
│   ├── RobotConfig         every constant and tunable (live-editable in Panels)
│   ├── Drivetrain          field/robot-centric mecanum through Pedro
│   ├── Intake              roller, auto-stop when full, jam detection
│   ├── Storage             tracks which elements are inside, in order
│   ├── ShooterConfig       distance → flywheel speed + hood angle, shoot-on-the-move
│   ├── Launcher            flywheel (velocity / PF / bang-bang) + hood
│   ├── Turret              CR servo turret, hall-effect homing, cable unwinding
│   ├── Trigger             gate + feeder, only fires when shooter and turret are ready
│   ├── HiveVision          Limelight AprilTag: averaged ta + MegaTag2 relocalization
│   ├── ElementVision       Limelight detector: finds pollen/nectar on the field
│   ├── PoseStorage         carries the pose from auto into TeleOp
│   └── helperClasses/      InterpLUT, MotionProfile, ShotDetector, FieldDrawing, MatchLogger, ValueTuner
├── teleop/                 TeleopBase + CompTeleopRed/Blue, pit test + tuning OpModes
├── auto/                   AutoBase + Subordinate / Parallel Inside / Parallel Outside autos
└── pedro/                  Pedro Constants (follower setup) + tuning procedures
```

## How the pieces fit

Subsystems never call each other. Each loop, the OpMode reads one subsystem's output and
passes it into the next one as a parameter:

```mermaid
flowchart LR
    F[Pedro follower<br/>pose + velocity] --> SC[ShooterConfig]
    HV[HiveVision<br/>average ta] --> SC
    SC -- turret angle --> T[Turret]
    SC -- velocity + hood angle --> L[Launcher]
    S[Storage] -- next element --> L
    S -- isFull --> I[Intake]
    L -- isReady --> TR[Trigger]
    T -- isOnTarget --> TR
    L -- shotDetected --> S
```

One loop of `TeleopBase` / `AutoBase`:

1. `robot.clearBulkCache()`: one hub read for every sensor this loop
2. `follower.update()`, then read the pose and velocity
3. vision, storage, intake
4. `shooterConfig.calculateShootingConfig(...)`, then turret, then launcher
5. trigger fires only if `launcher.isReady() && turret.isOnTarget()`
6. a velocity dip on the flywheel (`launcher.shotDetected()`) removes one element from storage

## Coding protocol

From the team software doc:

- **Every subsystem looks like this.** It gets `RobotHardware` in the constructor and uses
  `robot.someMotor` directly. Never call `hardwareMap` or declare motors in a subsystem.
  ```java
  public class Intake {
      private final RobotHardware robot;
      public Intake(RobotHardware robot) { this.robot = robot; }
      public void runIntake() { robot.intakeMotor.setPower(RobotConfig.INTAKE_POWER); }
  }
  ```
- **No calling other subsystems.** The only allowed exceptions are `RobotConfig` constants
  and `RobotHardware`. Anything else comes in as a parameter. `RobotConfig` (and the static
  `PoseStorage`) don't use the constructor pattern.
- **No `Math.pow` for squares or cubes.** Write `x * x` or `x * x * x`.
- **No `new` inside the loop.** Create timers, arrays and the like once as fields. The loops
  in `TeleopBase` and `AutoBase` only allocate on a button press.
- **Git:** push work to `predevelopment`. Once it's tested on the robot, commit *your file*
  to `main` in a single commit. Never merge the whole branch, because others might not be done.

## Roles

| Area | Files | Owner |
|---|---|---|
| Drivetrain | `robot/Drivetrain.java`, `pedro/Constants.java` | Arnav |
| Robot hardware / config | `robot/RobotHardware.java`, `robot/RobotConfig.java` | Arnav |
| Launcher, shooter config, turret | `robot/Launcher.java`, `robot/ShooterConfig.java`, `robot/Turret.java` | Alex |
| Hive vision, element vision | `robot/HiveVision.java`, `robot/ElementVision.java` | Gautham |
| Storage, intake | `robot/Storage.java`, `robot/Intake.java` | Vibhav |
| Trigger (+ gate), pose storage | `robot/Trigger.java`, `robot/PoseStorage.java` | Ryan |
| Subordinate auto | `auto/SubordinateAutoBase.java` (+ Red/Blue) | Alex |
| Parallel autos (inside, outside) | `auto/ParallelAutoInsideBase.java`, `auto/ParallelAutoOutsideBase.java` (+ Red/Blue) | Vivek |
| TeleOp red/blue | `teleop/TeleopBase.java`, `teleop/CompTeleopRed.java`, `teleop/CompTeleopBlue.java` | Arnav |

## Robot configuration (Driver Station)

Names must match `RobotConfig` exactly:

| Device | Type | Name |
|---|---|---|
| Drive motors | goBILDA motor | `front_left_drive`, `back_left_drive`, `front_right_drive`, `back_right_drive` |
| Odometry | goBILDA Pinpoint | `pinpoint` |
| Intake | motor | `intake_motor` |
| Storage entry sensor | REV Color Sensor V3 | `storage_color` |
| Flywheel | motor | `launch_motor` |
| Hood angle | servo ×2 | `hood_angle_servo_1`, `hood_angle_servo_2` |
| Hood gap | servo ×2 | `hood_gap_servo_1`, `hood_gap_servo_2` |
| Turret | CR servo | `turret_servo` |
| Turret encoder | motor port (encoder only) | `turret_encoder` |
| Turret zero | REV magnetic limit switch (digital) | `turret_hall` |
| Feeder | motor | `trigger_motor` |
| Gate | servo | `gate_servo` |
| Camera | Limelight 3A | `limelight` |

Run **SubsystemTest** first on a new robot: it drives every mechanism by hand and shows
every sensor, so wiring mistakes show up before a match.

The shooter test OpModes (`ShooterTestPanels`, `FlywheelPFTuning*`) call
`robot.initLauncher()`, which only needs `launch_motor`, so they run on a bare shooter bench.

## TeleOp controls

| Driver 1 (gamepad1) | | Driver 2 (gamepad2) | |
|---|---|---|---|
| Left stick | drive | A | flywheel on/off |
| Right stick | turn | Right trigger (hold) | **fire** |
| Right bumper | slow mode | X | auto-fire on/off |
| Right trigger | intake | B | manual turret on/off (right stick X) |
| Left trigger | outtake | D-pad left/right | turret trim ±1° |
| A (hold) | aim chassis at hive | D-pad down | clear trim |
| Left bumper | snap pose to Limelight | Y | re-home turret |
| Options | reset field-centric heading | Start | turret: call current spot zero (if homing fails) |
| Back | field ↔ robot centric | Back | reset storage count |

**Shooting modes:**
- **Auto spin-up:** the flywheel spins up whenever storage has elements in it
  (`AUTO_SPINUP_WHEN_LOADED`), so it's already at speed when you pull fire.
- **Auto-fire (X):** fires on its own whenever the robot is loaded, within
  `MIN/MAX_SHOT_DISTANCE`, and the shot is ready. Driver 2 just watches.
- **Turret down?** Press B for manual turret on driver 2's right stick, or have driver 1 hold
  A to turn the whole robot toward the hive. In both cases the driver judges the aim, so
  firing doesn't wait for the turret to report on target.

**On-screen stats and warnings:** shots fired, elements intaken and last cycle time, plus
warnings for low battery, a disconnected Limelight, a slow loop, or a match log that isn't writing.

**Match logs:** every TeleOp and auto writes a CSV of pose, flywheel, turret, storage and
firing to `/sdcard/FIRST/data/matchlogs/` on the Control Hub, at 20 Hz. Pull them with
`adb pull /sdcard/FIRST/data/matchlogs` or Android Studio's Device Explorer, then graph them
in a spreadsheet to see why a shot missed or where cycle time went.

**Feedback:** gamepad1 double-buzzes when storage fills. Gamepad2's light is **green** when a
shot is ready, **yellow** while the turret homes, and **red** if homing failed (press Start
with the turret pointing forward). Both controllers buzz at 30 s and 10 s left.

**Optional assists** (off by default until Pedro and the Limelight are tuned; turn on in `RobotConfig`):
- `LOCK_POSITION_WHILE_FIRING`: Pedro holds the robot in place while firing, so defense
  can't push it off the shot. Moving a stick cancels it.
- `AUTO_RELOCALIZE`: snap odometry to the Limelight whenever the robot is nearly stopped and
  they disagree by more than 2 in.

Panels shows all telemetry, plus a live field view with the robot, its heading, the aim line
and the hive.

## Autos

`AutoBase` handles the subsystems, the loop and `PoseStorage`. Each auto only lists its
steps for **blue**, using `drive`, `driveIntaking`, `shoot` and `pause`. Red mirrors them
automatically across the field centre line, so there's one set of numbers to tune per auto.

- **Subordinate:** preloads, the far spike only, then park out of the partner's way.
- **Parallel Inside:** start near the hive; preloads, then the close and middle spikes.
- **Parallel Outside:** start far; preloads, then the far and middle spikes from the far zone.

Set the robot up with the **turret facing straight forward**. Auto uses that position as
turret zero instead of spending time homing.

**Init menu (gamepad1, during init):** D-pad up/down sets a start delay in 0.5 s steps, to
stay out of a partner's way. D-pad left/right sets the preload count.

The flywheel and turret spin up during the delay, and the intake keeps running for a moment
after each pickup so the last element gets all the way in.

## Test & tuning OpModes (group "Test")

| OpMode | Use it to |
|---|---|
| `SubsystemTest` | run each mechanism by hand; check motor directions, turret encoder direction, element hues |
| `TurretTuner` | homes, then holds a target or steps between two; tune `TURRET_KP/KD/KS` live in Panels |
| `ShotTableTuner` | at each distance, adjust velocity and hood in Panels until shots score, then press A to log the row |
| `FlywheelPFTuning*`, `ShooterTest*` | flywheel only, on the shooter bench |

## Unit tests

`TeamCode/src/test` has JUnit tests for the pure math: interpolation, motion profile, shot
detection, shooter geometry, turret wrap-around, color classification and auto mirroring. They
run on the computer, with no robot needed:

```
./gradlew :TeamCode:testDebugUnitTest
```

Or right-click `TeamCode/src/test/java` in Android Studio, then Run Tests.

## Tuning checklist

Every placeholder is marked `TODO(tune)`, `TODO(measure)` or `TODO(confirm)`. Search for `TODO(`.

1. **Pedro:** run the Pinpoint, Mecanum and Foresight tuners in `pedro/procedures`, then
   paste their output into `pedro/Constants.java`.
2. **Flywheel:** use `FlywheelPFTuningPanels` (or the Gamepad version) to tune `kP`/`kF`,
   then pick `Launcher.controlMode`.
3. **Shot table:** use `ShotTableTuner` at several distances and copy the logged rows into
   `distance`, `velocity`, `pollenHoodAngle`, `nectarHoodAngle` and `timeOfFlight` in
   `ShooterConfig`.
   `InterpLUT` needs at least two points with x strictly increasing.
4. **Turret:** in `SubsystemTest`, confirm left bumper makes the raw ticks go up; if not, set
   `TURRET_ENCODER_REVERSED`. Then set `TURRET_TICKS_PER_RAD` and the soft limits, and tune
   with `TurretTuner`.
5. **Storage:** hold each element at the sensor in `SubsystemTest` and set the hue windows.
6. **Field:** measure hive positions, tag IDs and every auto pose.
7. **Vision:** check the FTC → Pedro conversion in `HiveVision.readBotpose` on a known spot.
