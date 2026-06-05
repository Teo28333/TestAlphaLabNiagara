# TeleOp Explained

This document explains how the current TeleOp code works, starting from the Driver Station opmode and ending at the motors.

## Big Picture

There are two TeleOp choices on the Driver Station:

- `Red`
- `Blue`

They both run the same robot logic. The only difference is the alliance color passed into the robot:

- `Red` creates `new Robot(..., false)`
- `Blue` creates `new Robot(..., true)`

That alliance value changes the field-centric drive offset, the starting pose, and which shooting goal the robot aims at.

## Files Involved

- `opmodes/teleop/Red.java`: Driver Station entry point for red alliance.
- `opmodes/teleop/Blue.java`: Driver Station entry point for blue alliance.
- `robot/Robot.java`: Main TeleOp brain. Reads the gamepad and updates drive, intake, shooter, and telemetry.
- `commands/IntakeCommands.java`: Small intake state machine.
- `subsystems/IntakeSS.java`: Talks directly to intake motors and gate servo.
- `subsystems/ShooterSS.java`: Talks directly to shooter motors.
- `math/ShooterEquation.java`: Calculates target shooter RPM from distance.
- `robot/RobotConstants.java`: Tunable constants for offsets, goals, RPM step size, etc.

## TeleOp Lifecycle

FTC iterative opmodes run in three main steps:

1. `init()`
   - Runs when the opmode is selected and initialized.
   - Creates the `Robot` object.
   - Maps hardware, creates Pedro Pathing follower, intake, shooter, and command objects.

2. `start()`
   - Runs once when the Play button is pressed.
   - Sets the robot pose.
   - Starts Pedro Pathing teleop drive mode.

3. `loop()`
   - Runs over and over until the opmode stops.
   - Calls `robot.update(gamepad1)`.
   - Sends telemetry to the Driver Station.

## Driver Controls

| Control | What it does |
| --- | --- |
| Left stick Y | Drive forward/backward |
| Left stick X | Strafe left/right |
| Right stick X | Turn manually |
| `A` | Toggle shooter and automatic goal aiming on/off |
| Right bumper | Toggle intake on/off |
| `B` | Hold to outtake |
| Left bumper | Hold to transfer |
| D-pad up | Increase shooter RPM offset by 50 RPM |
| D-pad down | Decrease shooter RPM offset by 50 RPM |
| Share | Reset pose to the alliance human-zone pose and alliance heading |

## Drive

Drive is handled by Pedro Pathing's `Follower`.

Every loop, `Robot.update()` sends this to the follower:

- forward/backward power from left stick Y
- strafe power from left stick X
- turn power from either right stick X or heading lock
- alliance field-centric heading offset

Blue alliance uses a 180 degree field-centric offset. Red alliance uses 0 degrees.

## Shooting

The shooter runs when `A` has toggled shooter mode on.

When shooter mode is toggled off:

- shooter motor power is set to 0
- target RPM is set to 0
- shooter ready becomes false

When shooter mode is toggled on:

1. The robot calculates distance to the alliance shooting goal.
2. `ShooterEquation` converts that distance into a target RPM.
3. The D-pad RPM offset is added, clamped to plus or minus 200 RPM.
4. The PIDFS controller calculates motor power.
5. Both shooter motors receive the same power.
6. `shooterReady` becomes true if current RPM is close enough to target RPM.
7. The gamepad rumbles once when the shooter first becomes ready.

While shooter mode is toggled on, the robot also automatically turns toward the goal. This is called heading lock.

## Heading Lock

Normally, right stick X controls turning.

When shooter mode is toggled on, manual turning is replaced by automatic aiming:

1. The robot finds the angle from its current position to the shooting goal.
2. It compares that target angle to the robot's current heading.
3. A heading PIDF controller calculates turn power.
4. Turn power is clipped so it does not exceed the max allowed turn power.

This means the driver can move around while the robot keeps trying to face the goal.

## Intake

The intake uses `IntakeCommands` as a state machine.

The possible intake states are:

- `IDLE`
- `INTAKING`
- `OUTTAKING`
- `OPEN_GATE`
- `TRANSFERRING`

Right bumper toggles between `INTAKING` and `IDLE`.

`B` temporarily forces `OUTTAKING` while held.

Left bumper temporarily forces `TRANSFERRING` while held.

If multiple intake buttons are pressed at the same time, priority is:

1. Transfer
2. Intake
3. Outtake

## Intake Details

When intaking:

1. The gate closes.
2. The robot waits for the gate to settle.
3. The front rollers run.
4. The back roller runs unless current gets too high.
5. If back roller current goes above the limit, the back roller stops.

That current limit is probably being used as a simple "something is loaded" detector.

When outtaking:

- the gate opens
- both rollers run backward at half intake speed

When transferring:

- the gate opens
- the robot waits for the gate to settle
- both rollers run forward at transfer speed

When idle:

- the gate closes
- both intake motors stop
- timing and current-trigger state reset

## Telemetry

Telemetry shows:

- alliance
- robot position
- field-centric heading offset
- heading lock error
- intake state
- shooter RPM offset
- whether shooter is ready
- shooter-ready rumble status through gamepad feedback
- loop frequency
- shooter target/current RPM
- back roller current

## Current Behavior Summary

In plain English: one gamepad drives the robot and runs the mechanisms. Right bumper toggles intake, `B` and left bumper are hold actions, `A` toggles shooter plus automatic aiming, and Share resets pose to the alliance human-zone location with heading `0` on red or `180` on blue. When shooter mode is on, the robot automatically faces the correct goal for the selected alliance.
