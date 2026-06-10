package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

import java.util.List;

public class Robot {
    // Shared FTC telemetry object. Anything added here appears on the Driver Station.
    private final Telemetry telemetry;
    // Pedro Pathing drive/localization object. It handles field-centric mecanum drive.
    private final Follower follower;
    // PIDF controller used only when the shooter button is held and the robot aims itself.
    private final PIDFController headingLockController;
    // Subsystems talk directly to hardware motors/servos.
    private final IntakeSS intake;
    private final ShooterSS shooter;
    // Command layer remembers which intake mode should be running.
    private final IntakeCommands intakeCommands;
    // Control hubs are cleared manually each loop so sensor/motor reads stay fresh.
    private final List<LynxModule> controlHubs;
    // Blue and red use different starting poses, field offsets, and shooting goals.
    private final boolean isBlueAlliance;

    // These remember the previous loop's operator button states, so a press can be detected once.
    private boolean lastIntake = false;
    private boolean lastShooterToggle = false;
    private boolean lastShooterRpmUp = false;
    private boolean lastShooterRpmDown = false;
    private boolean lastShooterReady = false;
    private boolean lastHumanZoneReset = false;
    // Gamepad 1 A toggles shooter and heading lock together.
    private boolean shooterActive = false;
    // Loop timing is only for telemetry, so drivers can see if the code is running slowly.
    private long lastLoopTimeNs = 0L;
    private double loopHz = 0.0;
    // Shown in telemetry to tell how far heading lock is from the goal angle.
    private double headingLockErrorRad = 0.0;
    // Driver-tunable RPM adjustment. D-pad changes this during TeleOp.
    private double shooterRpmOffset = 0.0;

    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.telemetry = telemetry;
        this.isBlueAlliance = isBlueAlliance;
        // Reduce telemetry spam. The loop can run faster than telemetry needs to send.
        this.telemetry.setMsTransmissionInterval(RobotConstants.TELEMETRY_INTERVAL_MS);

        // Manual bulk caching means we decide exactly when fresh hardware data is read.
        controlHubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Build every major robot part once during init.
        follower = Constants.createFollower(hwm);
        headingLockController = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        intake = new IntakeSS(hwm, telemetry);
        shooter = new ShooterSS(hwm, telemetry);
        intakeCommands = new IntakeCommands(intake);
    }

    public void start() {
        // If autonomous left a valid pose, keep it. Otherwise use the alliance start pose.
        if (!PoseStorage.hasValidPose()) {
            PoseStorage.setCurrentPose(PoseStorage.allianceStartPose(isBlueAlliance));
        }

        // Pedro needs a starting pose before it can drive field-centrically.
        follower.setStartingPose(PoseStorage.currentPose);
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        // First do housekeeping that should happen once per loop.
        updateLoopRate();
        clearBulkCache();

        // One-driver setup: gamepad 1 drives and controls every mechanism.
        updateShooterToggle(gamepad1);
        updatePoseReset(gamepad1);
        follower.setTeleOpDrive(
                // FTC sticks are negative when pushed forward, so invert them.
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                // Turn comes from the right stick, unless the shooter is aiming automatically.
                getTurnInput(gamepad1, shooterActive),
                // Robot-centric is false here; the last argument provides field-centric offset.
                false,
                Math.toRadians(fieldCentricOffsetDeg())
        );
        follower.update();

        // Save the latest pose so other opmodes can continue from here.
        PoseStorage.setCurrentPose(currentFollowerPose());

        // Read buttons and choose which subsystem modes should be active.
        updateIntakeCommands(gamepad1);
        updateShooterRpmOffset(gamepad1);

        // Actually run the selected intake mode and shooter behavior.
        intakeCommands.update();
        shooter.setRpmOffset(shooterRpmOffset);
        shooter.activateShooter(distanceToShootingGoal(), shooterActive);
        rumbleWhenShooterBecomesReady(gamepad1);

        // Store current button states so next loop can detect new presses.
        lastIntake = gamepad1.right_bumper;
        lastShooterToggle = gamepad1.a;
        lastShooterRpmUp = gamepad1.dpad_up;
        lastShooterRpmDown = gamepad1.dpad_down;
        lastHumanZoneReset = gamepad1.share;
        lastShooterReady = shooter.isReady();
        teleopTelemetry();
    }

    public void stop() {
        // Safe TeleOp shutdown: stop mechanisms and stop any active path/follower command.
        follower.breakFollowing();
        intakeCommands.idle();
        intakeCommands.update();
        shooter.activateShooter(0.0, false);
        shooterActive = false;
        lastShooterReady = false;
        PoseStorage.setCurrentPose(currentFollowerPose());
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    private void teleopTelemetry() {
        // Keep driver-facing status in one place so it is easy to add/remove lines.
        telemetry.addLine("Drive");
        telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
        telemetry.addData("Robot position", "%.1f, %.1f, %.1f",
                robotX(),
                robotY(),
                Math.toDegrees(robotHeading()));
        telemetry.addData("Heading offset deg", "%.1f", fieldCentricOffsetDeg());
        telemetry.addData("Heading lock error deg", "%.1f", Math.toDegrees(headingLockErrorRad));
        telemetry.addData("Pose reset", "share -> human zone");
        telemetry.addData("Loop Hz", "%.1f", loopHz);

        telemetry.addLine("Intake");
        telemetry.addData("Intake state", intakeCommands.getState());

        telemetry.addLine("Shooter");
        telemetry.addData("Shooter active", shooterActive);
        telemetry.addData("Shooter distance", "%.1f", distanceToShootingGoal());
        telemetry.addData("Shooter RPM offset", "%.0f", shooterRpmOffset);
        telemetry.addData("Shooter ready", shooter.isReady());
    }

    private void clearBulkCache() {
        // With manual caching, old hub data stays around until we clear it.
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }

    private void updateLoopRate() {
        // Measure how many loops per second the robot code is managing.
        long now = System.nanoTime();
        if (lastLoopTimeNs != 0L) {
            double dtSeconds = (now - lastLoopTimeNs) / 1.0e9;
            if (dtSeconds > 0.0) {
                loopHz = 1.0 / dtSeconds;
            }
        }
        lastLoopTimeNs = now;
    }

    private double getTurnInput(Gamepad gamepad1, boolean shooterActive) {
        if (shooterActive) {
            // While shooting, ignore manual turning and aim at the goal automatically.
            return calculateHeadingLockTurn();
        }

        // When not shooting, use normal right-stick turning and reset heading lock state.
        headingLockController.reset();
        headingLockErrorRad = 0.0;
        return -gamepad1.right_stick_x * RobotConstants.TURN_MULTIPLIER;
    }

    private void updateIntakeCommands(Gamepad gamepad1) {
        // Priority if buttons overlap: transfer first, intake second, outtake last.
        if (gamepad1.left_bumper) {
            intakeCommands.transfer();
            return;
        }

        // Hold actions above are done, so release them back to idle before handling intake toggle.
        if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        // Rising-edge check: this is true once when right bumper is first pressed.
        boolean intakePressed = gamepad1.right_bumper && !lastIntake;
        if (intakePressed) {
            // Right bumper toggles intake instead of needing to be held.
            if (intakeCommands.isIntaking()) {
                intakeCommands.idle();
            } else {
                intakeCommands.intake();
            }
            return;
        }

        if (intakeCommands.isIntaking()) {
            return;
        }

        if (gamepad1.b) {
            intakeCommands.outtake();
        } else if (intakeCommands.isOuttaking()) {
            intakeCommands.idle();
        }
    }

    private void updateShooterToggle(Gamepad gamepad1) {
        // Rising-edge check: gamepad 1 A flips shooter/heading lock once per press.
        boolean shooterTogglePressed = gamepad1.a && !lastShooterToggle;
        if (shooterTogglePressed) {
            shooterActive = !shooterActive;
        }
    }

    private void updateShooterRpmOffset(Gamepad gamepad1) {
        // D-pad RPM changes only once per press, not every loop while held.
        boolean shooterRpmUpPressed = gamepad1.dpad_up && !lastShooterRpmUp;
        boolean shooterRpmDownPressed = gamepad1.dpad_down && !lastShooterRpmDown;

        if (shooterRpmUpPressed) {
            shooterRpmOffset += RobotConstants.SHOOTER_RPM_ADJUST_STEP;
        } else if (shooterRpmDownPressed) {
            shooterRpmOffset -= RobotConstants.SHOOTER_RPM_ADJUST_STEP;
        }

        // Keep live RPM trim inside a sane range: plus or minus 200 RPM by default.
        shooterRpmOffset = Range.clip(
                shooterRpmOffset,
                -RobotConstants.SHOOTER_RPM_OFFSET_LIMIT,
                RobotConstants.SHOOTER_RPM_OFFSET_LIMIT
        );
    }

    private void updatePoseReset(Gamepad gamepad1) {
        // Share resets localization to the alliance human-zone pose and alliance heading.
        boolean humanZoneResetPressed = gamepad1.share && !lastHumanZoneReset;
        if (humanZoneResetPressed) {
            resetPose(humanZonePose());
        }
    }

    private void resetPose(Pose pose) {
        // Reset both Pedro and PoseStorage so drive and saved pose agree.
        if (PoseStorage.isValid(pose)) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            follower.setPose(pose);
            PoseStorage.setCurrentPose(pose);
        }
    }

    private Pose humanZonePose() {
        // Red resets to heading 0. Blue resets to heading 180 degrees.
        return new Pose(
                isBlueAlliance ? 133.0 : 8.5,
                8.5,
                isBlueAlliance ? Math.PI : 0.0
        );
    }

    private void rumbleWhenShooterBecomesReady(Gamepad gamepad1) {
        // Buzz once when shooter first reaches ready RPM. Do not rumble every ready loop.
        boolean shooterReady = shooter.isReady();
        if (shooterReady && !lastShooterReady) {
            gamepad1.rumble(300);
        }
    }

    private double calculateHeadingLockTurn() {
        // Aim angle is the direction from the robot's current position to the goal.
        double targetHeading = ShootingTarget.headingToGoal(robotX(), robotY(), isBlueAlliance);
        // Normalize keeps the error between -180 and +180 degrees, so the robot turns the short way.
        headingLockErrorRad = normalizeAngle(targetHeading - robotHeading());
        headingLockController.updateError(headingLockErrorRad);
        // Clip keeps PID output from asking for more turn power than allowed.
        return Range.clip(
                headingLockController.run(),
                -RobotConstants.HEADING_LOCK_MAX_TURN_POWER,
                RobotConstants.HEADING_LOCK_MAX_TURN_POWER
        );
    }

    private static double normalizeAngle(double radians) {
        // Wrap any angle into the range -PI to +PI.
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    private double fieldCentricOffsetDeg() {
        // Blue starts facing the opposite way, so its field-centric frame is rotated 180 degrees.
        return isBlueAlliance
                ? RobotConstants.FIELD_CENTRIC_OFFSET_BLUE_DEG
                : RobotConstants.FIELD_CENTRIC_OFFSET_RED_DEG;
    }

    private Pose currentFollowerPose() {
        // Copy the follower pose into our own Pose object for storage.
        return new Pose(robotX(), robotY(), robotHeading());
    }

    private double robotX() {
        // Current field X from Pedro localization.
        return follower.getPose().getX();
    }

    private double robotY() {
        // Current field Y from Pedro localization.
        return follower.getPose().getY();
    }

    private double robotHeading() {
        // Current robot heading in radians from Pedro localization.
        return follower.getPose().getHeading();
    }

    private double distanceToShootingGoal() {
        // Hypotenuse distance from robot to goal. Shooter RPM is based on this.
        return ShootingTarget.distanceToGoal(robotX(), robotY(), isBlueAlliance);
    }
}
