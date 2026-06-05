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
    private final Telemetry telemetry;
    private final Follower follower;
    private final PIDFController headingLockController;
    private final IntakeSS intake;
    private final ShooterSS shooter;
    private final IntakeCommands intakeCommands;
    private final List<LynxModule> controlHubs;
    private final boolean isBlueAlliance;

    private boolean lastIntake = false;
    private boolean lastShooterRpmUp = false;
    private boolean lastShooterRpmDown = false;
    private long lastLoopTimeNs = 0L;
    private double loopHz = 0.0;
    private double headingLockErrorRad = 0.0;
    private double shooterRpmOffset = 0.0;

    public Robot(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.telemetry = telemetry;
        this.isBlueAlliance = isBlueAlliance;
        this.telemetry.setMsTransmissionInterval(RobotConstants.TELEMETRY_INTERVAL_MS);

        controlHubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hwm);
        headingLockController = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        intake = new IntakeSS(hwm, telemetry);
        shooter = new ShooterSS(hwm, telemetry);
        intakeCommands = new IntakeCommands(intake);
    }

    public void start() {
        if (!PoseStorage.hasValidPose()) {
            PoseStorage.setCurrentPose(PoseStorage.allianceStartPose(isBlueAlliance));
        }

        follower.setStartingPose(PoseStorage.currentPose);
        follower.startTeleopDrive();
    }

    public void update(Gamepad gamepad1) {
        updateLoopRate();
        clearBulkCache();

        boolean shooterActive = gamepad1.a;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                getTurnInput(gamepad1, shooterActive),
                false,
                Math.toRadians(fieldCentricOffsetDeg())
        );
        follower.update();

        PoseStorage.setCurrentPose(currentFollowerPose());

        updateIntakeCommands(gamepad1);
        updateShooterRpmOffset(gamepad1);

        intakeCommands.update();
        shooter.setRpmOffset(shooterRpmOffset);
        shooter.activateShooter(distanceToShootingGoal(), shooterActive);

        lastIntake = gamepad1.right_bumper;
        lastShooterRpmUp = gamepad1.dpad_up;
        lastShooterRpmDown = gamepad1.dpad_down;
        teleopTelemetry();
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    private void teleopTelemetry() {
        telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
        telemetry.addData("Robot position", "%.1f, %.1f, %.1f",
                robotX(),
                robotY(),
                Math.toDegrees(robotHeading()));
        telemetry.addData("Heading offset deg", "%.1f", fieldCentricOffsetDeg());
        telemetry.addData("Heading lock error deg", "%.1f", Math.toDegrees(headingLockErrorRad));
        telemetry.addData("Intake state", intakeCommands.getState());
        telemetry.addData("Shooter RPM offset", "%.0f", shooterRpmOffset);
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Loop Hz", "%.1f", loopHz);
    }

    private void clearBulkCache() {
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }

    private void updateLoopRate() {
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
            return calculateHeadingLockTurn();
        }

        headingLockController.reset();
        headingLockErrorRad = 0.0;
        return -gamepad1.right_stick_x * RobotConstants.TURN_MULTIPLIER;
    }

    private void updateIntakeCommands(Gamepad gamepad1) {
        boolean intakePressed = gamepad1.right_bumper && !lastIntake;
        if (intakePressed) {
            if (intakeCommands.isIntaking()) {
                intakeCommands.idle();
            } else {
                intakeCommands.intake();
            }
        }

        if (gamepad1.b) {
            if (!intakeCommands.isOuttaking()) {
                intakeCommands.outtake();
            }
        } else if (intakeCommands.isOuttaking()) {
            intakeCommands.idle();
        }

        if (gamepad1.left_bumper) {
            if (!intakeCommands.isTransferring()) {
                intakeCommands.transfer();
            }
        } else if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }
    }

    private void updateShooterRpmOffset(Gamepad gamepad1) {
        boolean shooterRpmUpPressed = gamepad1.dpad_up && !lastShooterRpmUp;
        boolean shooterRpmDownPressed = gamepad1.dpad_down && !lastShooterRpmDown;

        if (shooterRpmUpPressed) {
            shooterRpmOffset += RobotConstants.SHOOTER_RPM_ADJUST_STEP;
        } else if (shooterRpmDownPressed) {
            shooterRpmOffset -= RobotConstants.SHOOTER_RPM_ADJUST_STEP;
        }
    }

    private double calculateHeadingLockTurn() {
        double targetHeading = Math.atan2(shootingGoalY() - robotY(), shootingGoalX() - robotX());
        headingLockErrorRad = normalizeAngle(targetHeading - robotHeading());
        headingLockController.updateError(headingLockErrorRad);
        return Range.clip(
                headingLockController.run(),
                -RobotConstants.HEADING_LOCK_MAX_TURN_POWER,
                RobotConstants.HEADING_LOCK_MAX_TURN_POWER
        );
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    private double fieldCentricOffsetDeg() {
        return isBlueAlliance
                ? RobotConstants.FIELD_CENTRIC_OFFSET_BLUE_DEG
                : RobotConstants.FIELD_CENTRIC_OFFSET_RED_DEG;
    }

    private double shootingGoalX() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_X_BLUE : RobotConstants.SHOOTING_GOAL_X_RED;
    }

    private double shootingGoalY() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_Y_BLUE : RobotConstants.SHOOTING_GOAL_Y_RED;
    }

    private Pose currentFollowerPose() {
        return new Pose(robotX(), robotY(), robotHeading());
    }

    private double robotX() {
        return follower.getPose().getX();
    }

    private double robotY() {
        return follower.getPose().getY();
    }

    private double robotHeading() {
        return follower.getPose().getHeading();
    }

    private double distanceToShootingGoal() {
        return Math.hypot(shootingGoalX() - robotX(), shootingGoalY() - robotY());
    }
}
