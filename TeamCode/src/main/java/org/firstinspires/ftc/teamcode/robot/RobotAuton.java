package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

import java.util.List;

public class RobotAuton {
    public final Follower follower;
    public final IntakeSS intake;
    public final ShooterSS shooter;
    public final IntakeCommands intakeCommands;

    private final Telemetry telemetry;
    private final List<LynxModule> controlHubs;
    private final boolean isBlueAlliance;
    private final ElapsedTime actionTimer = new ElapsedTime();

    private State currentState = State.IDLE;
    private double actionTimeoutMs = 0.0;
    private double pathIntakeTimeoutMs = -1.0;
    private double shooterRpmOffset = 0.0;
    private boolean shooterModeEnabled = true;

    private enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FOLLOWING,
        FOLLOWING_AND_INTAKE,
        FOLLOWING_AND_OPEN_GATE
    }

    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.telemetry = telemetry;
        this.isBlueAlliance = isBlueAlliance;
        this.telemetry.setMsTransmissionInterval(RobotConstants.TELEMETRY_INTERVAL_MS);

        controlHubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hwm);
        intake = new IntakeSS(hwm, telemetry);
        shooter = new ShooterSS(hwm, telemetry);
        intakeCommands = new IntakeCommands(intake);
    }

    public void start(Pose startingPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        follower.setStartingPose(startingPose);
        PoseStorage.setCurrentPose(startingPose);
    }

    public void update() {
        clearBulkCache();
        follower.update();

        switch (currentState) {
            case INTAKING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                } else {
                    intakeCommands.transfer();
                }
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                if (pathIntakeTimeoutMs >= 0.0 && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_OPEN_GATE:
                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        intakeCommands.update();
        shooter.setRpmOffset(shooterRpmOffset);
        shooter.activateShooter(distanceToShootingGoal(), shooterModeEnabled);
        savePose();
        telemetry();
    }

    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState = State.INTAKING;
        actionTimer.reset();
    }

    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState = State.TRANSFERRING;
        actionTimer.reset();
    }

    public void followPath(Path path) {
        followPath(path, true);
    }

    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    public void followPath(PathChain path) {
        followPath(path, true);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        currentState = State.FOLLOWING;
    }

    public void followPathAndIntake(Path path) {
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndIntake(PathChain path) {
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndOpenGate(PathChain path) {
        followPathAndOpenGate(path, true);
    }

    public void followPathAndOpenGate(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        intakeCommands.openGate();
        currentState = State.FOLLOWING_AND_OPEN_GATE;
    }

    public void setShooterRpmOffset(double shooterRpmOffset) {
        this.shooterRpmOffset = shooterRpmOffset;
    }

    public void enableShooterMode() {
        shooterModeEnabled = true;
    }

    public void disableShooterMode() {
        shooterModeEnabled = false;
        shooter.activateShooter(0.0, false);
    }

    public boolean isShooterModeEnabled() {
        return shooterModeEnabled;
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public boolean isShooterReady() {
        return shooter.isReady();
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    public void forceIdle() {
        follower.breakFollowing();
        intakeCommands.idle();
        currentState = State.IDLE;
        pathIntakeTimeoutMs = -1.0;
    }

    public double getPathProgressPercent() {
        if (!follower.isBusy()) {
            return 100.0;
        }

        return Math.max(0.0, Math.min(100.0, follower.getPathCompletion() * 100.0));
    }

    public void savePose() {
        PoseStorage.setCurrentPose(currentFollowerPose());
    }

    private void telemetry() {
        telemetry.addData("Auton state", currentState);
        telemetry.addData("Path progress", "%.1f", getPathProgressPercent());
        telemetry.addData("Intake state", intakeCommands.getState());
        telemetry.addData("Shooter mode", shooterModeEnabled);
        telemetry.addData("Shooter ready", shooter.isReady());
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

    private double shootingGoalX() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_X_BLUE : RobotConstants.SHOOTING_GOAL_X_RED;
    }

    private double shootingGoalY() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_Y_BLUE : RobotConstants.SHOOTING_GOAL_Y_RED;
    }

    private double distanceToShootingGoal() {
        return Math.hypot(shootingGoalX() - robotX(), shootingGoalY() - robotY());
    }

    private void clearBulkCache() {
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }
}
