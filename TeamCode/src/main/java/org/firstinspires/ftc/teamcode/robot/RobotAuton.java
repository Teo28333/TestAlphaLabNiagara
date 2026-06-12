package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;

import java.util.List;

public class RobotAuton {
    // Public so autonomous opmodes can build paths with the same follower object.
    public final Follower follower;
    // Public subsystem handles are available if an auto ever needs direct access.
    public final IntakeSS intake;
    public final ShooterSS shooter;
    // Intake commands choose the current intake mode without exposing motor details.
    public final IntakeCommands intakeCommands;

    // FTC telemetry object used to print autonomous status to the Driver Station.
    private final Telemetry telemetry;
    private final PIDFController headingLockController;
    // Control hubs are manually cache-cleared each loop for fresh hardware data.
    private final List<LynxModule> controlHubs;
    // Alliance decides mirrored paths and which goal the shooter targets.
    private final boolean isBlueAlliance;
    // One timer is reused for timed intake, transfer, and intake-while-driving actions.
    private final ElapsedTime actionTimer = new ElapsedTime();
    // Separate timer for path failsafe, so timed intake/transfer actions stay accurate.
    private final ElapsedTime pathTimer = new ElapsedTime();

    // Current high-level autonomous action.
    private State currentState = State.IDLE;
    // Time limit for simple timed actions like intakeFor() and transferFor().
    private double actionTimeoutMs = 0.0;
    // Optional time limit for intake while following a path. Negative means no timeout.
    private double pathIntakeTimeoutMs = -1.0;
    // Extra RPM added to the shooter's distance-based target.
    private double shooterRpmOffset = 0.0;
    private double headingLockErrorRad = 0.0;
    private double targetHeadingRad = 0.0;
    // Telemetry flag that shows whether the last path ended because of the timeout.
    private boolean pathTimedOut = false;
    // Auto keeps the shooter spun up unless code explicitly disables it.
    private boolean shooterModeEnabled = true;

    private enum State {
        // No path or timed subsystem action is currently running.
        IDLE,
        // Intake is running until actionTimeoutMs expires.
        INTAKING,
        // Transfer is running until actionTimeoutMs expires.
        TRANSFERRING,
        // Follower is driving a path by itself.
        FOLLOWING,
        // Follower is driving a path while intake runs.
        FOLLOWING_AND_INTAKE,
        // Follower is driving a path while the intake gate stays open.
        FOLLOWING_AND_OPEN_GATE
    }

    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.telemetry = telemetry;
        this.isBlueAlliance = isBlueAlliance;
        // Slow telemetry sending a bit so the loop is not wasting time on radio updates.
        this.telemetry.setMsTransmissionInterval(RobotConstants.TELEMETRY_INTERVAL_MS);

        // Manual bulk caching lets update() control when sensor/motor values refresh.
        controlHubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Create the drive follower and every autonomous subsystem once during init.
        follower = Constants.createFollower(hwm);
        headingLockController = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        intake = new IntakeSS(hwm, telemetry);
        shooter = new ShooterSS(hwm, telemetry);
        intakeCommands = new IntakeCommands(intake);
    }

    public void start(Pose startingPose) {
        // Auto paths need a real starting pose. Null would make localization meaningless.
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        // Pedro and PoseStorage must agree on where the robot starts.
        follower.setStartingPose(startingPose);
        PoseStorage.setCurrentPose(startingPose);
    }

    public void update() {
        // Refresh hardware data and let Pedro advance path following.
        clearBulkCache();
        follower.update();

        // State machine: decide whether the current action is finished.
        switch (currentState) {
            case INTAKING:
                // Timed intake stops automatically when its timeout is reached.
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                // Transfer should feed balls only; clear any stale auto-aim turn command.
                stopAutoAimTurn();
                // Transfer keeps feeding until its timeout expires.
                if (actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                } else {
                    intakeCommands.transfer();
                }
                break;

            case FOLLOWING:
                // A plain path is done when Pedro reports it is no longer busy.
                if (pathTimedOut()) {
                    finishTimedOutPath();
                } else if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                // Optional timeout can stop intake before the path finishes.
                if (pathIntakeTimeoutMs >= 0.0 && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                // When the path ends, stop intake and return to idle.
                if (pathTimedOut()) {
                    finishTimedOutPath();
                } else if (!follower.isBusy()) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_OPEN_GATE:
                // Gate-open path returns to idle as soon as the path finishes.
                if (pathTimedOut()) {
                    finishTimedOutPath();
                } else if (!follower.isBusy()) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // Apply selected subsystem commands after the state machine picks them.
        intakeCommands.update();
        shooter.setRpmOffset(shooterRpmOffset);
        shooter.activateShooter(distanceToShootingGoal(), shooterModeEnabled);
        // Keep pose available for TeleOp or later opmodes.
        savePose();
        telemetry();
    }

    public void intakeFor(double timeoutMs) {
        // Start intake now and let update() stop it after timeoutMs.
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState = State.INTAKING;
        actionTimer.reset();
    }

    public void transferFor(double timeoutMs) {
        // Start transfer now and let update() stop it after timeoutMs.
        stopAutoAimTurn();
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState = State.TRANSFERRING;
        actionTimer.reset();
    }

    public void followPath(Path path) {
        // Default path behavior holds the final pose.
        followPath(path, true);
    }

    public void followPath(Path path, boolean holdEnd) {
        // Start a single Pedro path and mark auto as busy following.
        intakeCommands.idle();
        follower.followPath(path, holdEnd);
        startPathTimer();
        currentState = State.FOLLOWING;
    }

    public void followPath(PathChain path) {
        // Default path-chain behavior holds the final pose.
        followPath(path, true);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        // Start a Pedro path chain and mark auto as busy following.
        intakeCommands.idle();
        follower.followPath(path, holdEnd);
        startPathTimer();
        currentState = State.FOLLOWING;
    }

    public void followPathAndIntake(Path path) {
        // Intake for the full path by default.
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(Path path, boolean holdEnd) {
        // Drive and intake until the path is done.
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        startPathTimer();
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs) {
        // Timed intake defaults to holding the final path pose.
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs, boolean holdEnd) {
        // Drive the path while intake runs for only intakeTimeoutMs.
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        startPathTimer();
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndIntake(PathChain path) {
        // Intake for the full path chain by default.
        followPathAndIntake(path, true);
    }

    public void followPathAndIntake(PathChain path, boolean holdEnd) {
        // Drive the path chain and intake until the path chain is done.
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        startPathTimer();
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs) {
        // Timed intake defaults to holding the final path-chain pose.
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }

    public void followPathAndIntakeFor(PathChain path, double intakeTimeoutMs, boolean holdEnd) {
        // Drive the path chain while intake runs for only intakeTimeoutMs.
        follower.followPath(path, holdEnd);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        startPathTimer();
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndOpenGate(PathChain path) {
        // Open-gate path defaults to holding the final pose.
        followPathAndOpenGate(path, true);
    }

    public void followPathAndOpenGate(PathChain path, boolean holdEnd) {
        // Drive the path chain while the intake gate stays open.
        follower.followPath(path, holdEnd);
        intakeCommands.openGate();
        startPathTimer();
        currentState = State.FOLLOWING_AND_OPEN_GATE;
    }

    public void setShooterRpmOffset(double shooterRpmOffset) {
        // Lets an auto tune shooter RPM without changing the shooter equation.
        this.shooterRpmOffset = shooterRpmOffset;
    }

    public void enableShooterMode() {
        // Auto shooter control loop will run every update().
        shooterModeEnabled = true;
    }

    public void disableShooterMode() {
        // Stop the auto shooter control loop and immediately shut off motors.
        shooterModeEnabled = false;
        shooter.activateShooter(0.0, false);
    }

    public boolean isShooterModeEnabled() {
        // Expose shooter mode for telemetry or auto decisions.
        return shooterModeEnabled;
    }

    public boolean isBusy() {
        // Auto is busy whenever its internal state is not idle.
        return currentState != State.IDLE;
    }

    public boolean isShooterReady() {
        // Auto uses a wider high-side RPM window so overshoot does not block transfer forever.
        return shooter.isReadyForAuton();
    }

    public boolean aimAtShootingGoal() {
        // Use live pose, like TeleOp heading lock, instead of trusting a fixed path heading.
        if (follower.isBusy()) {
            return false;
        }

        targetHeadingRad = ShootingTarget.headingToGoal(robotX(), robotY(), isBlueAlliance);
        headingLockErrorRad = normalizeAngle(targetHeadingRad - robotHeading());

        if (isAimedAtShootingGoal()) {
            headingLockController.reset();
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false, Math.toRadians(fieldCentricOffsetDeg()));
            return true;
        }

        headingLockController.updateError(headingLockErrorRad);

        double turnPower = Range.clip(
                headingLockController.run() * autoAimTurnSign(),
                -RobotConstants.HEADING_LOCK_MAX_TURN_POWER,
                RobotConstants.HEADING_LOCK_MAX_TURN_POWER
        );

        follower.startTeleopDrive();
        follower.setTeleOpDrive(0.0, 0.0, turnPower, false, Math.toRadians(fieldCentricOffsetDeg()));
        return isAimedAtShootingGoal();
    }

    public void stopAutoAimTurn() {
        // Keep the follower in teleop-drive mode but remove any leftover turn command.
        headingLockController.reset();
        follower.breakFollowing();
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0.0, 0.0, 0.0, false, Math.toRadians(fieldCentricOffsetDeg()));
    }

    public boolean isAimedAtShootingGoal() {
        targetHeadingRad = ShootingTarget.headingToGoal(robotX(), robotY(), isBlueAlliance);
        headingLockErrorRad = normalizeAngle(targetHeadingRad - robotHeading());
        return Math.abs(headingLockErrorRad) <= Math.toRadians(RobotConstants.AUTON_SHOOT_HEADING_TOL_DEG);
    }

    public boolean isBlueAlliance() {
        // Used by auto code that needs alliance-specific behavior.
        return isBlueAlliance;
    }

    public void forceIdle() {
        // Emergency cleanup: stop path following and return intake to safe idle.
        follower.breakFollowing();
        intakeCommands.idle();
        currentState = State.IDLE;
        pathIntakeTimeoutMs = -1.0;
    }

    public double getPathProgressPercent() {
        // If Pedro is not following, report complete instead of stale progress.
        if (!follower.isBusy()) {
            return 100.0;
        }

        // Convert Pedro's 0.0-1.0 completion value into a clamped percentage.
        return Math.max(0.0, Math.min(100.0, follower.getPathCompletion() * 100.0));
    }

    public void savePose() {
        // Store the latest localized pose globally for the next opmode.
        PoseStorage.setCurrentPose(currentFollowerPose());
    }

    private void telemetry() {
        // Autonomous status lines shown on the Driver Station.
        telemetry.addData("Auton state", currentState);
        telemetry.addData("Path progress", "%.1f", getPathProgressPercent());
        telemetry.addData("Path timeout", pathTimedOut);
        telemetry.addData("Intake state", intakeCommands.getState());
        telemetry.addData("Shooter mode", shooterModeEnabled);
        telemetry.addData("Shooter ready", isShooterReady());
        telemetry.addData("Shooter distance", "%.1f", distanceToShootingGoal());
        telemetry.addData("Auto aim target deg", "%.1f", Math.toDegrees(targetHeadingRad));
        telemetry.addData("Auto aim error deg", "%.1f", Math.toDegrees(headingLockErrorRad));
    }

    private void startPathTimer() {
        // Each new path gets its own 4500 ms chance to finish.
        pathTimedOut = false;
        pathTimer.reset();
    }

    private boolean pathTimedOut() {
        // The failsafe only matters while Pedro still thinks a path is active.
        return follower.isBusy() && pathTimer.milliseconds() >= RobotConstants.AUTON_PATH_TIMEOUT_MS;
    }

    private void finishTimedOutPath() {
        // Give up on the stuck path and let the outer auto state machine continue.
        pathTimedOut = true;
        follower.breakFollowing();
        intakeCommands.idle();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.IDLE;
    }

    private Pose currentFollowerPose() {
        // Copy only valid follower poses into storage; otherwise keep the last trusted pose.
        Pose pose = follower.getPose();
        if (PoseStorage.isValid(pose)) {
            return new Pose(pose.getX(), pose.getY(), pose.getHeading());
        }
        return PoseStorage.currentPose;
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
        // Current heading in radians from Pedro localization.
        return follower.getPose().getHeading();
    }

    private double fieldCentricOffsetDeg() {
        // Match TeleOp's alliance drive frame when auto aim temporarily uses teleop drive.
        return isBlueAlliance
                ? RobotConstants.FIELD_CENTRIC_OFFSET_BLUE_DEG
                : RobotConstants.FIELD_CENTRIC_OFFSET_RED_DEG;
    }

    private double autoAimTurnSign() {
        // Red is mirrored, so its autonomous aiming correction may need the opposite turn sign.
        return isBlueAlliance
                ? RobotConstants.AUTON_AIM_TURN_SIGN_BLUE
                : RobotConstants.AUTON_AIM_TURN_SIGN_RED;
    }

    private double distanceToShootingGoal() {
        // Shooter RPM is based on straight-line distance to the goal.
        return ShootingTarget.distanceToGoal(robotX(), robotY(), isBlueAlliance);
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    private void clearBulkCache() {
        // Manual bulk caching requires clearing each hub before fresh reads.
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }
}
