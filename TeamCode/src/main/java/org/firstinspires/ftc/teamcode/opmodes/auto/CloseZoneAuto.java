package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShootingTarget;

abstract class CloseZoneAuto extends OpMode {
    // How long the transfer motors run when feeding balls into the shooter.
    private static final double SHOOT_TIME_MS = 2500.0;
    // Base path coordinates are written for blue. Red uses Pose.mirror().
    private static final Pose BLUE_START_POSE = new Pose(
            19.034629404617252,
            117.86452004860269,
            Math.toRadians(146)
    );
    // Shared shooting location used after preload and each spike pickup.
    // Heading is calculated like TeleOp auto-aim: face from this pose toward the goal.
    private static final Pose BLUE_SHOOTING_POSE = blueShootingPose();

    // RobotAuton owns follower, intake, shooter, and auto state helpers.
    private RobotAuton robot;
    // All Pedro paths are built after robot exists because they need robot.follower.
    private Paths paths;
    // Limits how long live auto aim can keep turning while waiting for shooter RPM.
    private final ElapsedTime aimSettleTimer = new ElapsedTime();
    // Limits how long auto can wait for shooter RPM before continuing the routine.
    private final ElapsedTime shooterWaitTimer = new ElapsedTime();
    private boolean aimSettleStarted = false;
    private boolean shooterWaitStarted = false;
    private boolean shooterWaitTimedOut = false;
    // Outer routine state: this chooses which path/action comes next.
    private State state = State.GO_SHOOT_PRELOAD;

    private enum State {
        // Drive from start to the first shooting pose.
        GO_SHOOT_PRELOAD,
        // Feed the preload into the shooter.
        SHOOT_PRELOAD,
        // Drive to first spike while intaking.
        GO_INTAKE_FIRST_SPIKE,
        // Drive back to the shooting pose.
        GO_SHOOT_FIRST_SPIKE,
        // Feed first spike balls.
        SHOOT_FIRST_SPIKE,
        // Drive to second spike while intaking.
        GO_INTAKE_SECOND_SPIKE,
        // Drive back to the shooting pose.
        GO_SHOOT_SECOND_SPIKE,
        // Feed second spike balls.
        SHOOT_SECOND_SPIKE,
        // Only 12-ball auto: drive to third spike while intaking.
        GO_INTAKE_THIRD_SPIKE,
        // Only 12-ball auto: drive back to the shooting pose.
        GO_SHOOT_THIRD_SPIKE,
        // Only 12-ball auto: feed third spike balls.
        SHOOT_THIRD_SPIKE,
        // Drive to the final parking/exit location.
        LEAVE_ZONE,
        // Finished; robot should sit idle.
        DONE
    }

    // Small wrapper classes provide alliance without duplicating the whole routine.
    protected abstract boolean isBlueAlliance();
    // Small wrapper classes choose between 9-ball and 12-ball versions.
    protected abstract boolean isTwelveBallAuto();

    private static Pose blueShootingPose() {
        double x = 43.295;
        double y = 96.878;
        return new Pose(x, y, ShootingTarget.headingToGoal(x, y, true));
    }

    @Override
    public void init() {
        // Build robot and paths while the Driver Station is in INIT.
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        paths = new Paths();
        // Start at the blue base pose or mirrored red pose.
        robot.start(alliancePose(BLUE_START_POSE));

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // First action after Play: spin shooter and drive to preload shot location.
        state = State.GO_SHOOT_PRELOAD;
        robot.enableShooterMode();
        robot.followPath(paths.goShootPreload);
    }

    @Override
    public void loop() {
        // Let RobotAuton update path following, intake/shooter, and its internal state.
        robot.update();

        // Outer state machine decides the next action when RobotAuton finishes the current one.
        switch (state) {
            case GO_SHOOT_PRELOAD:
                if (readyToShoot()) {
                    robot.transferFor(SHOOT_TIME_MS);
                    state = State.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (!robot.isBusy()) {
                    robot.followPathAndIntake(paths.goIntakeFirstSpike);
                    state = State.GO_INTAKE_FIRST_SPIKE;
                }
                break;

            case GO_INTAKE_FIRST_SPIKE:
                if (!robot.isBusy()) {
                    robot.followPath(paths.goShootFirstSpike);
                    state = State.GO_SHOOT_FIRST_SPIKE;
                }
                break;

            case GO_SHOOT_FIRST_SPIKE:
                if (readyToShoot()) {
                    robot.transferFor(SHOOT_TIME_MS);
                    state = State.SHOOT_FIRST_SPIKE;
                }
                break;

            case SHOOT_FIRST_SPIKE:
                if (!robot.isBusy()) {
                    robot.followPathAndIntake(paths.goIntakeSecondSpike);
                    state = State.GO_INTAKE_SECOND_SPIKE;
                }
                break;

            case GO_INTAKE_SECOND_SPIKE:
                if (!robot.isBusy()) {
                    robot.followPath(paths.goShootSecondSpike);
                    state = State.GO_SHOOT_SECOND_SPIKE;
                }
                break;

            case GO_SHOOT_SECOND_SPIKE:
                if (readyToShoot()) {
                    robot.transferFor(SHOOT_TIME_MS);
                    state = State.SHOOT_SECOND_SPIKE;
                }
                break;

            case SHOOT_SECOND_SPIKE:
                if (!robot.isBusy()) {
                    // 12-ball keeps going for a third pickup. 9-ball leaves the zone now.
                    if (isTwelveBallAuto()) {
                        robot.followPathAndIntake(paths.goIntakeThirdSpike);
                        state = State.GO_INTAKE_THIRD_SPIKE;
                    } else {
                        robot.followPath(paths.leaveZone);
                        state = State.LEAVE_ZONE;
                    }
                }
                break;

            case GO_INTAKE_THIRD_SPIKE:
                if (!robot.isBusy()) {
                    robot.followPath(paths.goShootThirdSpike);
                    state = State.GO_SHOOT_THIRD_SPIKE;
                }
                break;

            case GO_SHOOT_THIRD_SPIKE:
                if (readyToShoot()) {
                    robot.transferFor(SHOOT_TIME_MS);
                    state = State.SHOOT_THIRD_SPIKE;
                }
                break;

            case SHOOT_THIRD_SPIKE:
                if (!robot.isBusy()) {
                    robot.followPath(paths.leaveZone);
                    state = State.LEAVE_ZONE;
                }
                break;

            case LEAVE_ZONE:
                if (!robot.isBusy()) {
                    // Cleanly stop follower/intake and mark the routine complete.
                    robot.forceIdle();
                    state = State.DONE;
                }
                break;

            case DONE:
            default:
                break;
        }

        // Extra auto telemetry on top of RobotAuton's telemetry.
        telemetry.addData("Auto", opModeName());
        telemetry.addData("State", state);
        telemetry.addData("Path progress %", "%.1f", robot.getPathProgressPercent());
        telemetry.addData("Shooter mode", robot.isShooterModeEnabled());
        telemetry.addData("Waiting shooter", isWaitingForShooter());
        telemetry.addData("Shooter wait timeout", shooterWaitTimedOut);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save final pose for TeleOp handoff and stop mechanisms safely.
        if (robot != null) {
            robot.savePose();
            robot.forceIdle();
        }
    }

    private Pose alliancePose(Pose bluePose) {
        // Paths are authored once on blue. Red gets the mirrored field position.
        if (isBlueAlliance()) {
            return bluePose;
        }

        Pose redPose = bluePose.mirror();
        if (bluePose == BLUE_SHOOTING_POSE) {
            return new Pose(
                    redPose.getX(),
                    redPose.getY(),
                    ShootingTarget.headingToGoal(
                            redPose.getX(),
                            redPose.getY(),
                            false
                    )
            );
        }

        return redPose;
    }

    private String opModeName() {
        // Build a readable telemetry name from the selected wrapper name.
        String alliance = getClass().getSimpleName().startsWith("Blue") ? "Blue" : "Red";
        String balls = isTwelveBallAuto() ? "12 Ball" : "9 Ball";
        return alliance + " Close Zone " + balls;
    }

    private boolean readyToShoot() {
        if (robot.isBusy()) {
            aimSettleStarted = false;
            shooterWaitStarted = false;
            shooterWaitTimedOut = false;
            return false;
        }

        if (!shooterWaitStarted) {
            shooterWaitTimer.reset();
            shooterWaitStarted = true;
            shooterWaitTimedOut = false;
        }

        if (!aimSettleStarted) {
            aimSettleTimer.reset();
            aimSettleStarted = true;
        }

        // Run live auto aim briefly, but do not block transfer on aim tolerance.
        if (aimSettleTimer.milliseconds() <= RobotConstants.AUTON_AIM_SETTLE_MS) {
            robot.aimAtShootingGoal();
        } else {
            robot.stopAutoAimTurn();
        }

        // Transfer only waits for the shooter RPM to be ready.
        double shooterReadyTimeoutMs = Math.max(0.0, RobotConstants.AUTON_SHOOTER_READY_TIMEOUT_MS);
        shooterWaitTimedOut = shooterWaitTimer.milliseconds() >= shooterReadyTimeoutMs;
        return robot.isShooterReady() || shooterWaitTimedOut;
    }

    private boolean isWaitingForShooter() {
        return robot.isShooterModeEnabled()
                && !robot.isBusy()
                && !robot.isShooterReady()
                && !shooterWaitTimedOut
                && (state == State.GO_SHOOT_PRELOAD
                || state == State.GO_SHOOT_FIRST_SPIKE
                || state == State.GO_SHOOT_SECOND_SPIKE
                || state == State.GO_SHOOT_THIRD_SPIKE);
    }

    private class Paths {
        // Individual chunks make the state machine simple to read.
        private final PathChain goShootPreload;
        private final PathChain goIntakeFirstSpike;
        private final PathChain goShootFirstSpike;
        private final PathChain goIntakeSecondSpike;
        private final PathChain goShootSecondSpike;
        private final PathChain goIntakeThirdSpike;
        private final PathChain goShootThirdSpike;
        private final PathChain leaveZone;
        // Full routine path chain, currently built for reference/tuning but not used by loop().
        private final PathChain mainChain;

        private Paths() {
            // Start to shooting pose for the preload.
            goShootPreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(BLUE_START_POSE),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(BLUE_START_POSE).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .build();

            // Shooting pose to first spike pickup.
            goIntakeFirstSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(38.169, 81.905)),
                            alliancePose(new Pose(17.632, 82.311))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // First spike back to shooting pose.
            goShootFirstSpike = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(17.632, 82.311)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .build();

            // Shooting pose to second spike pickup.
            goIntakeSecondSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(49.244, 55.389)),
                            alliancePose(new Pose(14.063, 54.203))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Second spike back to shooting pose.
            goShootSecondSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(new Pose(14.063, 54.203)),
                            alliancePose(new Pose(46.812, 60.064)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .build();

            // Shooting pose to third spike pickup for the 12-ball routine.
            goIntakeThirdSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(50.394, 31.108)),
                            alliancePose(new Pose(18.812, 34.184))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Third spike back to shooting pose for the 12-ball routine.
            goShootThirdSpike = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(18.812, 34.184)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .build();

            // Final path after the last shooting cycle.
            leaveZone = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(50.021, 119.076))
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(BLUE_SHOOTING_POSE).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading()
                    )
                    .build();

            // Same route as one continuous chain; useful for visualization or future simplification.
            mainChain = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(BLUE_START_POSE),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(BLUE_START_POSE).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(38.169, 81.905)),
                            alliancePose(new Pose(17.632, 82.311))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(17.632, 82.311)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(49.244, 55.389)),
                            alliancePose(new Pose(14.063, 54.203))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierCurve(
                            alliancePose(new Pose(14.063, 54.203)),
                            alliancePose(new Pose(38.812, 64.064)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(50.394, 31.108)),
                            alliancePose(new Pose(18.812, 34.184))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(18.812, 34.184)),
                            alliancePose(BLUE_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(BLUE_SHOOTING_POSE).getHeading()
                    )
                    .addPath(new BezierLine(
                            alliancePose(BLUE_SHOOTING_POSE),
                            alliancePose(new Pose(50.021, 119.076))
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(BLUE_SHOOTING_POSE).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading()
                    )
                    .build();
        }
    }
}
