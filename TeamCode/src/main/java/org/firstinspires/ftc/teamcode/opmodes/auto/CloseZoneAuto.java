package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.RobotAuton;

abstract class CloseZoneAuto extends OpMode {
    private static final double SHOOT_TIME_MS = 1750.0;
    private static final Pose RED_START_POSE = new Pose(
            19.034629404617252,
            117.86452004860269,
            Math.toRadians(146)
    );
    private static final Pose RED_SHOOTING_POSE = new Pose(49.295, 91.878);

    private RobotAuton robot;
    private Paths paths;
    private State state = State.GO_SHOOT_PRELOAD;

    private enum State {
        GO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        GO_INTAKE_FIRST_SPIKE,
        GO_SHOOT_FIRST_SPIKE,
        SHOOT_FIRST_SPIKE,
        GO_INTAKE_SECOND_SPIKE,
        GO_SHOOT_SECOND_SPIKE,
        SHOOT_SECOND_SPIKE,
        GO_INTAKE_THIRD_SPIKE,
        GO_SHOOT_THIRD_SPIKE,
        SHOOT_THIRD_SPIKE,
        LEAVE_ZONE,
        DONE
    }

    protected abstract boolean isBlueAlliance();
    protected abstract boolean isTwelveBallAuto();

    @Override
    public void init() {
        robot = new RobotAuton(hardwareMap, telemetry, isBlueAlliance());
        paths = new Paths();
        robot.start(alliancePose(RED_START_POSE));

        telemetry.addLine(opModeName() + " ready");
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.GO_SHOOT_PRELOAD;
        robot.enableShooterMode();
        robot.followPath(paths.goShootPreload);
    }

    @Override
    public void loop() {
        robot.update();

        switch (state) {
            case GO_SHOOT_PRELOAD:
                if (!robot.isBusy()) {
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
                if (!robot.isBusy()) {
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
                if (!robot.isBusy()) {
                    robot.transferFor(SHOOT_TIME_MS);
                    state = State.SHOOT_SECOND_SPIKE;
                }
                break;

            case SHOOT_SECOND_SPIKE:
                if (!robot.isBusy()) {
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
                if (!robot.isBusy()) {
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
                    robot.forceIdle();
                    state = State.DONE;
                }
                break;

            case DONE:
            default:
                break;
        }

        telemetry.addData("Auto", opModeName());
        telemetry.addData("State", state);
        telemetry.addData("Path progress %", "%.1f", robot.getPathProgressPercent());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.savePose();
            robot.forceIdle();
        }
    }

    private Pose alliancePose(Pose redPose) {
        if (!isBlueAlliance()) {
            return redPose;
        }

        return redPose.mirror();
    }

    private String opModeName() {
        String alliance = isBlueAlliance() ? "Blue" : "Red";
        String balls = isTwelveBallAuto() ? "12 Ball" : "9 Ball";
        return alliance + " Close Zone " + balls;
    }

    private class Paths {
        private final PathChain goShootPreload;
        private final PathChain goIntakeFirstSpike;
        private final PathChain goShootFirstSpike;
        private final PathChain goIntakeSecondSpike;
        private final PathChain goShootSecondSpike;
        private final PathChain goIntakeThirdSpike;
        private final PathChain goShootThirdSpike;
        private final PathChain leaveZone;
        private final PathChain mainChain;

        private Paths() {
            goShootPreload = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(19.035, 117.865)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(146))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .build();

            goIntakeFirstSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(38.169, 81.905)),
                            alliancePose(new Pose(17.632, 82.311))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            goShootFirstSpike = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(17.632, 82.311)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .build();

            goIntakeSecondSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(49.244, 58.389)),
                            alliancePose(new Pose(14.063, 57.203))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            goShootSecondSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(new Pose(14.063, 57.203)),
                            alliancePose(new Pose(38.812, 67.064)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .build();

            goIntakeThirdSpike = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(50.394, 31.108)),
                            alliancePose(new Pose(12.812, 34.184))
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            goShootThirdSpike = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(12.812, 34.184)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .build();

            leaveZone = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(50.021, 119.076))
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading()
                    )
                    .build();

            mainChain = robot.follower.pathBuilder()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(19.035, 117.865)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(146))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(38.169, 81.905)),
                            alliancePose(new Pose(17.632, 82.311))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(17.632, 82.311)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(49.244, 58.389)),
                            alliancePose(new Pose(14.063, 57.203))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierCurve(
                            alliancePose(new Pose(14.063, 57.203)),
                            alliancePose(new Pose(38.812, 67.064)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .addPath(new BezierCurve(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(50.394, 31.108)),
                            alliancePose(new Pose(12.812, 34.184))
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(
                            alliancePose(new Pose(12.812, 34.184)),
                            alliancePose(RED_SHOOTING_POSE)
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading()
                    )
                    .addPath(new BezierLine(
                            alliancePose(RED_SHOOTING_POSE),
                            alliancePose(new Pose(50.021, 119.076))
                    ))
                    .setLinearHeadingInterpolation(
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(135))).getHeading(),
                            alliancePose(new Pose(0.0, 0.0, Math.toRadians(180))).getHeading()
                    )
                    .build();
        }
    }
}
