package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShootingTarget;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSS;
import org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants;

import java.util.List;

@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuning extends OpMode {
    // Field center in inches. Heading 0 makes this match red-side field-centric driving.
    private static final Pose STARTING_POSE = new Pose(141.5 / 2.0, 141.5 / 2.0, 0.0);

    private Follower follower;
    private ShooterSS shooter;
    private IntakeCommands intakeCommands;
    private List<LynxModule> controlHubs;
    private boolean lastIntake = false;

    @Override
    public void init() {
        controlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);
        follower.startTeleopDrive();

        shooter = new ShooterSS(hardwareMap, telemetry);
        intakeCommands = new IntakeCommands(new IntakeSS(hardwareMap, telemetry));
    }

    @Override
    public void loop() {
        clearBulkCache();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * RobotConstants.TURN_MULTIPLIER,
                false,
                Math.toRadians(RobotConstants.FIELD_CENTRIC_OFFSET_RED_DEG)
        );
        follower.update();

        shooter.runToTargetRPM(ShooterConstants.tuningRPM);
        updateIntakeCommands();
        intakeCommands.update();

        telemetry.addData("Robot distance to red goal", "%.1f", distanceToRedGoal());
        telemetry.addData("Current speed", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Shooter ready", shooter.isReady());
        telemetry.addData("Intake state", intakeCommands.getState());
        telemetry.update();

        lastIntake = gamepad1.right_bumper;
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stopShooter();
        }
        if (intakeCommands != null) {
            intakeCommands.idle();
            intakeCommands.update();
        }
    }

    private void updateIntakeCommands() {
        if (gamepad1.left_bumper) {
            intakeCommands.transfer();
            return;
        }

        if (intakeCommands.isTransferring()) {
            intakeCommands.idle();
        }

        boolean intakePressed = gamepad1.right_bumper && !lastIntake;
        if (intakePressed) {
            if (intakeCommands.isIntaking()) {
                intakeCommands.idle();
            } else {
                intakeCommands.intake();
            }
        }
    }

    private double distanceToRedGoal() {
        Pose pose = follower.getPose();
        return ShootingTarget.distanceToGoal(pose.getX(), pose.getY(), false);
    }

    private void clearBulkCache() {
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }
}
