package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Red", group = "TeleOp")
public class Red extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        // false means this TeleOp uses the red alliance field offsets and shooting goal.
        robot = new Robot(hardwareMap, telemetry, false);
    }

    @Override
    public void start() {
        // Give the shared Robot class a chance to set pose and enable teleop driving.
        robot.start();
    }

    @Override
    public void loop() {
        // All real TeleOp behavior is inside Robot.update(gamepad1).
        robot.update(gamepad1);
        // Push all telemetry lines added during this loop to the Driver Station.
        telemetry.update();
    }

    @Override
    public void stop() {
        // Shut down shooter/intake safely when the Driver Station stops TeleOp.
        if (robot != null) {
            robot.stop();
        }
    }
}
