package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.autonHighRpmTol;
import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.rpmTol;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.PIDFSController;
import org.firstinspires.ftc.teamcode.math.ShooterEquation;
import org.firstinspires.ftc.teamcode.subsystems.constant.SS_Interface;

public class ShooterSS implements SS_Interface {

    // Two shooter motors are driven together with the same calculated power.
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    // PIDFS turns target/current RPM into motor power.
    private final PIDFSController pidfsController;

    // Converts distance-to-goal into target RPM.
    private final ShooterEquation shooterEquation;

    // Driver telemetry uses this to say whether the shooter is close enough to fire.
    private boolean shooterReady = false;

    // Cached motor power. write() sends it to both motors.
    private double mPow = 0.0;
    private double targetSpeed = 0.0;
    private double currentRPM = 0.0;

    // Driver-adjusted correction added to the equation RPM.
    private double rpmOffset = 0.0;

    public ShooterSS(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftMotor = hwm.get(DcMotorEx.class, "Shooter1");
        rightMotor = hwm.get(DcMotorEx.class, "Shooter2");

        // Motors face opposite directions, so one is reversed to spin the wheels together.
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Float lets shooter wheels coast down instead of braking hard.
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfsController = new PIDFSController(hwm);
        shooterEquation = new ShooterEquation();
    }

    @Override
    public void read() {
        // Convert encoder ticks per second to RPM. Abs handles reversed encoder signs.
        currentRPM = Math.abs(leftMotor.getVelocity() * 60.0 / 28.0);
    }

    @Override
    public void write() {
        // Both shooter motors get the same power command.
        leftMotor.setPower(mPow);
        rightMotor.setPower(mPow);
    }

    @Override
    public void telemetryUpdate() {
        telemetry.addData("target rpm", targetSpeed);
        telemetry.addData("current rpm", currentRPM);
        telemetry.addData("shooter power", mPow);
    }

    public void update(double distance) {
        // Active shooter loop: read speed, calculate target, calculate power, then write motors.
        read();
        telemetryUpdate();

        // Base RPM comes from distance. D-pad offset lets the driver tune shots live.
        targetSpeed = shooterEquation.getTargetRPM(distance) + rpmOffset;

        mPow = pidfsController.calculate(targetSpeed, currentRPM);
        // Ready means current RPM is inside the acceptable tolerance window.
        shooterReady = currentRPM > targetSpeed - rpmTol && currentRPM < targetSpeed + rpmTol / 3.0;
        write();
    }

    public void activateShooter(double distance, boolean active) {
        if (active) {
            // TeleOp passes true while shooter mode is toggled on.
            update(distance);
        } else {
            // Releasing A shuts shooter output and readiness off.
            mPow = 0.0;
            targetSpeed = 0.0;
            shooterReady = false;
            write();
        }
    }

    public void runToTargetRPM(double targetRPM) {
        // Tuning mode bypasses the distance equation and drives directly to the requested RPM.
        read();
        targetSpeed = targetRPM;
        mPow = pidfsController.calculate(targetSpeed, currentRPM);
        shooterReady = currentRPM > targetSpeed - rpmTol && currentRPM < targetSpeed + rpmTol / 3.0;
        write();
    }

    public void stopShooter() {
        // Shared hard stop for simple tuning opmodes.
        mPow = 0.0;
        targetSpeed = 0.0;
        shooterReady = false;
        write();
    }

    public boolean isReady() {
        return shooterReady;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public boolean isReadyForAuton() {
        // Auto can overshoot while waiting; use a wider high-side window so it does not get stuck.
        return targetSpeed > 0.0
                && currentRPM > targetSpeed - rpmTol
                && currentRPM < targetSpeed + autonHighRpmTol;
    }

    public void setRpmOffset(double rpmOffset) {
        this.rpmOffset = rpmOffset;
    }
}
