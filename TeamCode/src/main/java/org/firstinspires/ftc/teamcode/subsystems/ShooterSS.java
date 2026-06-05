package org.firstinspires.ftc.teamcode.subsystems;

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

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;
    private final PIDFSController pidfsController;
    private final ShooterEquation shooterEquation;

    private boolean shooterReady = false;
    private double mPow = 0.0;
    private double targetSpeed = 0.0;
    private double currentRPM = 0.0;
    private double rpmOffset = 0.0;

    public ShooterSS(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftMotor = hwm.get(DcMotorEx.class, "Shooter1");
        rightMotor = hwm.get(DcMotorEx.class, "Shooter2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidfsController = new PIDFSController(hwm);
        shooterEquation = new ShooterEquation();
    }

    @Override
    public void read() {
        currentRPM = leftMotor.getVelocity() * 60.0 / 28.0;
    }

    @Override
    public void write() {
        leftMotor.setPower(mPow);
        rightMotor.setPower(mPow);
    }

    @Override
    public void telemetryUpdate() {
        telemetry.addData("target rpm", targetSpeed);
        telemetry.addData("current rpm", currentRPM);
    }

    public void update(double distance) {
        read();
        telemetryUpdate();

        targetSpeed = shooterEquation.getTargetRPM(distance) + rpmOffset;

        mPow = pidfsController.calculate(targetSpeed, currentRPM);
        shooterReady = currentRPM > targetSpeed - rpmTol && currentRPM < targetSpeed + rpmTol / 3.0;
        write();
    }

    public void activateShooter(double distance, boolean active) {
        if (active) {
            update(distance);
        } else {
            mPow = 0.0;
            targetSpeed = 0.0;
            shooterReady = false;
            write();
        }
    }

    public boolean isReady() {
        return shooterReady;
    }

    public void setRpmOffset(double rpmOffset) {
        this.rpmOffset = rpmOffset;
    }
}
