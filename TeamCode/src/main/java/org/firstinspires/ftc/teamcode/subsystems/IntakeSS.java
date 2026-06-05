package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.currentLimit;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.gateSettleTime;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.intakeSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.openPos;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.transferSpeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.constant.SS_Interface;

public class IntakeSS implements SS_Interface {

    private final DcMotorEx frontRollers;
    private final DcMotorEx backRoller;
    private final ServoImplEx gate;
    private final Telemetry telemetry;
    private final ElapsedTime gateTimer = new ElapsedTime();

    private double mPow1 = 0.0;
    private double mPow2 = 0.0;
    private double gatePos = closePos;
    private double backRollerCurrent = 0.0;
    private boolean wasTriggered;
    private boolean gateClosingForIntake = false;
    private boolean gateOpeningForTransfer = false;

    public IntakeSS(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontRollers = hwm.get(DcMotorEx.class, "frontRollers");
        backRoller = hwm.get(DcMotorEx.class, "backRoller");
        gate = hwm.get(ServoImplEx.class, "gate");

        frontRollers.setDirection(DcMotorSimple.Direction.REVERSE);
        backRoller.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void read() {
        backRollerCurrent = backRoller.getCurrent(CurrentUnit.MILLIAMPS);
    }

    @Override
    public void write() {
        frontRollers.setPower(mPow1);
        backRoller.setPower(mPow2);
        gate.setPosition(gatePos);
    }

    @Override
    public void telemetryUpdate() {
        telemetry.addData("back roller current", backRollerCurrent);
    }

    public void activateIntake() {
        read();
        telemetryUpdate();
        closeGate();
        if (!gateClosingForIntake) {
            gateTimer.reset();
            gateClosingForIntake = true;
        }
        gateOpeningForTransfer = false;

        if (gateTimer.milliseconds() < gateSettleTime) {
            stopMotors();
            write();
            return;
        }

        mPow1 = intakeSpeed;
        if (backRollerCurrent > currentLimit) {
            wasTriggered = true;
        }

        if (wasTriggered) {
            mPow2 = 0.0;
        } else {
            mPow2 = intakeSpeed;
        }
        write();
    }

    public void activateOuttake() {
        read();
        telemetryUpdate();
        wasTriggered = false;
        resetGateTiming();
        openGate();
        mPow1 = -intakeSpeed / 2.0;
        mPow2 = -intakeSpeed / 2.0;
        write();
    }

    public void activateTransfer() {
        read();
        telemetryUpdate();
        wasTriggered = false;
        openGate();
        if (!gateOpeningForTransfer) {
            gateTimer.reset();
            gateOpeningForTransfer = true;
        }
        gateClosingForIntake = false;

        if (gateTimer.milliseconds() < gateSettleTime) {
            stopMotors();
            write();
            return;
        }

        mPow1 = transferSpeed;
        mPow2 = transferSpeed;
        write();
    }

    public void intakeCMD() {
        activateIntake();
    }

    public void outtakeCMD() {
        activateOuttake();
    }

    public void transferCMD() {
        activateTransfer();
    }

    public void openGateCMD() {
        openGate();
        stopMotors();
        gateClosingForIntake = false;
        gateOpeningForTransfer = true;
        write();
    }

    public void stop() {
        wasTriggered = false;
        closeGate();
        stopMotors();
        resetGateTiming();
        write();
    }

    public void resetIntake() {
        wasTriggered = false;
        resetGateTiming();
    }

    public void openGate() {
        gatePos = openPos;
    }

    public void closeGate() {
        gatePos = closePos;
    }

    private void resetGateTiming() {
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        gateTimer.reset();
    }

    private void stopMotors() {
        mPow1 = 0.0;
        mPow2 = 0.0;
    }
}
