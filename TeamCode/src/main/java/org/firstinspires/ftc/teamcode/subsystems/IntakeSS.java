package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.closePos;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.currentLimit;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.gateSettleTime;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.intakeSpeed;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.openPos;
import static org.firstinspires.ftc.teamcode.subsystems.constant.IntakeConstants.transferGateSettleTime;
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

    // Front rollers and back roller are separate so the back roller can stop on high current.
    private final DcMotorEx frontRollers;
    private final DcMotorEx backRoller;
    // Gate servo chooses whether game pieces are blocked or allowed through.
    private final ServoImplEx gate;
    private final Telemetry telemetry;
    // Used to wait briefly after opening/closing the gate before spinning rollers.
    private final ElapsedTime gateTimer = new ElapsedTime();

    // Cached motor powers and servo position. write() sends these to hardware.
    private double mPow1 = 0.0;
    private double mPow2 = 0.0;
    private double gatePos = closePos;
    // Back roller current is used as a simple load/jam detection signal.
    private double backRollerCurrent = 0.0;
    // Once current has tripped during intake, keep the back roller stopped.
    private boolean wasTriggered;
    // These flags make sure the gate settle timer starts only once per action.
    private boolean gateClosingForIntake = false;
    private boolean gateOpeningForTransfer = false;

    public IntakeSS(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        frontRollers = hwm.get(DcMotorEx.class, "frontRollers");
        backRoller = hwm.get(DcMotorEx.class, "backRoller");
        gate = hwm.get(ServoImplEx.class, "gate");

        // Directions make positive power mean "run intake forward" for this robot.
        frontRollers.setDirection(DcMotorSimple.Direction.FORWARD);
        backRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake keeps rollers from coasting when the intake is stopped.
        frontRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void read() {
        // Read current draw in milliamps so intake can detect a loaded/stalled back roller.
        backRollerCurrent = backRoller.getCurrent(CurrentUnit.MILLIAMPS);
    }

    @Override
    public void write() {
        // This is the only place this subsystem sends cached outputs to hardware.
        frontRollers.setPower(mPow1);
        backRoller.setPower(mPow2);
        gate.setPosition(gatePos);
    }

    @Override
    public void telemetryUpdate() {
        telemetry.addData("back roller current", backRollerCurrent);
    }

    public void activateIntake() {
        // Intake closes the gate first, waits, then starts the rollers.
        read();
        telemetryUpdate();
        closeGate();
        if (!gateClosingForIntake) {
            gateTimer.reset();
            gateClosingForIntake = true;
        }
        gateOpeningForTransfer = false;

        // Do not spin rollers until the gate has had time to physically move.
        if (gateTimer.milliseconds() < gateSettleTime) {
            stopMotors();
            write();
            return;
        }

        mPow1 = intakeSpeed;
        // High current probably means a game piece is loaded or the back roller is stalled.
        if (backRollerCurrent > currentLimit) {
            wasTriggered = true;
        }

        // Stop only the back roller after current trips; keep the front rollers pulling.
        if (wasTriggered) {
            mPow2 = 0.0;
        } else {
            mPow2 = intakeSpeed;
        }
        write();
    }

    public void activateOuttake() {
        // Outtake opens the gate and runs both rollers backward at half intake speed.
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
        // Transfer opens the gate, waits for it to move, then feeds forward.
        read();
        telemetryUpdate();
        wasTriggered = false;
        openGate();
        if (!gateOpeningForTransfer) {
            gateTimer.reset();
            gateOpeningForTransfer = true;
        }
        gateClosingForIntake = false;

        // Do not feed until the gate has had time to open.
        if (gateTimer.milliseconds() < transferGateSettleTime) {
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
        // Open gate only: useful as a command mode without roller movement.
        openGate();
        stopMotors();
        gateClosingForIntake = false;
        gateOpeningForTransfer = true;
        write();
    }

    public void stop() {
        // Safe default: clear trigger state, close the gate, and stop both rollers.
        wasTriggered = false;
        closeGate();
        stopMotors();
        resetGateTiming();
        write();
    }

    public void resetIntake() {
        // Called when intake is started again, so old current trips do not carry over.
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
        // Next intake/transfer action should restart the gate wait timer.
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        gateTimer.reset();
    }

    private void stopMotors() {
        // Cache zero power for both intake motors.
        mPow1 = 0.0;
        mPow2 = 0.0;
    }
}
