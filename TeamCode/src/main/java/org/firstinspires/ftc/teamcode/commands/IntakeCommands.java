package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

public class IntakeCommands {

    // This class does not touch hardware directly. It only decides which intake mode should run.
    private final IntakeSS intake;
    private Mode currentMode = Mode.IDLE;

    private enum Mode {
        // Motors off, gate closed.
        IDLE,
        // Bring game pieces into the robot.
        INTAKING,
        // Push game pieces back out of the intake.
        OUTTAKING,
        // Open the gate without running rollers.
        OPEN_GATE,
        // Move game pieces from intake toward the shooter/indexer path.
        TRANSFERRING
    }

    public IntakeCommands(IntakeSS intake) {
        this.intake = intake;
    }

    public void update() {
        // Run exactly one intake behavior based on the current state.
        switch (currentMode) {
            case INTAKING:
                intake.intakeCMD();
                break;

            case OUTTAKING:
                intake.outtakeCMD();
                break;

            case OPEN_GATE:
                intake.openGateCMD();
                break;

            case TRANSFERRING:
                intake.transferCMD();
                break;

            case IDLE:
            default:
                intake.stop();
                break;
        }
    }

    public void intake() {
        // Starting intake fresh clears the old current-trigger and gate timers.
        if (currentMode != Mode.INTAKING) {
            intake.resetIntake();
        }
        setMode(Mode.INTAKING);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING);
    }

    public void openGate() {
        setMode(Mode.OPEN_GATE);
    }

    public void transfer() {
        setMode(Mode.TRANSFERRING);
    }

    public void idle() {
        setMode(Mode.IDLE);
    }

    public boolean isIntaking() {
        return currentMode == Mode.INTAKING;
    }

    public boolean isOuttaking() {
        return currentMode == Mode.OUTTAKING;
    }

    public boolean isOpeningGate() {
        return currentMode == Mode.OPEN_GATE;
    }

    public boolean isTransferring() {
        return currentMode == Mode.TRANSFERRING;
    }

    public String getState() {
        return currentMode.toString();
    }

    private void setMode(Mode mode) {
        // All state changes pass through here, making the current mode easy to track.
        currentMode = mode;
    }
}
