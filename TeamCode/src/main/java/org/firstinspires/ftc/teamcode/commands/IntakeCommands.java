package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSS;

public class IntakeCommands {

    private final IntakeSS intake;
    private Mode currentMode = Mode.IDLE;

    private enum Mode {
        IDLE,
        INTAKING,
        OUTTAKING,
        OPEN_GATE,
        TRANSFERRING
    }

    public IntakeCommands(IntakeSS intake) {
        this.intake = intake;
    }

    public void update() {
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
        currentMode = mode;
    }
}
