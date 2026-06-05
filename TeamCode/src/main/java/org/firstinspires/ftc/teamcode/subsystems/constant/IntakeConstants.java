package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    // Back roller current above this value means the intake should stop that roller.
    public static double currentLimit = 2000;

    // Normal intake motor speed.
    public static double intakeSpeed = 1.0;
    // Speed used when transferring game pieces forward.
    public static double transferSpeed = 1.0;
    // Servo position that opens the intake gate.
    public static double openPos = 0.6;
    // Servo position that closes the intake gate.
    public static double closePos = 0.0;
    // Wait time after moving the gate before rollers start.
    public static double gateSettleTime = 150.0;
    // Transfer opens the gate earlier and waits this long before running rollers.
    public static double transferGateSettleTime = 750.0;
}
