package org.firstinspires.ftc.teamcode.subsystems.constant;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    // Proportional gain: reacts to current RPM error.
    public static double kP = 0.0005;
    // Integral gain: reacts to accumulated RPM error over time.
    public static double kI = 0.0;
    // Derivative gain: reacts to how quickly RPM error is changing.
    public static double kD = 0.0;
    // Feedforward gain: baseline power based on target RPM.
    public static double kF = 0.0001725;
    // Static feedforward: small extra power to overcome friction.
    public static double kS = 0.060;
    // Caps integral buildup so the controller does not overcorrect forever.
    public static double MAX_INTEGRAL = 5000.0;
    // Voltage the shooter model is tuned around.
    public static double nominalVoltage = 12.0;
    // RPM window used to decide whether the shooter is ready.
    public static double rpmTol = 75.0;
}
