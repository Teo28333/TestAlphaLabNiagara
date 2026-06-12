package org.firstinspires.ftc.teamcode.math;

public class ShooterEquation {
    private static final double MAX_RPM = 6832.09085;
    private static final double DISTANCE_COEFFICIENT = 0.0101303;
    private static final double OFFSET = -0.982773;

    public double getTargetRPM(double distance) {
        // Logistic fit from testing: input is distance to goal, output is shooter RPM.
        return MAX_RPM / (1.0 + Math.exp(-(DISTANCE_COEFFICIENT * distance + OFFSET)));
    }
}
