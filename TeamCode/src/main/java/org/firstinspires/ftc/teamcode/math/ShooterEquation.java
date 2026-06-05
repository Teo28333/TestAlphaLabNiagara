package org.firstinspires.ftc.teamcode.math;

public class ShooterEquation {
    public double getTargetRPM(double distance) {
        // Polynomial fit from testing: input is distance to goal, output is shooter RPM.
        return  -0.0000114126 * Math.pow(distance, 4)
                + 0.00518183   * Math.pow(distance, 3)
                - 0.846593     * Math.pow(distance, 2)
                + 73.04495     * distance
                + 678.72007;
    }
}
