package org.firstinspires.ftc.teamcode.math;

public class ShooterEquation {
    public double getTargetRPM(double distance) {
        return  -0.0000114126 * Math.pow(distance, 4)
                + 0.00518183   * Math.pow(distance, 3)
                - 0.846593     * Math.pow(distance, 2)
                + 73.04495     * distance
                + 625.72007;
    }
}
