package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystems.constant.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class PIDFSController {

    // Battery voltage sensor used for voltage compensation.
    private final VoltageSensor voltageSensor;

    // PID state remembered between calls.
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = -1L;

    public PIDFSController(HardwareMap hardwareMap) {
        // FTC exposes voltage sensors as an iterable; use the first one available.
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double calculate(double targetVel, double currentVel) {
        // Compute elapsed time so integral and derivative are scaled correctly.
        long nowNs = System.nanoTime();

        // First call only seeds controller state; no output yet.
        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            lastError = targetVel - currentVel;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;
        // Avoid divide-by-zero or bad timestamps.
        if (dt <= 0) return 0.0;

        // If target is zero, reset the controller and command no power.
        if (targetVel == 0.0) {
            reset();
            return 0.0;
        }

        double error = targetVel - currentVel;
        double derivative = (error - lastError) / dt;

        // Anti-windup: only integrate while proportional output is not already saturated.
        if (Math.abs(kP * error) < 1.0) {
            integral = clamp(integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        // Scale feedforward when battery voltage is below/above nominal tuning voltage.
        double voltage = voltageSensor.getVoltage();
        double voltageScale = voltage > 0.1 ? nominalVoltage / voltage : 1.0;

        // kF is velocity feedforward; kS is static friction feedforward.
        double ff = kF * targetVel * voltageScale;
        double ks = Math.copySign(kS, targetVel);

        // Full PIDFS output before clamping to legal motor power.
        double output = kP * error
                + kI * integral
                + kD * derivative
                + ff
                + ks;

        lastError = error;
        return clamp(output, -1.0, 1.0);
    }

    public void reset() {
        // Clear memory so the next calculate() starts fresh.
        integral = 0.0;
        lastError = 0.0;
        lastTimeNs = -1L;
    }

    private static double clamp(double value, double min, double max) {
        // Keep value inside the requested range.
        return Math.max(min, Math.min(max, value));
    }
}
