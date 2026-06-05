package org.firstinspires.ftc.teamcode.subsystems.constant;

public interface SS_Interface {

    // Pull fresh sensor/encoder/current data from hardware into cached variables.
    void read();
    // Push cached motor powers and servo positions out to hardware.
    void write();
    // Add subsystem-specific telemetry lines for the Driver Station.
    void telemetryUpdate();
}
