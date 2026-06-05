package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    // Hardware map names for the four drivetrain motors.
    public static final String FRONT_LEFT = "FL";
    public static final String FRONT_RIGHT = "FR";
    public static final String BACK_LEFT = "BL";
    public static final String BACK_RIGHT = "BR";

    // Blue is rotated 180 degrees for field-centric driving because it starts opposite red.
    public static double FIELD_CENTRIC_OFFSET_BLUE_DEG = 180.0;
    // Red field-centric driving uses the normal field frame.
    public static double FIELD_CENTRIC_OFFSET_RED_DEG = 0.0;
    // Manual turn input multiplier. Lower means gentler driver turning.
    public static double TURN_MULTIPLIER = 0.5;
    // One D-pad press changes shooter target by this many RPM.
    public static double SHOOTER_RPM_ADJUST_STEP = 50.0;
    // Driver/operator RPM trim is clamped to plus or minus this value.
    public static double SHOOTER_RPM_OFFSET_LIMIT = 200.0;
    // Maximum turn power heading lock is allowed to command.
    public static double HEADING_LOCK_MAX_TURN_POWER = 1.0;
    // Minimum delay between telemetry packets sent to Driver Station.
    public static int TELEMETRY_INTERVAL_MS = 100;

    // Red alliance fallback start pose if no saved autonomous pose exists.
    public static final double START_X_RED = 8.25;
    public static final double START_Y_RED = 8.5;
    public static final double START_H_RED = 0.0;

    // Blue alliance fallback start pose if no saved autonomous pose exists.
    public static final double START_X_BLUE = 135.75;
    public static final double START_Y_BLUE = 8.5;
    public static final double START_H_BLUE = Math.PI;

    // Goal coordinates used for heading lock and distance-based shooter RPM.
    public static double SHOOTING_GOAL_X_RED = 139.5;
    public static double SHOOTING_GOAL_Y_RED = 142.0;
    public static double SHOOTING_GOAL_X_BLUE = 2.0;
    public static double SHOOTING_GOAL_Y_BLUE = 142.0;
}
