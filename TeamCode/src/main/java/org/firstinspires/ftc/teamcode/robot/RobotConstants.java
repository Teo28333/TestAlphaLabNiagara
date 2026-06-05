package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";

    public static double FIELD_CENTRIC_OFFSET_BLUE_DEG = 180.0;
    public static double FIELD_CENTRIC_OFFSET_RED_DEG = 0.0;
    public static double TURN_MULTIPLIER = 0.5;
    public static double SHOOTER_RPM_ADJUST_STEP = 50.0;
    public static double HEADING_LOCK_MAX_TURN_POWER = 1.0;
    public static int TELEMETRY_INTERVAL_MS = 100;

    public static final double START_X_RED = 8.25;
    public static final double START_Y_RED = 8.5;
    public static final double START_H_RED = 0.0;

    public static final double START_X_BLUE = 135.75;
    public static final double START_Y_BLUE = 8.5;
    public static final double START_H_BLUE = Math.PI;

    public static double SHOOTING_GOAL_X_RED = 139.5;
    public static double SHOOTING_GOAL_Y_RED = 142.0;
    public static double SHOOTING_GOAL_X_BLUE = 2.0;
    public static double SHOOTING_GOAL_Y_BLUE = 142.0;
}
