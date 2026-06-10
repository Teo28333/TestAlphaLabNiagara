package org.firstinspires.ftc.teamcode.robot;

public final class ShootingTarget {
    private ShootingTarget() {
        // Utility class; do not create one.
    }

    public static double goalX(boolean isBlueAlliance) {
        // Both TeleOp and auto must use the exact same goal X for shooter distance.
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_X_BLUE : RobotConstants.SHOOTING_GOAL_X_RED;
    }

    public static double goalY(boolean isBlueAlliance) {
        // Both TeleOp and auto must use the exact same goal Y for shooter distance.
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_Y_BLUE : RobotConstants.SHOOTING_GOAL_Y_RED;
    }

    public static double distanceToGoal(double robotX, double robotY, boolean isBlueAlliance) {
        // Shooter RPM is based only on straight-line field distance to the alliance goal.
        return Math.hypot(goalX(isBlueAlliance) - robotX, goalY(isBlueAlliance) - robotY);
    }

    public static double headingToGoal(double robotX, double robotY, boolean isBlueAlliance) {
        // Heading lock and shooting-pose headings use the same target point as shooter RPM.
        return Math.atan2(goalY(isBlueAlliance) - robotY, goalX(isBlueAlliance) - robotX);
    }
}
