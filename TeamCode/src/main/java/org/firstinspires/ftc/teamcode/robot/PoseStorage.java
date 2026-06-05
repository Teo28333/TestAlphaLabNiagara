package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    // FTC fields are 141.5 inches square due to field perimeter, so valid saved poses must stay inside this.
    private static final double FIELD_SIZE = 141.5;

    // Last known robot pose. Starts at field center until auto or teleop updates it.
    public static Pose currentPose = new Pose(141.5 / 2.0, 141.5 / 2.0, 0.0);

    public static void setCurrentPose(Pose pose) {
        // Ignore null poses and copy values so outside code cannot mutate this object later.
        if (pose != null) {
            currentPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        }
    }

    public static boolean hasValidPose() {
        // Convenience check for whether currentPose can safely be reused.
        return isValid(currentPose);
    }

    public static boolean isValid(Pose pose) {
        // Reject null, NaN, and positions outside the field.
        return pose != null
                && !Double.isNaN(pose.getX())
                && !Double.isNaN(pose.getY())
                && !Double.isNaN(pose.getHeading())
                && pose.getX() >= 0.0
                && pose.getX() <= FIELD_SIZE
                && pose.getY() >= 0.0
                && pose.getY() <= FIELD_SIZE;
    }

    public static Pose allianceStartPose(boolean isBlueAlliance) {
        // Return the fallback starting pose for the selected alliance.
        if (isBlueAlliance) {
            return new Pose(RobotConstants.START_X_BLUE, RobotConstants.START_Y_BLUE, RobotConstants.START_H_BLUE);
        }

        return new Pose(RobotConstants.START_X_RED, RobotConstants.START_Y_RED, RobotConstants.START_H_RED);
    }

    public static Pose fieldCenterPose() {
        // Useful reset pose when you want a neutral field location.
        return new Pose(141.5 / 2.0, 141.5 / 2.0, 0.0);
    }
}
