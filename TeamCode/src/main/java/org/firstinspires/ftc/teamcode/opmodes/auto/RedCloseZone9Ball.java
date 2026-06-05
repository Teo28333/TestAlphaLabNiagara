package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 9 Ball", group = "Auto")
public class RedCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Names were inverted on the robot; this Red opmode uses the mirrored/blue path set.
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // false stops after the second spike set, making this the shorter 9-ball auto.
        return false;
    }
}
