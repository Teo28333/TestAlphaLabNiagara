package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 9 Ball", group = "Auto")
public class BlueCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Names were inverted on the robot; this Blue opmode uses the base/red path set.
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // false stops after the second spike set, making this the shorter 9-ball auto.
        return false;
    }
}
