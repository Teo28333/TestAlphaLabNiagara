package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 12 Ball", group = "Auto")
public class BlueCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Names were inverted on the robot; this Blue opmode uses the base/red path set.
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // true means run the longer routine that collects the third spike set.
        return true;
    }
}
