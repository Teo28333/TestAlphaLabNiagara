package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 12 Ball", group = "Auto")
public class RedCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Names were inverted on the robot; this Red opmode uses the mirrored/blue path set.
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // true means run the longer routine that collects the third spike set.
        return true;
    }
}
