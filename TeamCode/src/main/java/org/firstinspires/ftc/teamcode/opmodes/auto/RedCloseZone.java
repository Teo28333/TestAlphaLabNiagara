package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 12 Ball", group = "Auto")
public class RedCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Red alliance uses red goal data; the base blue path is mirrored in CloseZoneAuto.
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // true means run the longer routine that collects the third spike set.
        return true;
    }
}
