package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 12 Ball", group = "Auto")
public class BlueCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Blue alliance uses blue goal data and the base blue path directly.
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // true means run the longer routine that collects the third spike set.
        return true;
    }
}
