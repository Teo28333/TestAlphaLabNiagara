package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 12 Ball", group = "Auto")
public class BlueCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        return true;
    }
}
