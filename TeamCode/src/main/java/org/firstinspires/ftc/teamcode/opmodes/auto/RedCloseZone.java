package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 12 Ball", group = "Auto")
public class RedCloseZone extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        return true;
    }
}
