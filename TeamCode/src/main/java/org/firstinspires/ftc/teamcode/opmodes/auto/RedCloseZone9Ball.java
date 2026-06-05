package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 9 Ball", group = "Auto")
public class RedCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        return false;
    }
}
