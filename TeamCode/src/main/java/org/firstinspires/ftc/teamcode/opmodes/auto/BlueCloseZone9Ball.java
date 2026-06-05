package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 9 Ball", group = "Auto")
public class BlueCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        return false;
    }
}
