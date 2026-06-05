package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close Zone 9 Ball", group = "Auto")
public class BlueCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Blue alliance uses blue goal data and the base blue path directly.
        return true;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // false stops after the second spike set, making this the shorter 9-ball auto.
        return false;
    }
}
