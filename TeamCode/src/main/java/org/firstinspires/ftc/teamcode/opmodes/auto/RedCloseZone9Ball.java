package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close Zone 9 Ball", group = "Auto")
public class RedCloseZone9Ball extends CloseZoneAuto {
    @Override
    protected boolean isBlueAlliance() {
        // Red alliance uses red goal data; the base blue path is mirrored in CloseZoneAuto.
        return false;
    }

    @Override
    protected boolean isTwelveBallAuto() {
        // false stops after the second spike set, making this the shorter 9-ball auto.
        return false;
    }
}
