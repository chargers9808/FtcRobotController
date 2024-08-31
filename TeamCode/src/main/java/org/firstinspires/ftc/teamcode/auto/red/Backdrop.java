package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.position.BackdropBase;
import org.firstinspires.ftc.teamcode.auto.position.PositionBase;

@Autonomous(name = "Red Backdrop (State)", group = "Auto")
public class Backdrop extends RedAuto {
    private final BackdropBase positionBase = new BackdropBase();
    protected PositionBase getPositionBase() { return positionBase; }

    protected void plow( String propLocation ) {
        switch( propLocation) {
            case "Left":
                driveBase.plowFromRedRightStartToLeftSpike();
                break;
            case "Right":
                driveBase.plowFromRedRightStartToRightSpike();
                break;
            case "Center":
                driveBase.plowFromRedRightStartToCenterSpike();
                break;
        }
    }
}
