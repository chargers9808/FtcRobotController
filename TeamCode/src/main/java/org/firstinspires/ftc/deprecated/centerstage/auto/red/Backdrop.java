package org.firstinspires.ftc.deprecated.centerstage.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.deprecated.centerstage.auto.position.BackdropBase;
import org.firstinspires.ftc.deprecated.centerstage.auto.position.PositionBase;

@Autonomous(name = "Red Backdrop (State)", group = "Auto")
public class Backdrop extends CenterstageRedAuto {
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
