package org.firstinspires.ftc.deprecated.centerstage.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.deprecated.centerstage.auto.position.BackdropBase;
import org.firstinspires.ftc.deprecated.centerstage.auto.position.PositionBase;

@Autonomous(name = "Blue Backdrop (State)", group = "Auto")
public class Backdrop extends CenterstageBlueAuto {
    private final BackdropBase positionBase = new BackdropBase();
    protected PositionBase getPositionBase() { return positionBase; }

    protected void plow( String propLocation ) {
        switch( propLocation) {
            case "Left":
                driveBase.plowFromBlueBackdropStartToLeftSpike();
                break;
            case "Right":
                driveBase.plowFromBlueBackdropStartToRightSpike();
                break;
            case "Center":
                driveBase.plowFromBlueBackdropStartToCenterSpike();
                break;
        }
    }
}
