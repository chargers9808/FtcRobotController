package org.firstinspires.ftc.deprecated.centerstage.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.deprecated.centerstage.auto.position.BackdropBase;
import org.firstinspires.ftc.deprecated.centerstage.auto.position.PositionBase;

@Autonomous(name = "Blue Backdrop (State)", group = "Auto")
@Disabled
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
