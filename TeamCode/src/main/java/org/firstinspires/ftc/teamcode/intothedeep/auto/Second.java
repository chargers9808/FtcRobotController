package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Pickup One (Second) 9808", group = "Auto")
public class Second extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
        sweeperIn();
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.8, false);
        sweeperOff();

//      Drive Off Wall
        driveBase.tankDrive(.5, 5);
        driveBase.driveSidewaysUntil(.5, 9, false);
        score(Basket.TOP); // Loaded

//      Second Sample
        autoSamples(sample2Offset, forwardDistance, 39.0);

        driveBase.tankDriveUntil(.5, sample1Offset-2, true, false);
        driveBase.gyroTurn(.5,0);
        driveBase.tankDrive(.5, 38);
        driveBase.gyroTurn(.5, 270);
        driveBase.tankDrive(.5, 8);
    }
}
