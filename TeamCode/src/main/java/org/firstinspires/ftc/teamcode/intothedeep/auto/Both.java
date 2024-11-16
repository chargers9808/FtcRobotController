package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.intothedeep.auto.blue.IntoTheDeepBlueAuto;

/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Pickup Two 9808", group = "Auto")
public class Both extends IntoTheDeepBlueAuto {
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

//      First Sample
        autoSamples(sample1Offset, forwardDistance, 37.5);
//      Second Sample
        autoSamples(sample2Offset, forwardDistance, 37.5);

        driveBase.tankDrive(.5, -4);
        driveBase.stopMotors();
    }
}