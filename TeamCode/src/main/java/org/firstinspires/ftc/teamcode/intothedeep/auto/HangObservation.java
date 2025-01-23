package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side scoring Auto
 */
@Autonomous(name = "Observation Auto 9808", group = "Auto")
public class HangObservation extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
    private int hangingCount = 0;

    private final double FIRST_HANG_Y_OFFSET = 4.0;//6.0;

    private final double TO_FC_X_OFFSET = 8.0;
    private final double TO_FC_Y_OFFSET = 78.0;

    private Pose2DGobilda hangPos;

    protected void wallPickup() {
        driveBase.driveTo( .3, 17.5, 116, 180); //x14
        pickupSpecimenFromWall();
//        sleep(100);
        closeGripper();
        sleep(100);
        travel();
        closeGripper();
    }

    protected void scoreSpecimen() {
        //driveBase.driveTo(.3, 32 + TO_FC_X_OFFSET, (hangingCount * 2.5) + -FIRST_HANG_Y_OFFSET + TO_FC_Y_OFFSET, 0.0);
        driveBase.driveTo( .5, hangPos);
        hangSpecimen();
        hangPos = new Pose2DGobilda(
                DistanceUnit.INCH,hangPos.getX(DistanceUnit.INCH), hangPos.getY(DistanceUnit.INCH) + 1 - (hangingCount * 1.5),
                AngleUnit.DEGREES, hangPos.getHeading(AngleUnit.DEGREES));
        hangingCount += 1;
    }

    protected void run_auto() {

        double headingThreshold = 15;
        double farX = 50;
        hangPos = new Pose2DGobilda(
                DistanceUnit.INCH, 32 + TO_FC_X_OFFSET, TO_FC_Y_OFFSET - FIRST_HANG_Y_OFFSET - 2,
                AngleUnit.DEGREES, 0);
        hangingCount = 0;
        diagnosticMode = true;
        travel();
        scoreSpecimen();
        travel();
        driveBase.moveMotor(driveBase.slide, slideIn, 0.5, false);

//        getSpecimen();

        // Back up
        //driveBase.driveTo( .5, 15 + TO_FC_X_OFFSET, 26.5 + TO_FC_Y_OFFSET, 0,2, headingThreshold); //-22.5 y
        //12.5 + TO_FC_X...
        driveBase.tankDrive( .5, -3);
        driveBase.driveTo( .5, 36, 29 + TO_FC_Y_OFFSET, 0, 2, headingThreshold);

        // Get in place
        driveBase.driveTo( .5, farX + TO_FC_X_OFFSET, 29 + TO_FC_Y_OFFSET, 0, 2, headingThreshold);
        driveBase.driveTo( .5, farX + TO_FC_X_OFFSET, 40.5+ TO_FC_Y_OFFSET, 0, 2, headingThreshold);
        //sleep(50);


        // Plow
        driveBase.driveTo( .5, 13 + TO_FC_X_OFFSET, 40.5 + TO_FC_Y_OFFSET, 0);


        // Pickup and score
        driveBase.driveTo( .5, 20 + TO_FC_X_OFFSET, 43 + TO_FC_Y_OFFSET, 90, 1, headingThreshold); //y 53 10:14 1/18
        wallPickup();
        driveBase.driveTo(.5, 28 + TO_FC_X_OFFSET, (hangingCount * 2.5) - FIRST_HANG_Y_OFFSET + TO_FC_Y_OFFSET, 0.0, 5, 5);
        scoreSpecimen();
        travel();

        driveBase.driveTo( .5, 20, 116, 180, 3, headingThreshold); //x14
        wallPickup();
        driveBase.driveTo(.5, 28 + TO_FC_X_OFFSET, (hangingCount * 2.5) - FIRST_HANG_Y_OFFSET + TO_FC_Y_OFFSET, 0.0, 5, 5);
        scoreSpecimen();
        travel();

        driveBase.moveMotor(driveBase.slide, slideIn, .5,false);
        driveBase.driveTo( 0.5, new Pose2DGobilda(DistanceUnit.INCH, 15, 105, AngleUnit.DEGREES, 0));

        telemetry.addData( "Runtime", getRuntime());
        telemetry.update();
    }
}