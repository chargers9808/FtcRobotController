package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side scoring Auto
 */
@Autonomous(name = "Score Observation", group = "Auto")
public class HangObservation extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
    private int hangingCount = 0;

    private final double FIRST_HANG_Y_OFFSET = 6.0;
    private final double PLOW_START_X = 1;
    private final double PLOW_DEST_X = 10.5;

    protected void getSpecimen() {
        driveBase.driveTo(.3, 2.5, -30.5, 270.0);
        pickupSpecimen( false );
        closeGripper();
        sleep(300);
        travel();
    }

    protected void scoreSpecimen() {
        driveBase.driveTo(.3, 32, (hangingCount * -2.5) + FIRST_HANG_Y_OFFSET, 0.0);
        hangSpecimen();
        hangingCount += 1;
    }

    protected void plow( double x ) {
        driveBase.driveTo( 0.5, 25.6, -29.3, 0);
        //driveBase.driveTo( 0.5, x, -40.5, 0;
    }

    protected void run_auto() {
        hangingCount = 0;
        diagnosticMode = true;
        travel();
        scoreSpecimen();
        driveBase.moveMotor(driveBase.slide, slideIn, 0.5, false);
        getSpecimen();
        scoreSpecimen();
        travel();

        double headingThreshold = 7;
        double farX = 50;

        // Back up
        driveBase.driveTo( .5, 12.5, -24.5, 0,2, headingThreshold); //-22.5 y
        driveBase.setLED(DraculaBase.LEDColor.RED);

        // Stage
        //driveBase.driveTo( .5, 25.6, -29.3, 0, 2, headingThreshold);
        //sleep(50);

        // Get in place
        driveBase.driveTo( .5, farX, -29.3, 0, 2, headingThreshold);
        driveBase.moveMotor(driveBase.arm, armCollectPositionUp, 0.5, false);
        driveBase.driveTo( .5, farX, -40.5, 0, 2, headingThreshold);
        //sleep(50);

        // Plow
        driveBase.driveTo( .5, 13, -40.5, 0);

        // 2nd ....
        driveBase.driveTo( .5, farX, -40.5, 90, 2, headingThreshold);
        driveBase.driveTo( .5, farX, -48, 90, 1, headingThreshold);

        // Plow
        driveBase.driveTo( .5, 15, -53, 90, 1, headingThreshold);
        //sleep(100);
        //driveBase.driveTo( .2, 9, -46.7, 125);
        //sleep(100);
        driveBase.gyroTurn(0.5, 125);
        driveBase.tankDrive(0.2, 4);
        pickupSpecimen( false );
        closeGripper();
        sleep(175);
        travel();
        driveBase.driveTo(.5, 28, (hangingCount * -2.5) + FIRST_HANG_Y_OFFSET, 0.0, 5, 5);

        scoreSpecimen();
        driveBase.tankDrive(0.8, -5);

        telemetry.addData( "Runtime", getRuntime());
        telemetry.update();

        // Spin to pick up
        //driveBase.driveTo( 0.4, 15, -40.5, 180);
        //driveBase.driveTo( 0.2, 12.8, -40.5, 180);

//        50 -40.5
//                52 -52
        // 13 -52 0
        // 15 -52 180
        // 15 40.5
        // 12.8 -40.5
    }
}