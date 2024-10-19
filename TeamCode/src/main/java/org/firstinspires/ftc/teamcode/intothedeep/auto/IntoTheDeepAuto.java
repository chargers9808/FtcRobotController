package org.firstinspires.ftc.teamcode.intothedeep.auto;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.intothedeep.IntoTheDeepBase;

abstract public class IntoTheDeepAuto extends IntoTheDeepBase {
    //TODO: Defile init operations
    /*
    ============================================================================================
    Init Controls:
        A:                      enableDiagnosticMode()


    ============================================================================================
     */
    abstract protected Position getPosition();
    @Override
    protected void pre_initialize() {

    }

    @Override
    protected void initialize() {
        driveBase.setLED( getPosition().getStaticColor() );
    }

    /**
     * Run AUTO opmode
     */
    @Override
    protected void run_9808() {
        if( opModeIsActive() ) {
            displayDiagnostics();
            // Call the run code for the specific opmode

            //Drive Off Wall
            driveBase.tankDrive(.5, driveBase.rearDistanceToWall()+12);

            //Drive Closer to Left Wall
            driveBase.driveSidewaysUntil(.5, 12, false);

            //Preset to Score
            score(Basket.TOP); // Loaded



            //Move to Move Sample
            driveBase.gyroTurn(.5, 0);
            driveBase.driveSidewaysCorrected(.5, driveBase.leftDistanceToWall()-15, 0);
            driveBase.tankDrive(.5, driveBase.rearDistanceToWall()+30);
            driveBase.driveSideways(.2, 6);
            driveBase.moveMotor(driveBase.arm, armCollectPositonDown, .4, false);
            sweeperIn();
            driveBase.tankDrive(.5, 2);
            travel();
            driveBase.tankDriveUntil(.5, 12, false);

            score(Basket.TOP); // First Pickup


            //Move to Move Sample
            driveBase.gyroTurn(.5, 0);
            driveBase.driveSidewaysCorrected(.5, driveBase.leftDistanceToWall()-9, 0);
            driveBase.tankDrive(.5, driveBase.rearDistanceToWall()+30);
            driveBase.driveSideways(.2, -6);
            driveBase.moveMotor(driveBase.arm, armCollectPositonDown, .4, false);
            sweeperIn();
            driveBase.tankDrive(.5, 2);
            travel();
            driveBase.tankDriveUntil(.5, 12, false);

            score(Basket.TOP); //Second Pickup



            //Move to Move Sample
            driveBase.gyroTurn(.5, 0);
            driveBase.driveSidewaysCorrected(.5, driveBase.leftDistanceToWall()-18, 0);
            driveBase.tankDriveCorrected(.5, driveBase.rearDistanceToWall()+40, 0);
            driveBase.gyroTurn(.5,90);

            driveBase.moveMotor(driveBase.arm, (armCollectPositonDown ), 0.6, true);
            driveBase.moveMotor(driveBase.slide, slideCollectPosition, .8, true);
            sweeperIn();
            driveBase.tankDrive(.2, 2);
            travel();
            driveBase.tankDriveUntil(.5, 12, false);

            score(Basket.TOP); // Third Pickup

            //Park
            finish();
        }
    }

    /**
     * Finalize AUTO phase in prep for Teleop
     */
    private void finish() {
        HeadingHolder.setHeading(driveBase.getFieldHeading());
        setLEDHeartbeat();
        telemetry.addData("Path", "Complete");

        telemetry.update();
        while (opModeIsActive());
    }
}
