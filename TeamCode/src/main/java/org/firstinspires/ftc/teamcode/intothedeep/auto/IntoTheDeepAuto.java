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
            displayDiagnostics();
            // Call the run code for the specific opmode

//          Prepare To Move
            sweeperIn();
            driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.8, false);
            sweeperOff();

//          Drive Off Wall
            driveBase.tankDrive(.5, 5);
            driveBase.driveSidewaysUntil(.5, 9, false);
            score(Basket.TOP); // Loaded

//          First Sample
            autoSamples(25.0);

//          Second Sample
            autoSamples(16.0);       //Time in middle of this.


//            //Move to Move Sample
//            driveBase.moveMotor(driveBase.arm, armCollectPositionUp, .4, false);
//            sleep(50);
//            driveBase.tankDrive(.5, driveBase.frontDistanceToWall()-8);
//            sleep(50);
//            driveBase.gyroTurnWait(.5,90);
//            driveBase.driveSideways(.5, 38 - driveBase.leftDistanceToWall());
//            sleep(50);
//            driveBase.gyroTurnWait(.5,90);
//
//            sweeperIn();
//            driveBase.tankDrive(.5, driveBase.frontDistanceToWall()-5);
//            driveBase.moveMotor(driveBase.arm, armCollectPositionMat, .2, false);
//            while (driveBase.arm.isBusy());
//            sleep(500);
//            sweeperOff();
//
//            travel();
//            driveBase.driveSideways(.5, -24);
//            driveBase.tankDrive(.5, 10);
//            score(Basket.TOP); // Third Pickup
//
//            //Park
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
