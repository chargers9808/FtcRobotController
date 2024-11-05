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
        HeadingHolder.setHeading(0.0);
    }

    /**
     * Run AUTO opmode
     */
    @Override
    protected void run_9808() {
        if( opModeIsActive() ) {
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
            autoSamples(23.0, 8.0);

//          Second Sample
            autoSamples(14.0, 7.0);       //Time in middle of this.

            driveBase.tankDrive(.5, -3);
//          Park
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
