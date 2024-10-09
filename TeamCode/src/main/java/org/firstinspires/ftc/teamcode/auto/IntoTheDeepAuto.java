package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

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
