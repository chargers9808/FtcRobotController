package org.firstinspires.ftc.teamcode.intothedeep.auto;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.intothedeep.IntoTheDeepBase;

abstract public class IntoTheDeepAuto extends IntoTheDeepBase {

    protected double sample1Offset = 22.0;
    //15
    protected double sample2Offset = 14.0;
    protected double forwardDistance = 7.0;
    abstract protected Position getPosition();
    @Override
    protected void pre_initialize() {

    }

    @Override
    protected void initialize() {
        driveBase.setLED( getPosition().getStaticColor() );

        driveBase.imu.resetYaw();
        HeadingHolder.setHeading(0);
        telemetry.addData("Gyro Reset", "Complete");

        telemetry.update();
        HeadingHolder.setHeading(0.0);
    }

    abstract protected void run_auto();

    /**
     * Run AUTO opmode
     */
    @Override
    protected void run_9808() {
        if( opModeIsActive() ) {
            displayDiagnostics();
            // Call the run code for the specific opmode
            run_auto();
        }
        finish();
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
