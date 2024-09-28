package org.firstinspires.ftc.deprecated;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.auto.Position;

abstract public class AutoBase extends LinearOpMode9808 {
    abstract protected void autoInit();
    abstract protected void autoPreInit();
    abstract protected void autoRun();
    abstract protected Position getPosition();

    /**
     * Prepare the bot for initialization and running
     */
    protected void pre_init_9808() {
        driveBase.init(hardwareMap, this);

        // Game specific Setup

        // Call the pre-init of the specific opmode
        autoPreInit();

        // Set LED state
        setLEDHeartbeat();
    }

    /**
     * Perform bot initialization
     */
    protected void init_9808() {
        // Call init for the specific opmode
        autoInit();

        telemetry.addLine("ready for START");
        telemetry.addLine( "Alliance: " + getAlliance().getColorString());
        telemetry.addLine( "Position " + getPosition().getPositionString());
        telemetry.update();
    }

    /**
     * Run AUTO opmode
     */
    protected void run_9808() {
        if( opModeIsActive() ) {
            // Call the run code for the specific opmode
            autoRun();
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
