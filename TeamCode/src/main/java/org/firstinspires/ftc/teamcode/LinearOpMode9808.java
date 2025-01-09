package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

public abstract class LinearOpMode9808 extends LinearOpMode {
    /**
     * Code that will be run once prior to the initialization loop
     */
    abstract protected void pre_init_9808();

    /**
     * Code that will run repeatedly while in Init mode
     */
    abstract protected void init_9808();

    /**
     * Code that will run once once 'Start' has been pressed
     */
    abstract protected void run_9808();
    abstract protected Alliance getAlliance();
    // 9808 HW interface layer
    protected DraculaBase driveBase = new DraculaBase();
    protected String getColorString() {return getAlliance().getColorString();}
    protected RevBlinkinLedDriver.BlinkinPattern getStaticColor() {return getAlliance().getStaticColor();}
    protected RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor() {return getAlliance().getHeartbeatColor();}
    protected DraculaBase.LEDColor getLEDColor() { return getAlliance().getLEDColor();}

    /**
     * Run the OpMode itself
     */
    public void runOpMode() {
        pre_init_9808();
        while (opModeInInit()) {
            init_9808();
        }
        driveBase.runtime.reset();
        run_9808();
    }

    /**
     * Set the LEDs to a static color for this alliance
     */
    protected void setStaticLED() {
        if(driveBase.useBlinkinDriver) {
            driveBase.setLED(getStaticColor());
        } else {
            driveBase.setLED(getLEDColor());
        }
    }

    /**
     * Set the LEDs to a heartbeat color for this alliance
     */
    protected void setLEDHeartbeat() {
        if(driveBase.useBlinkinDriver) {
            driveBase.setLED(getHeartbeatColor());
        } else {
            driveBase.setLED(getLEDColor());
        }
    }

    protected Pose2DGobilda getLocation() {
        return driveBase.odometryComputer.getPosition();
    }
}
