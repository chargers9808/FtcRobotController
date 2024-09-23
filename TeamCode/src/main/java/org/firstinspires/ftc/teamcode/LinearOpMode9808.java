package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class LinearOpMode9808 extends LinearOpMode {
    abstract protected void pre_init_9808();
    abstract protected void init_9808();
    abstract protected void run_9808();
    abstract protected Alliance getAlliance();
    // 9808 HW interface layer
    protected DraculaBase driveBase = new DraculaBase();
    protected String getColorString() {return getAlliance().getColorString();}
    protected RevBlinkinLedDriver.BlinkinPattern getStaticColor() {return getAlliance().getStaticColor();}
    protected RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor() {return getAlliance().getHeartbeatColor();}

    /**
     * Run the OpMode itself
     */
    public void runOpMode() {
        pre_init_9808();
        while (opModeInInit()) {
            init_9808();
        }
        run_9808();
    }

    /**
     * Set the LEDs to a static color for this alliance
     */
    protected void setStaticLED() {
        driveBase.setLED(getStaticColor());
    }

    /**
     * Set the LEDs to a heartbeat color for this alliance
     */
    protected void setLEDHeartbeat() {

        //driveBase.setLED(getHeartbeatColor());
    }
}
