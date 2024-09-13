package org.firstinspires.ftc.deprecated.centerstage.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;

@TeleOp(name = "RED TELEOP (new)", group = "TeleOp")
@Disabled
public class RedTeleop extends Teleop {
    private final Alliance alliance = new Alliance(Alliance.Color.RED);
    protected Alliance getAlliance() { return alliance; }
    protected double drone_launch_deg() { return 85.0; }

    protected boolean approachFarWall() {
        return gamepad1.dpad_right && gamepad1.right_bumper;
    }

    protected boolean approachBackWall() {
        return gamepad1.dpad_left && gamepad1.right_bumper;
    }
}
