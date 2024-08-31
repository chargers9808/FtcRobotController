package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.alliance.BlueBase;
import org.firstinspires.ftc.teamcode.alliance.AllianceBase;

@TeleOp(name = "BLUE TELEOP (new)", group = "TeleOp")
@Disabled
public class BlueTeleop extends Teleop {
    private final BlueBase colorBase = new BlueBase();
    protected AllianceBase getColorBase() { return colorBase; }
    protected double drone_launch_deg() { return -85.0; }

    protected boolean approachFarWall() {
        return gamepad1.dpad_left && gamepad1.right_bumper;
    }

    protected boolean approachBackWall() {
        return gamepad1.dpad_right && gamepad1.right_bumper;
    }
}
