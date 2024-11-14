package org.firstinspires.ftc.teamcode.intothedeep.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Red Net", group = "Auto")
@Disabled
public class NetRed extends IntoTheDeepRedAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }
    protected void run_auto() {}
}
