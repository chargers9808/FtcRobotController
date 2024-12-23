package org.firstinspires.ftc.teamcode.intothedeep.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

/**
 * Net side code for Blue Alliance
 */
@Autonomous(name = "Blue Net", group = "Auto")
@Disabled
public class NetBlue extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }
    protected void run_auto(){}
}
