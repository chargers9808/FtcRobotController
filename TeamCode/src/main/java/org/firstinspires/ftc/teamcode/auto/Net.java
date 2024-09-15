package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.blue.IntoTheDeepBlueAuto;

/**
 * Net side code for Blue Alliance
 */
@Autonomous(name = "Blue Net", group = "Auto")
public class Net extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }
}
