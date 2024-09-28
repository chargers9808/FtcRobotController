package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Position;

/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Red Net", group = "Auto")
public class NetRed extends IntoTheDeepRedAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }
}
