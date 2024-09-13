package org.firstinspires.ftc.teamcode.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Position;

/**
 * Observation side code for Blue Alliance
 */
@Autonomous(name = "Blue Observation", group = "Auto")
public class Observation extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
}
