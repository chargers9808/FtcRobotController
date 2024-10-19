package org.firstinspires.ftc.teamcode.intothedeep.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

/**
 * Observation side code for Blue Alliance
 */
@Autonomous(name = "Blue Observation", group = "Auto")
@Disabled
public class ObservationBlue extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
}
