package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Position;

/**
 * Observation side code for Red Alliance
 */
@Autonomous(name = "Red Observation", group = "Auto")
public class ObservationRed extends IntoTheDeepRedAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
}
