package org.firstinspires.ftc.teamcode.intothedeep.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

/**
 * Observation side code for Red Alliance
 */
@Autonomous(name = "Red Observation", group = "Auto")
@Disabled
public class ObservationRed extends IntoTheDeepRedAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }
    protected void run_auto() {}
}
