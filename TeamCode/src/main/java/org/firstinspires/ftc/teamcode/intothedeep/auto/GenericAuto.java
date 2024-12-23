package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.intothedeep.auto.blue.IntoTheDeepBlueAuto;

/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Generic 9808", group = "Auto")
@Disabled
public class GenericAuto extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }
    protected void run_auto() {}
}
