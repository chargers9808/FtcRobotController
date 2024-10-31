package org.firstinspires.ftc.deprecated.centerstage.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.deprecated.centerstage.auto.blue.IntoTheDeepBlueAuto;


/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Generic", group = "Auto")
public class GenericAuto extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }
}
