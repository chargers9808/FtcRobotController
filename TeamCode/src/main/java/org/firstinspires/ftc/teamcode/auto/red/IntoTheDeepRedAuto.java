package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.IntoTheDeepAuto;
import org.firstinspires.ftc.teamcode.auto.Position;

abstract public class IntoTheDeepRedAuto extends IntoTheDeepAuto {
    private final Alliance alliance = new Alliance(Alliance.Color.RED);
    protected Alliance getAlliance() { return alliance; }

    /**
     * Net side code for Red Alliance
     */
    @Autonomous(name = "Red Net", group = "Auto")
    public static class Net extends IntoTheDeepRedAuto {
        private final Position position = new Position(Position.Location.NET);
        protected Position getPosition() { return position; }
    }

    /**
     * Observation side code for Red Alliance
     */
    @Autonomous(name = "Red Observation", group = "Auto")
    public static class Observation extends IntoTheDeepRedAuto {
        private final Position position = new Position(Position.Location.OBSERVATION);
        protected Position getPosition() { return position; }
    }
}
