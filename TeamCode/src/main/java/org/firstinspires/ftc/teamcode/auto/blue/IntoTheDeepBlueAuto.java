package org.firstinspires.ftc.teamcode.auto.blue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.IntoTheDeepAuto;

abstract public class IntoTheDeepBlueAuto extends IntoTheDeepAuto {
    private final Alliance alliance = new Alliance(Alliance.Color.BLUE);
    protected Alliance getAlliance() { return alliance; }
}
