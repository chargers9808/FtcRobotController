package org.firstinspires.ftc.teamcode.auto.red;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.IntoTheDeepAuto;

abstract public class IntoTheDeepRedAuto extends IntoTheDeepAuto {
    private final Alliance alliance = new Alliance(Alliance.Color.RED);
    protected Alliance getAlliance() { return alliance; }
}
