package org.firstinspires.ftc.teamcode.auto.position;

import org.firstinspires.ftc.teamcode.auto.Auto;

public abstract class PositionBase {
    public abstract boolean allowParkOnly();
    public abstract int getPixelDropHeight();
    public abstract Auto.Selection getDefaultPark();
}
