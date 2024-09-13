package org.firstinspires.ftc.deprecated.centerstage.auto.position;

import org.firstinspires.ftc.deprecated.centerstage.auto.Auto;

public abstract class PositionBase {
    public abstract boolean allowParkOnly();
    public abstract int getPixelDropHeight();
    public abstract Auto.Selection getDefaultPark();
}
