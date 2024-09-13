package org.firstinspires.ftc.deprecated.centerstage.auto.position;

import org.firstinspires.ftc.deprecated.centerstage.auto.Auto;

public class WingBase extends PositionBase {
    public boolean allowParkOnly() { return true; }
    public int getPixelDropHeight() { return 500; }
    public Auto.Selection getDefaultPark() { return Auto.Selection.PARK_MID; }
}
