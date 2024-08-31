package org.firstinspires.ftc.teamcode.auto.position;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class WingBase extends PositionBase {
    public boolean allowParkOnly() { return true; }
    public int getPixelDropHeight() { return 500; }
    public Auto.Selection getDefaultPark() { return Auto.Selection.PARK_MID; }
}
