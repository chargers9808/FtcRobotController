package org.firstinspires.ftc.teamcode.auto.position;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class BackdropBase extends PositionBase {
    public boolean allowParkOnly() { return false; }
    public int getPixelDropHeight() { return 500; }
    public Auto.Selection getDefaultPark() { return Auto.Selection.PARK_WALL; }
}
