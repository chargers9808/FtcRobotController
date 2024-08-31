package org.firstinspires.ftc.teamcode.auto.blue;

import org.firstinspires.ftc.teamcode.alliance.AllianceBase;
import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.alliance.BlueBase;

abstract public class BlueAuto extends Auto {
    private final BlueBase colorBase = new BlueBase();
    protected AllianceBase getColorBase() { return colorBase; }
    protected double distanceToCloseWall() { return driveBase.leftDistanceToWall(); }
    protected double cameraOffset() { return driveBase.cameraOffset; }

    protected String getParkString( Selection selection ) {
        String[] selectionStrings = { "Left", "Right", "Invalid"};
        return selectionStrings[ Math.min(selection.ordinal(), selectionStrings.length-1) ];
    }
    protected int getAprilTagId( String propLocation ) {
        int tagId = 0;
        switch( propLocation) {
            case "Left":
                tagId = 1;
                break;
            case "Right":
                tagId = 3;
                break;
            case "Center":
                tagId = 2;
                break;
        }
        return tagId;
    }

    protected PropLoc getPropLoc( String propLocation ) {
        PropLoc ret = PropLoc.LOC_INVALID;
        switch( propLocation) {
            case "Left":
                ret = PropLoc.LOC_WALL;
                break;
            case "Right":
                ret = PropLoc.LOC_MID;
                break;
            case "Center":
                ret = PropLoc.LOC_CENTER;
                break;
        }
        return ret;
    }

    /**
     * Correct distances (pos/neg) based on Blue / Red
     *
     * @param distance distance in inches
     * @return distance in inches, with correct sign for the side
     */
    protected double allianceCorrectedDistance(double distance ) {
        return distance;
    }
}
