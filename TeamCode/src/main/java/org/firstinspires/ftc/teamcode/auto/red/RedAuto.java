package org.firstinspires.ftc.teamcode.auto.red;

import org.firstinspires.ftc.teamcode.alliance.AllianceBase;
import org.firstinspires.ftc.teamcode.alliance.RedBase;
import org.firstinspires.ftc.teamcode.auto.Auto;

abstract public class RedAuto extends Auto {
    private final RedBase colorBase = new RedBase();
    protected AllianceBase getColorBase() { return colorBase; }
    protected double distanceToCloseWall() { return driveBase.rightDistanceToWall(); }
    protected double cameraOffset() { return -1 * driveBase.cameraOffset; }

    protected String getParkString( Selection selection ) {
        String[] selectionStrings = { "Right", "Left", "Invalid"};
        return selectionStrings[ Math.min(selection.ordinal(), selectionStrings.length-1) ];
    }
    protected int getAprilTagId( String propLocation ) {
        int tagId = 0;
        switch( propLocation) {
            case "Left":
                tagId = 4;
                break;
            case "Right":
                tagId = 6;
                break;
            case "Center":
                tagId = 5;
                break;
        }
        return tagId;
    }

    protected PropLoc getPropLoc( String propLocation ) {
        PropLoc ret = PropLoc.LOC_INVALID;
        switch( propLocation) {
            case "Left":
                ret = PropLoc.LOC_MID;
                break;
            case "Right":
                ret = PropLoc.LOC_WALL;
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
        return -1.0 * distance;
    }
}
