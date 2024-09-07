package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/****
 *
 * NOTE: THIS IS A TEMPORARY PLACEHOLDER TO GET 10.0 WORKING
 * SEE DEPRECATED FOLDER FOR PREVIOUS IMPLEMENTATION
 *
 */




public class VisionBase {
    public double lateralOffset=0;
    public double forwardDistanceToTag=0;
    public boolean targetFound=false;
    public String propLocation;
    public void initDoubleVision(HardwareMap ahwMap, OpMode _callingOpMode) {

    }

    public void getDistancesToAprilTag(int tagID){

    }

    public void locateOurPropStartingRightEdge() {
        propLocation="Not Found";propLocation = "Left";
    }
}
