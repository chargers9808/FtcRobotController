package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;

import org.firstinspires.ftc.teamcode.VisionBase;
import org.firstinspires.ftc.teamcode.auto.position.PositionBase;

abstract public class Auto extends LinearOpMode9808 {
    public enum Selection {
        PARK_WALL,
        PARK_MID,
        PARK_ONLY,
        PARK_BACKDROP
    }

    protected enum PropLoc {
        LOC_WALL,
        LOC_CENTER,
        LOC_MID,
        LOC_INVALID
    }

    /**
     * Distance from the wall to each aprilTag on the backdrop
     */
    private final double[] backdropTagLocations = {
            22.0, // LOC_WALL
            28.5, // LOC_CENTER
            35.0, // LOC_MID
            Double.NaN // LOC_INVALID
    };

    protected double midParkDistFromWall = 50.0;

    private VisionBase vision;

    protected Selection modeSelection;

    abstract protected PositionBase getPositionBase();

    abstract protected void plow( String propLocation );
    abstract protected double distanceToCloseWall();
    abstract protected int getAprilTagId( String propLocation );
    abstract protected double allianceCorrectedDistance(double distance );

    abstract protected String getParkString( Selection selection );
    abstract protected PropLoc getPropLoc( String propLocation );
    abstract protected double cameraOffset();

    /*        PositionBase Wrappers                     */
    protected int getPixelDropHeight() { return getPositionBase().getPixelDropHeight(); }
    protected Selection getDefaultPark() { return getPositionBase().getDefaultPark(); }
    protected boolean allowWingOnly() { return getPositionBase().allowParkOnly(); }

    protected double getCurrentBackdropTagLocation() {
        return backdropTagLocations[getPropLoc( vision.propLocation ).ordinal()];
    }

    protected void pre_init_9808() {
        vision = new VisionBase();
        driveBase.init(hardwareMap, this);
        vision.initDoubleVision(hardwareMap, this);

        // Initial orientation
        HeadingHolder.setHeading(0.0);
        driveBase.imu.resetYaw();
        driveBase.arm.setPower(.8);

        // Setup arm and positions
        driveBase.grip.setPosition(driveBase.gripOpened);//holding the yellow pixel
        sleep(500);
        driveBase.tilt.setPosition(driveBase.tiltVertical);
        driveBase.arm.setTargetPosition(200);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);

        setLEDHeartbeat();

        modeSelection = getDefaultPark();
    }

    protected void init_9808() {
        vision.locateOurPropStartingRightEdge();

        // Get controller input for path selection
        getPathSelection();

        switch (vision.propLocation) {
            case "Left":
                driveBase.setSolidRedLED();
                break;
            case "Center":
                driveBase.setVioletLED();
                break;
            default:
                driveBase.setSolidGreenLED();
        }
        telemetry.addLine("ready for START");
        telemetry.update();
    }

    protected void run_9808() {
        boolean parkOnly = modeSelection == Selection.PARK_ONLY;
        if( opModeIsActive() ) {
            plow( vision.propLocation );
            driveToBackdrop( vision.propLocation, parkOnly );
            if( !parkOnly ) {
                findAprilTag();
                dropPixel();
                park();
            }
            wrapUp();
        }
    }

    protected void getPathSelection() {
        telemetry.addLine("------------------------------------------");
        if (modeSelection == Selection.PARK_WALL) { // parking towards the wall
            telemetry.addLine("Full Auto Park " + getParkString(Selection.PARK_WALL));
            telemetry.addLine();
            telemetry.addLine("Press A to change to Full Auto Park " + getParkString(Selection.PARK_MID));
            if(allowWingOnly()) {
                telemetry.addLine("Press X to change to PARK ONLY");
                telemetry.addLine("Press Y to change to Park Center");
            }
        }
        if (modeSelection == Selection.PARK_MID) { // parking towards the middle
            telemetry.addLine("Full Auto Park " + getParkString(Selection.PARK_MID));
            telemetry.addLine();
            telemetry.addLine("Press B to change to Full Auto Park " + getParkString(Selection.PARK_WALL));
            if(allowWingOnly()) {
                telemetry.addLine("Press X to change to PARK ONLY");
                telemetry.addLine("Press Y to change to Park Center");
            }
        }
        if (modeSelection == Selection.PARK_ONLY) { // parking only
            telemetry.addLine("PARK ONLY");
            telemetry.addLine();
            telemetry.addLine("Press A to change to Full Auto Park " + getParkString(Selection.PARK_MID));
            telemetry.addLine("Press B to change to Full Auto Park " + getParkString(Selection.PARK_WALL));
            if(allowWingOnly()) {
                telemetry.addLine("Press Y to change to Park Center");
            }
        }
        if (modeSelection == Selection.PARK_BACKDROP) { // parking center of backdrop
            telemetry.addLine("Full Auto Park Center");
            telemetry.addLine();
            telemetry.addLine("Press A to change to Full Auto Park " + getParkString(Selection.PARK_MID));
            telemetry.addLine("Press B to change to Full Auto Park " + getParkString(Selection.PARK_WALL));
            if(allowWingOnly()) {
                telemetry.addLine("Press X to change to PARK ONLY");
            }
        }

        telemetry.addLine("------------------------------------------");

        if (gamepad1.a) {
            modeSelection = Selection.PARK_MID;
        }
        if (gamepad1.b) {
            modeSelection = Selection.PARK_WALL;
        }

        /* Check Wing-only parking selections */
        if(allowWingOnly()) {
            if (gamepad1.x) {
                modeSelection = Selection.PARK_ONLY;
            }
            if (gamepad1.y) {
                modeSelection = Selection.PARK_BACKDROP;
            }
        }
    }

    protected void dropPixel() {
        driveBase.gyroTurn(.5, getBackdropDeg());
        driveBase.tilt.setPosition(driveBase.tiltToRelease);
        driveBase.arm.setTargetPosition(getPixelDropHeight());
        while (driveBase.arm.isBusy());
        driveBase.tankDrive(.3, 3.0);
        driveBase.gyroTurn(.5, getBackdropDeg());
        driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, getBackdropDeg());
        driveBase.gyroTurn(.6, getBackdropDeg());
        vision.getDistancesToAprilTag(getAprilTagId(vision.propLocation));
        sleep(50);
        driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.5); // AJB Updated from 9.75 2/24
        driveBase.gyroTurn(.6, getBackdropDeg());
        driveBase.grip.setPosition(driveBase.gripClosed);
        sleep(1000);
        driveBase.tankDrive(.3, -5);
    }

    private void findAprilTag() {
        driveBase.runtime.reset();
        vision.targetFound = false;
        while (!vision.targetFound && driveBase.runtime.seconds() < 1.0) {
            vision.getDistancesToAprilTag(getAprilTagId(vision.propLocation));// loop until we find the target
        }
        telemetry.addData("Prop Location: ", vision.propLocation);
        telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
        telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
        telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
        telemetry.update();
    }

    protected void park() {
        double distance = Double.NaN;
        if (modeSelection == Selection.PARK_WALL) {
            distance = 4 - distanceToCloseWall();
        } else if (modeSelection == Selection.PARK_MID) {
            distance = (midParkDistFromWall+4) - getCurrentBackdropTagLocation();
        } else if (modeSelection == Selection.PARK_ONLY) {
            // This is handled by driveToBackdrop
        } else if (modeSelection == Selection.PARK_BACKDROP) {
            distance = (30-distanceToCloseWall());
        }

        if (distance != Double.NaN) {
            driveBase.DriveSidewaysCorrected(.5, allianceCorrectedDistance(distance), getBackdropDeg());
        }
    }

    protected void wrapUp() {
        if (modeSelection != Selection.PARK_ONLY) {
            driveBase.tilt.setPosition(driveBase.tiltVertical);
        }
        sleep(500);
        driveBase.gyroTurn(.6, getBackdropDeg());
        if (modeSelection != Selection.PARK_BACKDROP) {
            driveBase.tankDrive(.3, 10);
        } else {
            driveBase.tankDrive(.3,-1);
        }
        if (modeSelection == Selection.PARK_ONLY) {
            driveBase.tankDrive(.3, 8);
            driveBase.tilt.setPosition(driveBase.tiltToRelease);
            driveBase.grip.setPosition(driveBase.gripClosed);
            sleep(200);
            driveBase.tilt.setPosition(driveBase.tiltVertical);
        }
        driveBase.arm.setTargetPosition(5);
        driveBase.gyroTurn(.6, getBackdropDeg());
        HeadingHolder.setHeading(driveBase.robotFieldHeading());
        setLEDHeartbeat();
        telemetry.addData("Path", "Complete");

        telemetry.update();
        while (opModeIsActive());
    }

    protected void driveToBackdrop( String propLocation, boolean parkOnly ) {
        // Backdrop drive rules
        driveBase.gyroTurn(.6,getBackdropDeg());
        driveBase.tankDrive(.5,10);
        double distance = getCurrentBackdropTagLocation() - distanceToCloseWall() - cameraOffset();
        driveBase.DriveSidewaysCorrected(.5, allianceCorrectedDistance(distance), getBackdropDeg());
    }
}
