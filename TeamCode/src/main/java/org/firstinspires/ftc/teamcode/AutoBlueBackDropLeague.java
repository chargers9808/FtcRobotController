

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Backdrop", group = "Autonomous")

//@Disabled

public class AutoBlueBackDropLeague extends LinearOpMode {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ChargerCustomModelCubes.tflite";
    private static final String[] LABELS = {
            "blue prop",
            "red prop"
    };
    public String propLocation;
    public int numberOfDetections=0;

    /**
     * The variables to store our instance of the TensorFlow Object Detection processor and the object detector.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private AprilTagDetection desiredTag = null;
    private VisionPortal visionPortal;
    private int selection = 1;

    public double distanceFromFrontWall = 0;
    public double x,y,r;

    @Override
    public void runOpMode() {

        VisionBase vision = new VisionBase();
        DraculaBase driveBase = new DraculaBase();

        driveBase.init(hardwareMap, this);   // initialize hardware defined in the hardware class
        vision.initDoubleVision(hardwareMap, this);

        HeadingHolder.setHeading(0.); // record the initial heading of the robot at start of auto
        driveBase.imu.resetYaw();
        driveBase.arm.setPower(.8);

        driveBase.grip.setPosition(driveBase.gripOpened);//holding the yellow pixel
        sleep(500);
        driveBase.tilt.setPosition(driveBase.tiltVertical);
        driveBase.arm.setTargetPosition(200);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        //initTfod();
        // Disable or re-enable the TFOD processor at any time.
        // visionPortal.setProcessorEnabled(tfod, true);


        driveBase.setBlueHeartbeatLED();// indication correct opmode for alliance


//--------------- Init is pressed ---------------------------------------------
        while (opModeInInit()) {
            vision.locateOurPropStartingRightEdge();
            telemetry.addLine("Blue-near backdrop");
            telemetry.addLine("yellow pixel should be loaded onto gripper-- ");

            if (selection == 1) {
                telemetry.addLine("Full Auto Park Left");
                telemetry.addLine("Press A to change to Full Auto Park Right");
            }
            if (selection == 2) {
                telemetry.addLine("Full Auto Park Right");
                telemetry.addLine("Press B to change to Full Auto Park Left");
            }

            if (gamepad1.a) {
                selection = 2;
            }
            if (gamepad1.b) {
                selection = 1;
            }


            // loop until driver presses PLAY
            if (vision.propLocation == "Left") {
                driveBase.setSolidRedLED();
            } else if (vision.propLocation == "Center") {
                driveBase.setSolidGoldLED();
            } else {
                driveBase.setSolidGreenLED();
            }
            telemetry.addLine("ready for START");
            telemetry.update();
        }

// Play has been pressed.. now perform one of three sequences depending on the location of the team prop
        //visionPortal.setProcessorEnabled(tfod, false);// turn off vision processor to conserve cycles
        if (opModeIsActive() && selection == 1) {
            if (vision.propLocation == "Left") {
                driveBase.plowFromBlueBackdropStartToLeftSpike();

                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, driveBase.lineUpOnLeftColumnBlue - driveBase.leftDistanceToWall(), 90);
                driveBase.runtime.reset();
                vision.targetFound = false;
                driveBase.gyroTurn(.6, 90);
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(1);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(1);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5, 4 - driveBase.leftDistanceToWall(), 90);
            }
            if (vision.propLocation == "Center") {
                driveBase.plowFromBlueBackdropStartToCenterSpike();
                driveBase.gyroTurn(.6, 90);
                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, driveBase.lineUpOnCenterColumnBlue - driveBase.leftDistanceToWall(), 90);

                driveBase.gyroTurn(.6, 90);
                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(2);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(2);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);
                driveBase.DriveSidewaysCorrected(.5, 4 - driveBase.leftDistanceToWall(), 90);
            }


            if (vision.propLocation == "Right") {
                driveBase.plowFromBlueBackdropStartToRightSpike();

                driveBase.gyroTurn(.6, 90);
                driveBase.tankDrive(.6, 18);
                driveBase.DriveSidewaysCorrected(.3, (driveBase.lineUpOnRightColumnBlue) - driveBase.leftDistanceToWall(), 90);
                // driveBase.tankDrive(.5,-4);
                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(3);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                //driveBase.tankDrive(.5,11);

                //driveBase.blueAutoDropPixel();
                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(3);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.5);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(900);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5, -18, 90);
                sleep(400);
                driveBase.DriveSidewaysCorrected(.5, 4 - driveBase.leftDistanceToWall(), 90);


            }

            driveBase.tilt.setPosition(driveBase.tiltVertical);
            sleep(500);
            driveBase.gyroTurn(.6, 90);
            driveBase.tankDrive(.3, 10);
            driveBase.arm.setTargetPosition(5);

            driveBase.setBlueHeartbeatLED();
            driveBase.arm.setTargetPosition(5);
            driveBase.gyroTurn(.6, 90);
            sleep(300);
            HeadingHolder.setHeading(driveBase.robotFieldHeading());
            while (opModeIsActive()) {
            }
            //driveBase.turnOffLEDs();
            visionPortal.setProcessorEnabled(tfod, false);
            visionPortal.setProcessorEnabled(aprilTag, false);
            telemetry.addData("Path", "Complete");

            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        } // --------------- end of the Full Auto Park Left --------------------

        if (opModeIsActive() && selection == 2) {
            if (vision.propLocation == "Left") {
                driveBase.plowFromBlueBackdropStartToLeftSpike();

                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, driveBase.lineUpOnLeftColumnBlue - driveBase.leftDistanceToWall(), 90);
                driveBase.runtime.reset();
                vision.targetFound = false;
                driveBase.gyroTurn(.6, 90);
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(1);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(1);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5, 32, 90);
            }
            if (vision.propLocation == "Center") {
                driveBase.plowFromBlueBackdropStartToCenterSpike();
                driveBase.gyroTurn(.6, 90);
                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, driveBase.lineUpOnCenterColumnBlue - driveBase.leftDistanceToWall(), 90);

                driveBase.gyroTurn(.6, 90);
                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(2);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(2);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);
                driveBase.DriveSidewaysCorrected(.5, 27 , 90);
            }


            if (vision.propLocation == "Right") {
                driveBase.plowFromBlueBackdropStartToRightSpike();

                driveBase.gyroTurn(.6, 90);
                driveBase.tankDrive(.6, 18);
                driveBase.DriveSidewaysCorrected(.3, (driveBase.lineUpOnRightColumnBlue) - driveBase.leftDistanceToWall(), 90);
                // driveBase.tankDrive(.5,-4);
                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(3);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                //driveBase.tankDrive(.5,11);

                //driveBase.blueAutoDropPixel();
                driveBase.blueAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, 90);
                driveBase.gyroTurn(.6, 90);
                vision.getDistancesToAprilTag(3);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.5);
                driveBase.gyroTurn(.6, 90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(900);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5,17.5, 90);


            }

            driveBase.tilt.setPosition(driveBase.tiltVertical);
            sleep(500);
            driveBase.gyroTurn(.6, 90);
            driveBase.tankDrive(.3, 10);
            driveBase.arm.setTargetPosition(5);

            driveBase.setBlueHeartbeatLED();
            driveBase.arm.setTargetPosition(5);
            driveBase.gyroTurn(.6, 90);
            sleep(300);
            HeadingHolder.setHeading(driveBase.robotFieldHeading());
            while (opModeIsActive()) {
            }
            //driveBase.turnOffLEDs();
            visionPortal.setProcessorEnabled(tfod, false);
            visionPortal.setProcessorEnabled(aprilTag, false);
            telemetry.addData("Path", "Complete");

            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.
        } // --------------------- End of Full Auto Park Right ---------------------------------
    } // ------------------------- End of Run OpMode -------------------------------------------
    // --------------- local methods follow --------------------

    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        //===== Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        //===== Disable or re-enable the TFOD processor at any time.
        // visionPortal.setProcessorEnabled(tfod, true);
    }
    // --------------- end method initTfod() --------------------



    private void locateOurProp() {
        double x = 0;
        double y = 0;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size()==0){propLocation="Left";}

        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if(x>250){propLocation="Right";}
            else if (x>5 && x<250){propLocation="Center";}
            else {propLocation="Left";}
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }// end of the for() loop
        telemetry.addData("- Prop Location", propLocation);
        telemetry.addLine("-----------------------------");

    }
    
}//     End of the class

