

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RED Backdrop", group = "Autonomous")

//@Disabled

public class AutoRedBackDropLeague extends LinearOpMode {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ChargerCustomModelCubes.tflite";
    private static final String[] LABELS = {
            "blue prop",
            "red prop"
    };
    public String propLocation;

    /**
     * The variables to store our instance of the TensorFlow Object Detection processor and the object detector.
     */
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public double distanceFromFrontWall = 0;
    private int selection=1;

    @Override
    public void runOpMode() {

        VisionBase vision = new VisionBase();
        DraculaBase driveBase = new DraculaBase();
        driveBase.init(hardwareMap, this);   // initialize hardware defined in the hardware class
        //driveBase.initIMU(hardwareMap,this);// initialize hardware
        // driveBase.initIMU2(hardwareMap,this);// initialize hardware


        vision.initDoubleVision(hardwareMap, this);
        HeadingHolder.setHeading(0.); // record the initial heading of the robot at start of auto
        driveBase.imu.resetYaw();
        //driveBase.imu2.resetYaw();
        driveBase.arm.setPower(.8);
        driveBase.setRedHeartbeatLED();

        driveBase.grip.setPosition(driveBase.gripOpened);//holding the yellow pixel
        sleep(500);
        driveBase.tilt.setPosition(driveBase.tiltVertical);
        driveBase.arm.setTargetPosition(200);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        driveBase.holder.setPosition(driveBase.holderClosed);

        // Disable or re-enable the TFOD processor at any time.
        // visionPortal.setProcessorEnabled(tfod, true);


        driveBase.setBlueHeartbeatLED();// indication correct opmode for alliance


//--------------- Init is pressed ---------------------------------------------
        while (opModeInInit()) {
            vision.locateOurPropStartingRightEdge();
            telemetry.addLine("Red-near backdrop");
            telemetry.addLine("yellow pixel should be loaded onto gripper-- ");

            if (selection == 1) {
                telemetry.addLine("Full Auto Park Right");
                telemetry.addLine("Press A to change to Full Auto Park Left");
            }
            if (selection == 2) {
                telemetry.addLine("Full Auto Park Left");
                telemetry.addLine("Press B to change to Full Auto Park Right");
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
            // Full Auto Park Right
            if (vision.propLocation == "Left") {
                driveBase.plowFromRedRightStartToLeftSpike();

                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, -driveBase.lineUpOnLeftColumnRed + driveBase.rightDistanceToWall(), -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(4);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(4);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);

            }


            if (vision.propLocation == "Center") {
                driveBase.plowFromRedRightStartToCenterSpike();
                driveBase.tankDrive(.5, 10);
                driveBase.gyroTurn(.6, -90);
                driveBase.DriveSidewaysCorrected(.5, -(32 - driveBase.rightDistanceToWall()), -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(5);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                //
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(5);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.50);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -6);
                driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);

            }


            if (vision.propLocation == "Right") {
                driveBase.plowFromRedRightStartToRightSpike();

                driveBase.tankDrive(.6, 10);
                driveBase.gyroTurn(.6, -90);
                driveBase.DriveSidewaysCorrected(.5, -(30 - driveBase.rightDistanceToWall()), -90);
                sleep(500);
                driveBase.gyroTurn(.6, -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(6);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(6);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);

                driveBase.tankDrive(.3, -6);
                driveBase.gyroTurn(.6, -90);

                driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);
                //sleep(400);

            }

            if (opModeIsActive()) {


                driveBase.tilt.setPosition(driveBase.tiltVertical);
                sleep(500);
                driveBase.gyroTurn(.6, -90);

                driveBase.arm.setTargetPosition(3);
                sleep(500);
                driveBase.setRedHeartbeatLED();
                //while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6, -90);
                driveBase.tankDrive(.3, 10);
                HeadingHolder.setHeading(driveBase.robotFieldHeading());


                telemetry.addData("Path", "Complete");

                telemetry.update();
                while (opModeIsActive()) {
                }  // Pause to display last telemetry message.

            }

        }    // --------------- end of the Full Auto Park Right --------------------

        if (opModeIsActive() && selection == 2) {
            // Start of Full Auto Park Left
            if (vision.propLocation == "Left") {
                driveBase.plowFromRedRightStartToLeftSpike();

                driveBase.tankDrive(.5, 15);
                driveBase.DriveSidewaysCorrected(.5, -driveBase.lineUpOnLeftColumnRed + driveBase.rightDistanceToWall(), -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(4);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(4);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -5);

                driveBase.DriveSidewaysCorrected(.5, -17.5 , -90);

            }


            if (vision.propLocation == "Center") {
                driveBase.plowFromRedRightStartToCenterSpike();
                driveBase.tankDrive(.5, 10);
                driveBase.gyroTurn(.6, -90);
                driveBase.DriveSidewaysCorrected(.5, -(32 - driveBase.rightDistanceToWall()), -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(5);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                //
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(5);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.50);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3, -6);
                driveBase.DriveSidewaysCorrected(.5, -27, -90);

            }


            if (vision.propLocation == "Right") {
                driveBase.plowFromRedRightStartToRightSpike();

                driveBase.tankDrive(.6, 10);
                driveBase.gyroTurn(.6, -90);
                driveBase.DriveSidewaysCorrected(.5, -(30 - driveBase.rightDistanceToWall()), -90);
                sleep(500);
                driveBase.gyroTurn(.6, -90);

                driveBase.runtime.reset();
                vision.targetFound = false;
                while (vision.targetFound == false && driveBase.runtime.seconds() < 1.0) {
                    vision.getDistancesToAprilTag(6);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop
                driveBase.redAutoDropPixelNoDriving();

                driveBase.DriveSidewaysCorrected(.3, vision.lateralOffset + driveBase.cameraOffset, -90);
                driveBase.gyroTurn(.6, -90);
                vision.getDistancesToAprilTag(6);
                sleep(50);
                driveBase.tankDrive(.3, vision.forwardDistanceToTag - 9.75);
                driveBase.gyroTurn(.6, -90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);

                driveBase.tankDrive(.3, -6);
                driveBase.gyroTurn(.6, -90);

                driveBase.DriveSidewaysCorrected(.5, -32, -90);
                //sleep(400);

            }

            if (opModeIsActive()) {


                driveBase.tilt.setPosition(driveBase.tiltVertical);
                sleep(500);
                driveBase.gyroTurn(.6, -90);

                driveBase.arm.setTargetPosition(3);
                sleep(500);
                driveBase.setRedHeartbeatLED();
                //while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6, -90);
                driveBase.tankDrive(.3, 10);
                HeadingHolder.setHeading(driveBase.robotFieldHeading());


                telemetry.addData("Path", "Complete");

                telemetry.update();
                while (opModeIsActive()) {
                }  // Pause to display last telemetry message.

            }
        } // ------------------------- End of Full Auto Park Left --------------------------------
    } // --------------------------- End Of Run OpMode -------------------------------------
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
        tfod.setMinResultConfidence(0.87f);

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
            //telemetry.addData("- Prop Location", propLocation);

        }
        telemetry.addData("- Prop Location", propLocation);
        telemetry.addLine("-----------------------------");
        // end of the for() loop

    }   // --------------- end method locateOurProp" -------------------- "

}//     End of the class

