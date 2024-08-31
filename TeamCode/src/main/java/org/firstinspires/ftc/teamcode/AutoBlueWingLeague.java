

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Blue-WING", group = "Autonomous")

//@Disabled

public class AutoBlueWingLeague extends LinearOpMode {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ChargerCustomModelCubes.tflite";
    private static final String[] LABELS = {
            "blue prop",
            "red prop"
    };


    /**
     * The variables to store our instance of the TensorFlow Object Detection processor and the object detector.
     */
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public double distanceFromFrontWall = 0;
    public double lateral=0;
    public int selection=1;

    @Override
    public void runOpMode() {

        VisionBase vision = new VisionBase();
        DraculaBase driveBase=new DraculaBase();

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
        while(opModeInInit())
        {
            vision.locateOurPropStartingRightEdge();
            telemetry.addLine("Blue near wing");
            telemetry.addLine("yellow pixel should be loaded onto gripper-- ");

            if(selection==1){
                telemetry.addLine("Full Auto Park Right");
                telemetry.addLine("Press A to change to Park Only");
                telemetry.addLine("Press X to change to Full Auto Park Left");
            }
            if(selection==2){
                telemetry.addLine("PARK ONLY");
                telemetry.addLine("Press B to change to Full Auto Park Right");
                telemetry.addLine("Press X to change to FUll Auto Park Left");
            }
            if(selection==3){
                telemetry.addLine("Full Auto Park Left");
                telemetry.addLine("Press B to change to Full Auto Park Right");
                telemetry.addLine("Press A to change to Park Only");
            }
            if(gamepad1.a){selection=2;}
            if(gamepad1.b){selection=1;}
            if(gamepad1.x){selection=3;}

            // loop until driver presses PLAY
            if(vision.propLocation=="Left"){driveBase.setSolidRedLED();}
            else if(vision.propLocation=="Center"){driveBase.setSolidGoldLED();}
            else {driveBase.setSolidGreenLED();}
            telemetry.addLine("ready for START");
            telemetry.update();
        }


// Play has been pressed.. now perform one of three sequences depending on the location of the team prop
        //visionPortal.setProcessorEnabled(tfod, false);// turn off vision processor to conserve cycles

        if(opModeIsActive() && selection==1){
            // run the entire FULL AUTO

            if(vision.propLocation=="Left"){
                driveBase.plowFromBlueRightStartToLeftSpike();
                //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);
                //driveBase.DriveSidewaysCorrected(.3,12,0);
                driveBase.tankDrive(.4,32);
                driveBase.gyroTurn(.5,-90);
                driveBase.tankDrive(.6,-64);// drive under the stage door, into the middle
                driveBase.gyroTurn(.5,90);
                driveBase.DriveSidewaysCorrected(.4,-35,90);// move to center
                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(1);// loop until we find the target
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
                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(1);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                //driveBase.tankDrive(.3,.5);

                // driveBase.tankDrive(.3,.5);
                //driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3,-5);
                driveBase.gyroTurn(.6,90);

                driveBase.DriveSidewaysCorrected(.5,32,90);


            }


            if(vision.propLocation=="Center"){
                driveBase.plowFromBlueRightStartToCenterSpike();
                driveBase.gyroTurn(.6,0);
                driveBase.tankDrive(.6,-4);
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);

                driveBase.DriveSidewaysCorrected(.5,10,0);
                driveBase.tankDrive(.3,40);

                driveBase.gyroTurn(.6,-90);
                driveBase.tankDrive(.6,-68);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,90);
                driveBase.DriveSidewaysCorrected(.5,-35,90);// move to center
                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(2);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(2);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                //driveBase.tankDrive(.3,.5);

                // driveBase.tankDrive(.3,.5);
                //driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(900);
                driveBase.tankDrive(.3,-5);

                driveBase.DriveSidewaysCorrected(.5,27,90);

            }


            //driveBase.parkOnRightOfBackdrop();

            if(vision.propLocation=="Right"){
                driveBase.plowFromBlueRightStartToRightSpike();
                driveBase.tankDrive(.3,-5);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6,0);
                driveBase.DriveSidewaysCorrected(.5,-(29-driveBase.rightDistanceToWall()),0);
                driveBase.gyroTurn(.6,0);

                driveBase.tankDrive(.6,38);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,-90);

                driveBase.tankDrive(.6,-55);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,90);
                driveBase.DriveSidewaysCorrected(.5,-24,90);// move to center

                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(3);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(3);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);// release the yellow pixel
                sleep(1000);

                driveBase.tankDrive(.3,-5);
                driveBase.DriveSidewaysCorrected(.5,17.5,90);
            }


            driveBase.tilt.setPosition(driveBase.tiltVertical);
            sleep(500);
            driveBase.gyroTurn(.6,90);

            driveBase.arm.setTargetPosition(5);
            driveBase.tankDrive(.3,13);
            driveBase.setBlueHeartbeatLED();
            HeadingHolder.setHeading(driveBase.robotFieldHeading());
            telemetry.addData("Path", "Complete");
            telemetry.update();
            while(opModeIsActive()){}
        }
        // --------------- end of the blue wing FULL Right autonous opmode ------------------------------------

        else if(opModeIsActive() && selection==2){
            // ------ run the blue wing "park only" autonous opmode --------------

            if(vision.propLocation=="Left"){
                driveBase.plowFromBlueRightStartToLeftSpike();
                //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);
                //driveBase.DriveSidewaysCorrected(.3,12,0);
                driveBase.tankDrive(.4,30);
                driveBase.gyroTurn(.6,-90);
                driveBase.tankDrive(.6,-84);// drive under the stage door, into the middle
                //driveBase.gyroTurn(.6,90);
            }
            if(vision.propLocation=="Center"){
                driveBase.plowFromBlueRightStartToCenterSpike();
                driveBase.gyroTurn(.6,0);
                driveBase.tankDrive(.6,-4);
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);

                driveBase.DriveSidewaysCorrected(.5,10,0);
                driveBase.tankDrive(.3,35);

                driveBase.gyroTurn(.6,-90);
                driveBase.tankDrive(.6,-93);// drive under the stage door, into the middle
                //driveBase.gyroTurn(.6,90);
            }
            if(vision.propLocation=="Right"){
                driveBase.plowFromBlueRightStartToRightSpike();
                driveBase.tankDrive(.3,-5);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6,0);
                driveBase.DriveSidewaysCorrected(.5,-(29-driveBase.rightDistanceToWall()),0);
                driveBase.gyroTurn(.6,0);

                driveBase.tankDrive(.6,38);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,-90);

                driveBase.tankDrive(.6,-78);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,90);
            }

            if(opModeIsActive()){

                driveBase.arm.setTargetPosition(200);
                while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6,160);
                driveBase.tankDrive(.3,3);
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1500);
                driveBase.tankDrive(.3,-7);
                driveBase.gyroTurn(.6,90);
                driveBase.tankDrive(.3,7);
                driveBase.DriveSidewaysCorrected(.5,-3,90);

                driveBase.tilt.setPosition(driveBase.tiltVertical);
                driveBase.arm.setTargetPosition(5);
                sleep(500);
                driveBase.gyroTurn(.6,90);
                driveBase.setBlueHeartbeatLED();

                sleep(300);
                HeadingHolder.setHeading(driveBase.robotFieldHeading());
                telemetry.addData("Path", "Complete");
                telemetry.update();
            }

        }// --------------- end of the blue wing park only autonous opmode -------------------------

        if(opModeIsActive() && selection==3){
            // run the entire FULL AUTO

            if(vision.propLocation=="Left"){
                driveBase.plowFromBlueRightStartToLeftSpike();
                //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);
                //driveBase.DriveSidewaysCorrected(.3,12,0);
                driveBase.tankDrive(.4,32);
                driveBase.gyroTurn(.5,-90);
                driveBase.tankDrive(.6,-64);// drive under the stage door, into the middle
                driveBase.gyroTurn(.5,90);
                driveBase.DriveSidewaysCorrected(.4,-35,90);// move to center
                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(1);// loop until we find the target
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
                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(1);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                //driveBase.tankDrive(.3,.5);

                // driveBase.tankDrive(.3,.5);
                //driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(1000);
                driveBase.tankDrive(.3,-5);
                driveBase.gyroTurn(.6,90);

                driveBase.DriveSidewaysCorrected(.5,-17,90);


            }


            if(vision.propLocation=="Center"){
                driveBase.plowFromBlueRightStartToCenterSpike();
                driveBase.gyroTurn(.6,0);
                driveBase.tankDrive(.6,-4);
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}

                driveBase.gyroTurn(.6,0);

                driveBase.DriveSidewaysCorrected(.5,10,0);
                driveBase.tankDrive(.3,40);

                driveBase.gyroTurn(.6,-90);
                driveBase.tankDrive(.6,-68);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,90);
                driveBase.DriveSidewaysCorrected(.5,-35,90);// move to center
                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(2);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(2);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                //driveBase.tankDrive(.3,.5);

                // driveBase.tankDrive(.3,.5);
                //driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(900);
                driveBase.tankDrive(.3,-5);

                driveBase.DriveSidewaysCorrected(.5,-27,90);

            }


            //driveBase.parkOnRightOfBackdrop();

            if(vision.propLocation=="Right"){
                driveBase.plowFromBlueRightStartToRightSpike();
                driveBase.tankDrive(.3,-5);// backup after placing pixel on spike
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                driveBase.arm.setTargetPosition(driveBase.armLowered);
                while(driveBase.arm.isBusy()){}
                driveBase.gyroTurn(.6,0);
                driveBase.DriveSidewaysCorrected(.5,-(29-driveBase.rightDistanceToWall()),0);
                driveBase.gyroTurn(.6,0);

                driveBase.tankDrive(.6,38);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,-90);

                driveBase.tankDrive(.6,-55);// drive under the stage door, into the middle
                driveBase.gyroTurn(.6,90);
                driveBase.DriveSidewaysCorrected(.5,-24,90);// move to center

                driveBase.gyroTurn(.6,90);
                //face the backboard about 24" back from the center
                sleep(500);

                driveBase.runtime.reset();
                vision.targetFound=false;
                while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                    vision.getDistancesToAprilTag(3);// loop until we find the target
                }
                telemetry.addData("Prop Location: ", vision.propLocation);
                telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                telemetry.update();

                // remove the above telemetry once this is working correctly.
                // once we have the lateral offset we can strafe to line up and then drive to the back drop

                driveBase.blueAutoDropPixelNoDrivingHigh();

                driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,90);
                driveBase.gyroTurn(.6,90);
                vision.getDistancesToAprilTag(3);
                sleep(50);
                driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                driveBase.gyroTurn(.6,90);
                driveBase.grip.setPosition(driveBase.gripClosed);// release the yellow pixel
                sleep(1000);

                driveBase.tankDrive(.3,-5);
                driveBase.DriveSidewaysCorrected(.5,4-(driveBase.leftDistanceToWall()),90);
            }


            driveBase.tilt.setPosition(driveBase.tiltVertical);
            sleep(500);
            driveBase.gyroTurn(.6,90);

            driveBase.arm.setTargetPosition(5);
            driveBase.tankDrive(.3,11);
            driveBase.setBlueHeartbeatLED();
            HeadingHolder.setHeading(driveBase.robotFieldHeading());
            telemetry.addData("Path", "Complete");
            telemetry.update();
            while(opModeIsActive()){}
        }// --------------- end of the blue wing FULL Right autonous opmode ------------------------------------
    }// ------------------- end runopmode() --------------------------------------------------------
}//------------------------ End of the class -------------------------------------------------------

