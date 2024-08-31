

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RED-WING", group = "Autonomous")

//@Disabled

public class AutoRedWingLeague extends LinearOpMode {
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

        // Disable or re-enable the TFOD processor at any time.
        // visionPortal.setProcessorEnabled(tfod, true);


        driveBase.setRedHeartbeatLED();// indication correct opmode for alliance


//--------------- Init is pressed ---------------------------------------------
  
        while(opModeInInit())
        {
            vision.locateOurPropStartingRightEdge();
            telemetry.addLine("Red near wing");
            telemetry.addLine("yellow pixel should be loaded onto gripper-- ");

            if(selection==1){
                telemetry.addLine("Full Auto Park Left");
                telemetry.addLine("Press A to change to Park Only");
                telemetry.addLine("Press X to change to Full Auto Park Right");
            }
            if(selection==2){
                telemetry.addLine("PARK ONLY");
                telemetry.addLine("Press B to change to Full Auto Park Right");
                telemetry.addLine("Press X to change to FUll Auto Park Left");
            }
            if(selection==3){
                telemetry.addLine("Full Auto Park Right");
                telemetry.addLine("Press B to change to Full Auto Park Left");
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
        
        //waitForStart();
            //Play has been pressed

            if(opModeIsActive() && selection==1){
                // run the entire FULL AUTO Park Left

    //---------------------------------------------------------------------------------------------
                if(vision.propLocation=="Left"){
                    driveBase.plowFromRedWingStartToLeftSpike();
                    //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
                    driveBase.tilt.setPosition(driveBase.tiltToCarry);
                    driveBase.arm.setTargetPosition(driveBase.armLowered);
                    while(driveBase.arm.isBusy()){}

                    //driveBase.gyroTurn(.6,0);
                    // driveBase.DriveSidewaysCorrected(.3,12,0);
                    driveBase.tankDrive(.3,34);
                    driveBase.gyroTurn(.6,90);
                    driveBase.tankDrive(.6,-70);// drive under the stage door, into the middle
                    driveBase.gyroTurn(.6,-90);
                    driveBase.tilt.setPosition(driveBase.tiltToCarry);
                    driveBase.arm.setTargetPosition(driveBase.armLowered);
                    while(driveBase.arm.isBusy()){}
                    sleep(500);
                    driveBase.DriveSidewaysCorrected(.5,18,-90);//centered for apriltags
                    driveBase.gyroTurn(.6,-90);

                    driveBase.runtime.reset();
                    vision.targetFound=false;
                    while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                        vision.getDistancesToAprilTag(4);// loop until we find the target
                    }
                    telemetry.addData("Prop Location: ", vision.propLocation);
                    telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                    telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                    telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                    telemetry.update();

                    // remove the above telemetry once this is working correctly.
                    // once we have the lateral offset we can strafe to line up and then drive to the back drop

                    driveBase.redAutoDropPixelNoDrivingHigh();

                    driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
                    driveBase.gyroTurn(.6,-90);
                    vision.getDistancesToAprilTag(4);
                    sleep(50);
                    driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                    driveBase.gyroTurn(.6,-90);
                    driveBase.grip.setPosition(driveBase.gripClosed);
                    sleep(1000);
                    driveBase.tankDrive(.3,-5);

                    driveBase.DriveSidewaysCorrected(.5,-22,-90);

                }


                if(vision.propLocation=="Center"){
                    driveBase.plowFromRedWingStartToCenterSpike();
                    // driveBase.gyroTurn(.6,0);
                    driveBase.DriveSidewaysCorrected(.5,-2,90);
                    driveBase.tilt.setPosition(driveBase.tiltToCarry);
                    driveBase.arm.setTargetPosition(driveBase.armLowered);
                    while(driveBase.arm.isBusy()){}

                    //driveBase.gyroTurn(.6,90);
                    driveBase.tankDrive(.3,10);
                    driveBase.DriveSidewaysCorrected(.5,(54-driveBase.leftDistanceToWall()),90);

                    driveBase.gyroTurn(.5,90);
                    sleep(500);
                    driveBase.tankDrive(.6,-83);// drive under the stage door, into the middle
                    driveBase.gyroTurn(.6,-90);
                    sleep(500);
                    driveBase.DriveSidewaysCorrected(.5,25,-90);// centered?
                    driveBase.gyroTurn(.6,-90);

                    driveBase.runtime.reset();
                    vision.targetFound=false;
                    while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                        vision.getDistancesToAprilTag(5);// loop until we find the target
                    }
                    telemetry.addData("Prop Location: ", vision.propLocation);
                    telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                    telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                    telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                    telemetry.update();

                    // remove the above telemetry once this is working correctly.
                    // once we have the lateral offset we can strafe to line up and then drive to the back drop

                    // remove the above telemetry once this is working correctly.
                    // once we have the lateral offset we can strafe to line up and then drive to the back drop
                    driveBase.redAutoDropPixelNoDrivingHigh();

                    driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
                    driveBase.gyroTurn(.6,-90);
                    vision.getDistancesToAprilTag(5);
                    sleep(50);
                    driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                    driveBase.gyroTurn(.6,-90);

                    driveBase.grip.setPosition(driveBase.gripClosed);
                    sleep(1000);
                    driveBase.tankDrive(.3,-5);
                    driveBase.gyroTurn(.6,-90);

                    driveBase.DriveSidewaysCorrected(.5,-26,-90);

                }


                if(vision.propLocation=="Right"){
                    driveBase.plowFromRedWingStartToRightSpike();
                    //driveBase.tankDrive(.3,-15);// backup after placing pixel on spike
                    driveBase.tilt.setPosition(driveBase.tiltToCarry);
                    driveBase.arm.setTargetPosition(driveBase.armLowered);
                    while(driveBase.arm.isBusy()){}
                    driveBase.gyroTurn(.6,90);
                    driveBase.DriveSidewaysCorrected(.5,(54-driveBase.leftDistanceToWall()),90);
                    driveBase.gyroTurn(.6,90);
                    driveBase.tankDrive(.6,-73);// drive under the stage door, into the middle
                    driveBase.gyroTurn(.6,-90);
                    driveBase.DriveSidewaysCorrected(.5,30,-90);

                    driveBase.gyroTurn(.6,-90);

                    driveBase.runtime.reset();
                    vision.targetFound=false;
                    while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                        vision.getDistancesToAprilTag(6);// loop until we find the target
                    }
                    telemetry.addData("Prop Location: ", vision.propLocation);
                    telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
                    telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
                    telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
                    telemetry.update();

                    // remove the above telemetry once this is working correctly.
                    // once we have the lateral offset we can strafe to line up and then drive to the back drop
                    driveBase.redAutoDropPixelNoDrivingHigh();

                    driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
                    driveBase.gyroTurn(.6,-90);
                    vision.getDistancesToAprilTag(6);
                    sleep(50);
                    driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
                    driveBase.gyroTurn(.6,-90);

                    driveBase.grip.setPosition(driveBase.gripClosed);// release the yellow pixel
                    sleep(1000);
                    driveBase.tankDrive(.3,-5);
                    driveBase.DriveSidewaysCorrected(.5,-35,-90);
                }



                driveBase.tilt.setPosition(driveBase.tiltVertical);
                sleep(500);
                driveBase.gyroTurn(.6,-90);
                driveBase.tankDrive(.3,6);
                driveBase.arm.setTargetPosition(5);

                driveBase.setRedHeartbeatLED();
                driveBase.arm.setTargetPosition(5);
                driveBase.gyroTurn(.6,-90);
                sleep(300);
                HeadingHolder.setHeading(driveBase.robotFieldHeading());
                while(opModeIsActive()){}
                driveBase.turnOffLEDs();
                telemetry.addData("Path", "Complete");

                telemetry.update();
                sleep(1000);  // Pause to display last telemetry message.

            }
    //-----------------------------------end of the full auto---------------------------------------

             if(opModeIsActive() && selection==2){

    // run the entire Park Only routine
    //---------------------------------------------------------------------------------------------

        if(vision.propLocation=="Left"){
            driveBase.plowFromRedWingStartToLeftSpike();
            //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}

            driveBase.gyroTurn(.6,0);
            //driveBase.DriveSidewaysCorrected(.3,2,0);
            driveBase.tankDrive(.3,34);
            driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.6,-76);// drive under the stage door, into the middle

        }


        if(vision.propLocation=="Center"){
            driveBase.plowFromRedWingStartToCenterSpike();

            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}

            driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.3,10);
            driveBase.DriveSidewaysCorrected(.5,(56-driveBase.leftDistanceToWall()),90);
            driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.5,-92);
            driveBase.DriveSidewaysCorrected(.5,-2,90);

        }

        if(vision.propLocation=="Right"){
            driveBase.plowFromRedWingStartToRightSpike();
            //driveBase.tankDrive(.3,-15);// backup after placing pixel on spike
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}
            driveBase.gyroTurn(.6,90);
            driveBase.DriveSidewaysCorrected(.5,(54-driveBase.leftDistanceToWall()),90);

            driveBase.tankDrive(.6,-82);// drive under the stage door, into the middle
            //driveBase.gyroTurn(.6,-90);
        }
        if(opModeIsActive()){
        driveBase.arm.setTargetPosition(200);
        while(driveBase.arm.isBusy()){}
        driveBase.gyroTurn(.6,-160);
        driveBase.tankDrive(.3,3);
        driveBase.tilt.setPosition(driveBase.tiltToCarry);
        driveBase.grip.setPosition(driveBase.gripClosed);
        sleep(1500);
        driveBase.tankDrive(.3,-4);
        driveBase.gyroTurn(.6,-90);
        driveBase.tankDrive(.3,9);
        driveBase.DriveSidewaysCorrected(.5,6,-90);

        driveBase.tilt.setPosition(driveBase.tiltVertical);
        sleep(500);

        driveBase.setRedHeartbeatLED();
        driveBase.arm.setTargetPosition(5);
        driveBase.gyroTurn(.6,-90);
        sleep(300);
        HeadingHolder.setHeading(driveBase.robotFieldHeading());
        while(opModeIsActive()){}
        driveBase.turnOffLEDs();
        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
        }
            }
    // --------------- end of the Park Only opmode --------------------
        if(opModeIsActive() && selection==3){
            // --------------- Run Entire Full Auto Park Right -------------------------
        if(vision.propLocation=="Left"){
            driveBase.plowFromRedWingStartToLeftSpike();
            //driveBase.tankDrive(.3,-8);// backup after placing pixel on spike
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}

            //driveBase.gyroTurn(.6,0);
            // driveBase.DriveSidewaysCorrected(.3,12,0);
            driveBase.tankDrive(.3,34);
            driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.6,-70);// drive under the stage door, into the middle
            driveBase.gyroTurn(.6,-90);
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}
            sleep(500);
            driveBase.DriveSidewaysCorrected(.5,18,-90);//centered for apriltags
            driveBase.gyroTurn(.6,-90);

            driveBase.runtime.reset();
            vision.targetFound=false;
            while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                vision.getDistancesToAprilTag(4);// loop until we find the target
            }
            telemetry.addData("Prop Location: ", vision.propLocation);
            telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
            telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
            telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
            telemetry.update();

            // remove the above telemetry once this is working correctly.
            // once we have the lateral offset we can strafe to line up and then drive to the back drop

            driveBase.redAutoDropPixelNoDrivingHigh();

            driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
            driveBase.gyroTurn(.6,-90);
            vision.getDistancesToAprilTag(4);
            sleep(50);
            driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
            driveBase.gyroTurn(.6,-90);
            driveBase.grip.setPosition(driveBase.gripClosed);
            sleep(1000);
            driveBase.tankDrive(.3,-5);

            driveBase.DriveSidewaysCorrected(.5, 12, -90);
            driveBase.gyroTurn(.6,-90);
            driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);

        }


        if(vision.propLocation=="Center"){
            driveBase.plowFromRedWingStartToCenterSpike();
            // driveBase.gyroTurn(.6,0);
            driveBase.DriveSidewaysCorrected(.5,-2,90);
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}

            //driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.3,10);
            driveBase.DriveSidewaysCorrected(.5,(54-driveBase.leftDistanceToWall()),90);

            driveBase.gyroTurn(.5,90);
            sleep(500);
            driveBase.tankDrive(.6,-83);// drive under the stage door, into the middle
            driveBase.gyroTurn(.6,-90);
            sleep(500);
            driveBase.DriveSidewaysCorrected(.5,25,-90);// centered?
            driveBase.gyroTurn(.6,-90);

            driveBase.runtime.reset();
            vision.targetFound=false;
            while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                vision.getDistancesToAprilTag(5);// loop until we find the target
            }
            telemetry.addData("Prop Location: ", vision.propLocation);
            telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
            telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
            telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
            telemetry.update();

            // remove the above telemetry once this is working correctly.
            // once we have the lateral offset we can strafe to line up and then drive to the back drop

            // remove the above telemetry once this is working correctly.
            // once we have the lateral offset we can strafe to line up and then drive to the back drop
            driveBase.redAutoDropPixelNoDrivingHigh();

            driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
            driveBase.gyroTurn(.6,-90);
            vision.getDistancesToAprilTag(5);
            sleep(50);
            driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
            driveBase.gyroTurn(.6,-90);

            driveBase.grip.setPosition(driveBase.gripClosed);
            sleep(1000);
            driveBase.tankDrive(.3,-5);
            driveBase.gyroTurn(.6,-90);

            driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);

        }


        if(vision.propLocation=="Right"){
            driveBase.plowFromRedWingStartToRightSpike();
            //driveBase.tankDrive(.3,-15);// backup after placing pixel on spike
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            while(driveBase.arm.isBusy()){}
            driveBase.gyroTurn(.6,90);
            driveBase.DriveSidewaysCorrected(.5,(54-driveBase.leftDistanceToWall()),90);
            driveBase.gyroTurn(.6,90);
            driveBase.tankDrive(.6,-73);// drive under the stage door, into the middle
            driveBase.gyroTurn(.6,-90);
            driveBase.DriveSidewaysCorrected(.5,30,-90);

            driveBase.gyroTurn(.6,-90);

            driveBase.runtime.reset();
            vision.targetFound=false;
            while(vision.targetFound==false && driveBase.runtime.seconds()<1.0){
                vision.getDistancesToAprilTag(6);// loop until we find the target
            }
            telemetry.addData("Prop Location: ", vision.propLocation);
            telemetry.addData("time to Detect apriltag: ", driveBase.runtime.seconds());
            telemetry.addData("Lateral Offset to Tag: ", vision.lateralOffset);
            telemetry.addData("forward distance to backdrop: ", vision.forwardDistanceToTag);
            telemetry.update();

            // remove the above telemetry once this is working correctly.
            // once we have the lateral offset we can strafe to line up and then drive to the back drop
            driveBase.redAutoDropPixelNoDrivingHigh();

            driveBase.DriveSidewaysCorrected(.3,vision.lateralOffset+driveBase.cameraOffset,-90);
            driveBase.gyroTurn(.6,-90);
            vision.getDistancesToAprilTag(6);
            sleep(50);
            driveBase.tankDrive(.3,vision.forwardDistanceToTag-9.5);
            driveBase.gyroTurn(.6,-90);

            driveBase.grip.setPosition(driveBase.gripClosed);// release the yellow pixel
            sleep(1000);
            driveBase.tankDrive(.3,-5);
            driveBase.DriveSidewaysCorrected(.5, -(4 - driveBase.rightDistanceToWall()), -90);

        }



        driveBase.tilt.setPosition(driveBase.tiltVertical);
        sleep(500);
        driveBase.gyroTurn(.6,-90);
        driveBase.tankDrive(.3,6);
        driveBase.arm.setTargetPosition(5);

        driveBase.setRedHeartbeatLED();
        driveBase.arm.setTargetPosition(5);
        driveBase.gyroTurn(.6,-90);
        sleep(300);
        HeadingHolder.setHeading(driveBase.robotFieldHeading());
        while(opModeIsActive()){}
        driveBase.turnOffLEDs();
        telemetry.addData("Path", "Complete");

        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
    // --------------- end of the red wing FULL Right autonous opmode ------------------------------------
    }
    // --------------- end of the combined autonous opmode --------------------

}//     End of the class

