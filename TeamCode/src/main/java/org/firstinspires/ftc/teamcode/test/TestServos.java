
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DraculaBase;

@TeleOp(name = "TestServos", group = "Linear Opmode")
public class TestServos extends LinearOpMode {

//  Declare OpMode members.

    DraculaBase driveBase = new DraculaBase(); // Use Chargerbot's PowerPlay hardware class
    public CRServo intake;
    @Override
    public void runOpMode() {

        driveBase.init(hardwareMap,this);// initialize hardware


// -------------------------now waiting at the end of the init() -----
        while (!isStarted()) {
            // telemetry.addLine("extend/retract arm - left trigger/bumper");
            telemetry.addLine("----------------------------------------");

            telemetry.addLine("lift release - dpad_left/right");
            telemetry.addLine("tilt angle - right trigger/bumper");
            telemetry.addLine("gripper          a/b....");
            telemetry.addLine("drone release    x/y....");
            telemetry.addLine("----------------------------------------");
            telemetry.addLine("Waiting for START....");
            telemetry.update();
        }
//--------------------------Start of the test Loop--------------------------------------------

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_left ){
                //driveBase.holderPosition+=driveBase.holderIncrement;

                // driveBase.liftReleasePosition+=driveBase.liftReleaseIncrement;
                driveBase.holderPosition=driveBase.holderOpen;

            } else if ((gamepad1.dpad_right)) {
                // driveBase.liftReleasePosition-=driveBase.liftReleaseIncrement;
                driveBase.holderPosition=driveBase.holderClosed;
                //driveBase.holderPosition-=driveBase.holderIncrement;

            }

            if (gamepad1.right_trigger>.1 ){
                driveBase.bucketPosition+=driveBase.bucketIncrement;
                driveBase.bucket.setPosition(driveBase.bucketPosition);
                sleep(50);

            } else if ((gamepad1.right_bumper)) {
                driveBase.bucketPosition-=driveBase.bucketIncrement;
                driveBase.bucket.setPosition(driveBase.bucketPosition);
                sleep(50);

            }

            if (gamepad1.a){
                driveBase.gripPosition=driveBase.gripOpened;


            } else if ((gamepad1.b)) {
                driveBase.gripPosition=driveBase.gripClosed;

            }

            if (gamepad1.x){
                driveBase.droneReleasePosition=driveBase.droneReleaseOpen;
                //driveBase.droneReleasePosition-=driveBase.droneReleaseIncrement;

            } else if ((gamepad1.y)) {
                driveBase.droneReleasePosition = driveBase.droneReleaseClosed;
                //driveBase.droneReleasePosition+=driveBase.droneReleaseIncrement;
            }
/*
            if ((gamepad2.left_trigger > .1) || (gamepad1.left_trigger > .1))   {
                {driveBase.armNewTargetPosition -= driveBase.armIncrement;}
                if(driveBase.armNewTargetPosition<driveBase.armLowered){driveBase.armNewTargetPosition=driveBase.armLowered;}
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                while(driveBase.arm.isBusy()){}
                sleep(150);

            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                driveBase.armNewTargetPosition += driveBase.armIncrement;
                if(driveBase.armNewTargetPosition > driveBase.armup ){driveBase.armNewTargetPosition =driveBase.armup;}
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                while(driveBase.arm.isBusy()){}
                sleep(150);
            }
            */

//            telemetry.addData("droneRelease (x/y): ", driveBase.droneReleasePosition);
//            //telemetry.addData("liftRelease servo (dpad_left/dpad_right): ", driveBase.liftReleasePosition);
//            telemetry.addData("Arm position (Left Trigger/Bumper): ", driveBase.armNewTargetPosition);
//            telemetry.addData("tilt (right T/B): ", driveBase.tiltPosition);
//            telemetry.addData("grip (a/b): ", driveBase.gripPosition);
            telemetry.addData("bucket position ", driveBase.bucketPosition);

            telemetry.update();
//
//            driveBase.droneRelease.setPosition(driveBase.droneReleasePosition);
//            driveBase.liftRelease.setPosition(driveBase.liftReleasePosition);
//            driveBase.tilt.setPosition(driveBase.tiltPosition);
//            driveBase.grip.setPosition(driveBase.gripPosition);
//            driveBase.holder.setPosition(driveBase.holderPosition);

        }
    }

}



