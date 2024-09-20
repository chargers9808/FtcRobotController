
package org.firstinspires.ftc.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.HeadingHolder;

@TeleOp(name = "BLUE TELEOP", group = "Linear Opmode")
//@Disabled
public class TeleopBlueLeague extends LinearOpMode {

//  Declare OpMode members.

    DraculaBase driveBase = new DraculaBase(); // Use Chargerbot's NEW hardware class

    double y = 0.0;
    double x = 0.0;
    double r = 0.0;
    double yCommand = 0.0;
    double xCommand = 0.0;
    double theta=0.;
    double creepSpeed=.1;
    double distanceFromBackDrop=5.;
    double distanceFromRightWall=5.;
    boolean fieldCentric = true;
    double distanceFromLeftWall=5.;
    boolean selected=false;

    boolean drivingModeToggled = false;
    boolean drivingSpeedToggled = false;
    boolean diagnosticMode = false;
    boolean driveFast = true;


    double selectedAngle=-HeadingHolder.getHeading();// imu angle to the quarry
    double lastSavedAngle= HeadingHolder.getHeading();// field heading of robot at start of telex

    //int     lastSavedArmPosition = HeadingHolder.getLastArmPosition();

    int     lastSavedArmPosition = 0;
    double  deadZone = .02;
    double  speedfactor = 1.5;   // higher number reduces the speed

/*
============================================================================================

              driving Controls:  left stick forward/back, left right... right stick rotate (left/right)

             1. arm                           ---    left Bumper extend, left Trigger retract
             2. pick up pixels                ---    a button


             3. position Gripper just above line 1      ---    b button
             4. position Gripper just above line 2      ---    y button
             5. position Gripper just above line 3      ---    right bumper + y button

             6. face the backdrop                    ---    right bumper + dpad_left
             7. face away from the backdrop          ---    right bumper + dpad_downn
             8. lift the arm to prepare for pixels   ---    right bumper + dpad_up

             9.  quickly lower arm                     ---    back button
             10. Reset GYRO to NORTH                 ---    left stick button
             11. creep in direction of dpad buttons  --     dpad
             12  Release the pixel                   ---     right trigger

             13  second driver releases and raises the lift   ---     gamepad 2 - Y
             14  second driver releases the drone             ---     gamepad 2 - b


============================================================================================
*/

    @Override
    public void runOpMode() {

        driveBase.init(hardwareMap,this);// initialize hardware

        driveBase.setSolidBlueLED();
        driveBase.arm.setPower(.8);
        driveBase.lift.setPower(.8);

        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.holder.setPosition(driveBase.holderClosed);


// -------------------------now waiting at the end of the init() -----

        while (!isStarted()) {

            if (gamepad1.a) {
                diagnosticMode = true;
            }

            if (fieldCentric) {
                telemetry.addLine("BLUE TeleOp :  FIELD Centric");
            } else {
                telemetry.addLine("BLUE TeleOp :  ROBOT Centric");
            }

            telemetry.addLine("Driving Speed is HIGH");
            telemetry.addLine("Press Driver 2 Left Stick button at any time to reset the gyro");
            if (gamepad2.left_stick_button) {
                driveBase.imu.resetYaw();
                HeadingHolder.setHeading(0);
                driveBase.setSolidGoldLED();
            }

            telemetry.addLine("Press A button to the enter diagnostic mode");
            if (diagnosticMode) {
                telemetry.addLine("OpMode is in diagnostic mode; press PLAY.");
            }
            telemetry.addData("Gyro initialized to:   ", lastSavedAngle);
            telemetry.addData("heading:   ", driveBase.getFieldHeading());
            telemetry.addLine("Waiting for START....");
            telemetry.update();
        }

//--------------------------Start of the TeleOp Loop--------------------------------------------

        driveBase.runtime.reset();
        driveBase.setBlueHeartbeatLED();

        while (opModeIsActive()) {
            // get the steering commands from either gamepad #1 or #2
            yCommand = (-gamepad1.left_stick_y - gamepad2.left_stick_y) / speedfactor; // forward and backward with respect to robot
            // (note: The joystick goes negative when pushed up, so we negate it)
            xCommand = (gamepad1.left_stick_x + gamepad2.left_stick_x) / speedfactor;  // left and right with respect to robot
            r = (-gamepad1.right_stick_x - gamepad2.right_stick_x) / 3*speedfactor;        // spin cw or ccw
            // create a steering "deadzone" near zero joystick deflection
            if (Math.abs(yCommand) < deadZone) {
                yCommand = 0.;
            }
            if (Math.abs(xCommand) < deadZone) {
                xCommand = 0.;
            }
            if (Math.abs(r) < deadZone) {
                r = 0.;
            }
// get the robot's heading from the IMU:
            theta = driveBase.getFieldHeading() * Math.PI / 180.;// convert to 0-2Pi angle
            if (theta < 0) {
                theta = theta + 2. * Math.PI;
            }

// for Field Centric, rotate the joystick commands into the frame of reference of the robot ("coordinate system rotation")
            x = xCommand * Math.cos(theta) + yCommand * Math.sin(theta);
            y = yCommand * Math.cos(theta) - xCommand * Math.sin(theta);

// or... for robot-centric steering, use the scaled joystick inputs directly

            if (!fieldCentric) { // make the joystick inputs non-linear to make it easier to control the rotation rate at slow speeds
                x = xCommand * xCommand* xCommand;
                y = yCommand * yCommand* yCommand;
            }

            if (!diagnosticMode) {
                telemetry.addLine("RightStickButton -> change Mode");
                telemetry.addLine("CURRENT ---> ROBOT-CENTRIC");
                telemetry.addLine("                                ");
            }

            if (!diagnosticMode) {
                if (fieldCentric) {
                    telemetry.addLine("RightStickButton -> change Mode");
                    telemetry.addLine("CURRENT ---------> FIELD-CENTRIC");
                    telemetry.addLine("                                ");
                    telemetry.addLine("                                ");
                }
            }

//================= toggle the driving mode by pressing the right joystick  =============================

            if (gamepad1.right_stick_button && !drivingModeToggled) {
                fieldCentric = !fieldCentric;
                drivingModeToggled = true;
            } else if (!gamepad1.right_stick_button) {
                drivingModeToggled = false;
            }

//================= toggle the driving speed by pressing the left joystick  =============================

            if ((gamepad1.left_stick_button) && !drivingSpeedToggled) {
                driveBase.runtime.reset();
                driveFast = !driveFast;
                drivingSpeedToggled = true;
            }else if (!gamepad1.left_stick_button && !gamepad2.left_stick_button) {
                drivingSpeedToggled = false;
            }

            if (gamepad2.left_stick_button) {
                driveBase.imu.resetYaw();
                HeadingHolder.setHeading(0);
                driveBase.setSolidGoldLED();
            }

            if (driveFast) {
                speedfactor = 1.3;
            } else if (!driveFast){
                speedfactor = 2.2;
            }

//================= dpad CREEPing  ( forward, rear, left, right)

            if ((gamepad1.dpad_up && !gamepad1.right_bumper) || gamepad2.dpad_up ) { y=creepSpeed; }
            if ((gamepad1.dpad_down && !gamepad1.right_bumper)  || gamepad2.dpad_down) { y=-creepSpeed; }
            if ((gamepad1.dpad_left && !gamepad1.right_bumper)  || gamepad2.dpad_left) { x=-creepSpeed; }
            if ((gamepad1.dpad_right && !gamepad1.right_bumper)  || gamepad2.dpad_right) { x=creepSpeed; }

            if (gamepad1.dpad_up && gamepad1.right_bumper)
            {   driveBase.tilt.setPosition(driveBase.tiltToPick);
                driveBase.grip.setPosition(driveBase.gripClosed);
                driveBase.arm.setTargetPosition(driveBase.armPickingPosition); }
            if (gamepad1.dpad_down && gamepad1.right_bumper)
            {  }// reserved for future control
            if (gamepad1.dpad_left && gamepad1.right_bumper)
            { driveBase.gyroTurn(.6,0);
                driveBase.grip.setPosition(driveBase.gripClosed);
                driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
                driveBase.tilt.setPosition(driveBase.tiltToPick);

                if (driveBase.rightDistanceToWall() < 48) {
                    driveBase.DriveSideways(.5, driveBase.rightDistanceToWall() - 16.5);
                    driveBase.gyroTurn(.6,0);
                    driveBase.tankDrive(.4, driveBase.frontLeftDistance() - 8);
                }

            }



            if (gamepad1.dpad_right && gamepad1.right_bumper)
            { driveBase.gyroTurn(.6,-90);
                driveBase.grip.setPosition(driveBase.gripClosed);
                driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
                driveBase.tilt.setPosition(driveBase.tiltToPick);

                if (driveBase.leftDistanceToWall() < 48) {
                    driveBase.DriveSideways(.5, -(driveBase.leftDistanceToWall() - 17));
                    driveBase.tankDrive(.5, driveBase.frontLeftDistance() - 8);
                }
            }

//================= put gripper in carry position   button y



//================= gripper and arm down to collect/release the grip -- button a

            // if (gamepad2.y && (driveBase.runtime.seconds()>90)) {
            if (gamepad2.y && !gamepad2.right_bumper) {

                driveBase.lift.setPower(.8);
                driveBase.liftRelease.setPosition(driveBase.liftReleaseOpen);
                driveBase.lift.setTargetPosition(driveBase.liftUp+1400);

                int lastPosition=0;
                int loops=0;
                int currentPosition=lastPosition;
                while (driveBase.lift.isBusy()){
                    currentPosition = driveBase.lift.getCurrentPosition();
                    if(Math.abs(currentPosition-lastPosition)<2){
                        loops++;
                    }
                    if(loops>5){
                        driveBase.liftNewTargetPosition=driveBase.lift.getCurrentPosition();
                        driveBase.lift.setTargetPosition((driveBase.liftNewTargetPosition));
                        loops=0;
                    }
                    lastPosition=currentPosition;
                }
            }
            if (gamepad2.a) {
                driveBase.lift.setPower(.8);

                driveBase.lift.setTargetPosition(5);
            }
            if (gamepad1.a) {
                sleep(200);
                driveBase.pickUpPixel();
            }
            if (gamepad2.b) {
                //if (gamepad2.b && (driveBase.runtime.seconds()>90)) {
                driveBase.gyroTurn(.6,-85);
                driveBase.holder.setPosition(driveBase.holderOpen);
                sleep(500);
                driveBase.droneRelease.setPosition(driveBase.droneReleaseOpen);
            }
//================= Raise arm to deliver pixel into lower zone on backdrop
//          + orient perpendicular to the backdrop, at the delivery distance.

            if (gamepad1.b && !gamepad1.right_bumper) {
                driveBase.gyroTurn(.5,90);
                distanceFromBackDrop=driveBase.frontDistance();// prepare to drive back
                if(distanceFromBackDrop<25)
                {
                    driveBase.tankDriveCorrected(.3,(10.-distanceFromBackDrop),90);
                    driveBase.gyroTurn(.5,90);
                    driveBase.armToLow();
                    driveBase.tilt.setPosition(driveBase.tiltToRelease);
                }
            }

            if (gamepad1.y && !gamepad1.right_bumper) {
                driveBase.gyroTurn(.5,90);
                distanceFromBackDrop=driveBase.frontDistance();// prepare to drive back
                if(distanceFromBackDrop<25)
                {
                    driveBase.tankDriveCorrected(.3,(10.-distanceFromBackDrop),90);
                    driveBase.gyroTurn(.5,90);

                    driveBase.armToMid();
                    driveBase.tilt.setPosition(driveBase.tiltToRelease);
                }
            }

            if (gamepad1.y && gamepad1.right_bumper) {
                driveBase.gyroTurn(.5,90);
                distanceFromBackDrop=driveBase.frontDistance();// prepare to drive back
                if(distanceFromBackDrop<25)
                {
                    driveBase.tankDriveCorrected(.3,(10.-distanceFromBackDrop),90);
                    driveBase.gyroTurn(.5,90);

                    driveBase.armToTop();
                    driveBase.tilt.setPosition(driveBase.tiltToRelease);
                }
            }

//================= lower and retract the arm after delivering pixels -- back button

            if (gamepad1.back|| gamepad2.back){
                driveBase.tilt.setPosition(driveBase.tiltToCarry);
                sleep(300);
                driveBase.armNewTargetPosition= driveBase.armLowered;
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                driveBase.grip.setPosition(driveBase.gripClosed);
            }
            /*
            if (gamepad1.x){
                driveBase.DriveSideways(.3,-(driveBase.leftDistanceToWall()-3));
                driveBase.gyroTurn(.4,-90);
            }
            */

// ----->>> Use the left bumper and right trigger to raise/lower the arm

            if ((gamepad2.left_trigger > .1) || (gamepad1.left_trigger > .1))   {
                driveBase.armNewTargetPosition = driveBase.arm.getCurrentPosition() -driveBase.armIncrement;
                //driveBase.armNewTargetPosition -= .8*driveBase.armIncrement;
                if(driveBase.armNewTargetPosition<driveBase.armLowered){driveBase.armNewTargetPosition=driveBase.armLowered;}
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                //while(driveBase.arm.isBusy()){}
                sleep(50);

            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                driveBase.armNewTargetPosition = driveBase.arm.getCurrentPosition() +driveBase.armIncrement;
                if(driveBase.armNewTargetPosition > driveBase.armup ){driveBase.armNewTargetPosition =driveBase.armup;}
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                //while(driveBase.arm.isBusy()){}
                sleep(50);
            }
            if ((gamepad2.right_trigger > .1) || (gamepad1.right_trigger > .1))   {
                driveBase.grip.setPosition(driveBase.gripClosed);
                sleep(200);
            }
            if (gamepad1.x)  {
                driveBase.grip.setPosition(driveBase.gripOpened);
                sleep(200);
            }

// ----->>> display parameters in diagnostic mode

            if (diagnosticMode) {
                telemetry.addData("Left Front     : ", driveBase.frontLeft.getCurrentPosition());
                telemetry.addData("Right Front    : ", driveBase.frontRight.getCurrentPosition());
                telemetry.addData("Left Rear      : ", driveBase.backLeft.getCurrentPosition());
                telemetry.addData("Right Rear     : ", driveBase.backRight.getCurrentPosition());
                telemetry.addData("arm motor      : ", driveBase.arm.getCurrentPosition());
                telemetry.addData("lift motor     : ", driveBase.lift.getCurrentPosition());
                telemetry.addData("tiltServo      : ",(driveBase.tiltPosition));
                telemetry.addData("gripServo      : ",(driveBase.gripPosition));
                telemetry.addData("right Distance : ",(driveBase.rightDistanceToWall()));
                telemetry.addData("left distance  : ",(driveBase.leftDistanceToWall()));
                telemetry.addData("left front distance : ",(driveBase.frontLeftDistance()));
                telemetry.addData("right front distance : ",(driveBase.frontRightDistance()));

                telemetry.update();
            }
            telemetry.addData("run-time : ",(driveBase.runtime.seconds()));


//==================== put everything in motion.. calculate tilt adjustment ==========================

            driveBase.applyMecPower2(x,y,r);

        }   //          end of "While Opmode is active"
    }       //          end of  End of the TeleOp Loop ================================================
}

