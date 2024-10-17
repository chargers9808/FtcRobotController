package org.firstinspires.ftc.deprecated.centerstage.teleop;

import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.HeadingHolder;


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
public abstract class Teleop extends LinearOpMode9808 {
    abstract protected boolean approachFarWall();
    abstract protected boolean approachBackWall();
    abstract protected double drone_launch_deg();

    private double x = 0.0;
    private double y = 0.0;
    private double r = 0.0;
    private boolean diagnosticMode = false;
    private boolean fieldCentric = true;
    private boolean drivingModeToggled = false;
    private boolean drivingSpeedToggled = false;
    private boolean driveFast = true;
    private final double lastSavedAngle = HeadingHolder.getHeading();
    private final int LIFT_UP_POS = driveBase.liftUp + 1400;
    private final static double DEAD_ZONE = 0.2;
    private final static double FAST_SPEED_FACTOR = 1.3;
    private final static double SLOW_SPEED_FACTOR = 2.2;
    private final static double CREEP_SPEED = 0.1;
    private final static double ENDGAME_START_TIME = 90.0;

    protected void pre_init_9808() {
        driveBase.init(hardwareMap, this);
        setStaticLED();

        // Set motor powers
        driveBase.arm.setPower(.8);
        driveBase.lift.setPower(.8);

        // Setup positions
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.holder.setPosition(driveBase.holderClosed);
    }

    protected void init_9808() {
        if (gamepad1.a) {
            diagnosticMode = !diagnosticMode;
        }

        if (fieldCentric) {
            telemetry.addLine(getColorString() + " TeleOp :  FIELD Centric");
        } else {
            telemetry.addLine(getColorString() + " TeleOp :  ROBOT Centric");
        }

        telemetry.addLine("Driving Speed is HIGH");
        telemetry.addLine("Press Driver 2 Left Stick button at any time to reset the gyro");
        if (gamepad2.left_stick_button) {
            driveBase.imu.resetYaw();
            HeadingHolder.setHeading(0);
            driveBase.setSolidGoldLED();
        }

        telemetry.addLine("Press A button to the toggle diagnostic mode");
        if (diagnosticMode) {
            telemetry.addLine("OpMode is in diagnostic mode; press PLAY.");
        }
        telemetry.addData("Gyro initialized to:   ", lastSavedAngle);
        telemetry.addData("heading:   ", driveBase.getFieldHeading());
        telemetry.addLine("Waiting for START....");
        telemetry.update();
    }

    protected void run_9808() {
        driveBase.runtime.reset();
        setLEDHeartbeat();

        while(opModeIsActive()) {
            steeringCommands();
            updateDrivingMode();
            updateDrivingSpeed();
            resetHeading();
            dpadCreep();
            liftArm();
            pushPixelsToWall();
            setupArm();
            hang();
            pickupPixel();
            launchDrone();
            raiseArmToDropPixel();
            back();
            moveArm();
            releasePixel();
            openGrip();
            diagnosticMode();
            if (driveBase.runtime.seconds() > ENDGAME_START_TIME) {
                driveBase.setSolidGoldLED();
            }
            telemetry.addData("run-time : ",(driveBase.runtime.seconds()));
            driveBase.applyMecPower2(x,y,r);
        }
    }

    /**
     * Process steering commands
     */
    private void steeringCommands() {
        double speedFactor = ( driveFast ? FAST_SPEED_FACTOR : SLOW_SPEED_FACTOR );
        double yCommand = (-gamepad1.left_stick_y - gamepad2.left_stick_y) / speedFactor; // forward and backward with respect to robot
        // (note: The joystick goes negative when pushed up, so we negate it)
        double xCommand = (gamepad1.left_stick_x + gamepad2.left_stick_x) / speedFactor;  // left and right with respect to robot
        r = (-gamepad1.right_stick_x - gamepad2.right_stick_x) / 3*speedFactor;        // spin cw or ccw
        // create a steering "deadzone" near zero joystick deflection
        if (Math.abs(yCommand) < DEAD_ZONE) {
            yCommand = 0.0;
        }
        if (Math.abs(xCommand) < DEAD_ZONE) {
            xCommand = 0.0;
        }
        if (Math.abs(r) < DEAD_ZONE) {
            r = 0.0;
        }

        // get the robot's heading from the IMU:
        double theta = driveBase.getFieldHeading() * Math.PI / 180.;// convert to 0-2Pi angle
        if (theta < 0) {
            theta = theta + 2.0 * Math.PI;
        }

        // for Field Centric, rotate the joystick commands into the frame of reference of the robot ("coordinate system rotation")
        x = xCommand * Math.cos(theta) + yCommand * Math.sin(theta);
        y = yCommand * Math.cos(theta) - xCommand * Math.sin(theta);

        // or... for robot-centric steering, use the scaled joystick inputs directly
        if (!fieldCentric) {
            // make the joystick inputs non-linear to make it easier to control the rotation rate at slow speeds
            x = xCommand * xCommand * xCommand;
            y = yCommand * yCommand * yCommand;
        }
    }

    /**
     * Provide output to the driver on current mode and process update command
     */
    private void updateDrivingMode() {
        if (!diagnosticMode) {
            telemetry.addLine("RightStickButton -> change Mode");
            if (fieldCentric) {
                telemetry.addLine("CURRENT ---------> FIELD-CENTRIC");
            } else {
                telemetry.addLine("CURRENT ---> ROBOT-CENTRIC");
            }
            telemetry.addLine("                                ");
            telemetry.addLine("                                ");
        }

        if (gamepad1.right_stick_button && !drivingModeToggled) {
            fieldCentric = !fieldCentric;
            drivingModeToggled = true;
        } else if (!gamepad1.right_stick_button) {
            drivingModeToggled = false;
        }
    }

    /**
     * Process drive speed update command
     */
    private void updateDrivingSpeed() {
        if ((gamepad1.left_stick_button) && !drivingSpeedToggled) {
            driveBase.runtime.reset();
            driveFast = !driveFast;
            drivingSpeedToggled = true;
        }else if (!gamepad1.left_stick_button && !gamepad2.left_stick_button) {
            drivingSpeedToggled = false;
        }
    }

    /**
     * Reset heading to 0
     */
    private void resetHeading() {
        if (gamepad2.left_stick_button) {
            driveBase.imu.resetYaw();
            HeadingHolder.setHeading(0.0);
            driveBase.setSolidGoldLED();
        }
    }

    /**
     * Process D-pad creep
     */
    private void dpadCreep() {
        if ((gamepad1.dpad_up && !gamepad1.right_bumper) || gamepad2.dpad_up ) { y=CREEP_SPEED; }
        if ((gamepad1.dpad_down && !gamepad1.right_bumper)  || gamepad2.dpad_down) { y=-CREEP_SPEED; }
        if ((gamepad1.dpad_left && !gamepad1.right_bumper)  || gamepad2.dpad_left) { x=-CREEP_SPEED; }
        if ((gamepad1.dpad_right && !gamepad1.right_bumper)  || gamepad2.dpad_right) { x=CREEP_SPEED; }
    }

    /**
     * Lift arm to prepare for pixels
     */
    private void liftArm() {
        if (gamepad1.dpad_up && gamepad1.right_bumper) {
            driveBase.tilt.setPosition(driveBase.tiltToPick);
            driveBase.grip.setPosition(driveBase.gripClosed);
            driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
        }
    }

    /**
     * Process D-pad + R bumper commands to push pixels against the wall
     */
    private void pushPixelsToWall() {
        double targetDeg = Double.NaN;
        // Determine if buttons were pushed to approach a wall
        if (approachFarWall()) {
            targetDeg = 0.0;
        } else if (approachBackWall()) {
//            targetDeg = -1 * getBackdropDeg();
        }

        // Process the approach if required
        if (targetDeg != Double.NaN) {
            driveBase.gyroTurn(.6, targetDeg);
            driveBase.grip.setPosition(driveBase.gripClosed);
            driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
            driveBase.tilt.setPosition(driveBase.tiltToPick);

            // Determine which sensor to read from based on D-pad buttons
            double distance = 0.0;
            if (gamepad1.dpad_right) {
                distance = driveBase.leftDistanceToWall();
            } else if (gamepad1.dpad_left) {
                distance = driveBase.rightDistanceToWall();
            }

            if (distance < 48) {
                // AJB: D-pad right was -17, D-pad left was -16.5
                driveBase.driveSidewaysCorrected(.5, distance - 17, targetDeg);
                driveBase.tankDrive(.4, driveBase.frontLeftDistance() - 8);
            }
        }
    }

    private void setupArm() {
        if (gamepad2.y) {
            driveBase.lift.setPower(.8);
            driveBase.liftRelease.setPosition(driveBase.liftReleaseOpen);
            driveBase.lift.setTargetPosition(LIFT_UP_POS);

            /*
            AJB: In work code for stall detection

            if( gamepad2.right_bumper ) {
                int lastPosition=0;
                int loops=0;
                while (driveBase.lift.isBusy()){
                    int currentPosition = driveBase.lift.getCurrentPosition();
                    if(Math.abs(currentPosition-lastPosition)<2){
                        loops++;
                    }
                    if(loops>5){
                        currentPosition=driveBase.lift.getCurrentPosition();
                        driveBase.lift.setTargetPosition(currentPosition);
                        loops=0;
                    }
                    lastPosition=currentPosition;
            }
            */
        }
    }

    /**
     * Retract arm to hang
     */
    private void hang() {
        if (gamepad2.a) {
            driveBase.lift.setPower(.8);
            driveBase.lift.setTargetPosition(5);
        }
    }

    /**
     * Pickup pixels
     */
    private void pickupPixel() {
        if (gamepad1.a) {
            sleep(200);
            driveBase.pickUpPixel();
        }
    }

    /**
     * Launch the drone
     */
    private void launchDrone() {
        if (gamepad2.b) {
            driveBase.gyroTurn(.6,drone_launch_deg() );
            driveBase.holder.setPosition(driveBase.holderOpen);
            sleep(500);
            driveBase.droneRelease.setPosition(driveBase.droneReleaseOpen);
        }
    }

    /**
     * Raise the arm to drop a pixel
     */
    private void raiseArmToDropPixel() {
        int targetLevel = 0;
        if ( gamepad1.b && !gamepad1.right_bumper ) {
           targetLevel = 1;
        } else if (gamepad1.y && !gamepad1.right_bumper) {
           targetLevel = 2;
        } else if (gamepad1.y && gamepad1.right_bumper) {
            targetLevel = 3;
        }

        if (targetLevel > 0) {
//            driveBase.gyroTurn(.5,getBackdropDeg());
            double distanceFromBackDrop = driveBase.frontDistance();// prepare to drive back
            if(distanceFromBackDrop<25)
            {
//                driveBase.tankDriveCorrected(.3,(10.-distanceFromBackDrop),getBackdropDeg());
//                driveBase.gyroTurn(.5,getBackdropDeg());
                if( targetLevel == 1) {
                    driveBase.armToLow();
                } else if (targetLevel == 2) {
                    driveBase.armToMid();
                } else if (targetLevel == 3) {
                    driveBase.armToTop();
                }
                driveBase.tilt.setPosition(driveBase.tiltToRelease);
            }
        }
    }

    /**
     * Process back button
     */
    private void back() {
        if (gamepad1.back|| gamepad2.back){
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            sleep(300);
            driveBase.arm.setTargetPosition(driveBase.armLowered);
            driveBase.grip.setPosition(driveBase.gripClosed);
        }
    }

    /**
     * Use left bumper and right trigger to control arm
     */
    private void moveArm() {
        boolean process = false;
        int targetPosition = 0;
        if ((gamepad2.left_trigger > .1) || (gamepad1.left_trigger > .1))   {
            targetPosition = Math.max( driveBase.arm.getCurrentPosition() -driveBase.armIncrement, driveBase.armLowered);
            process = true;
        } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
            targetPosition = Math.min(driveBase.arm.getCurrentPosition() + driveBase.armIncrement, driveBase.armup);
            process = true;
        }
        if (process) {
            driveBase.arm.setTargetPosition(targetPosition);
            sleep(50);
        }
    }

    /**
     * Release the pixel
     */
    private void releasePixel() {
        if ((gamepad2.right_trigger > .1) || (gamepad1.right_trigger > .1))   {
            driveBase.grip.setPosition(driveBase.gripClosed);
            sleep(200);
        }
    }

    /**
     * Open gripper
     */
    private void openGrip() {
        if (gamepad1.x)  {
            driveBase.grip.setPosition(driveBase.gripOpened);
            sleep(200);
        }
    }

    /**
     * Output diagnostic mode data
     */
    private void diagnosticMode() {
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
            telemetry.addData("right front distance : ",(driveBase.rearDistance()));
            telemetry.update();
        }
    }
}
