
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

@TeleOp(name = "OldTeleop (Recatored)", group = "Linear Opmode")
//@Disabled
public class OldTeleop extends IntoTheDeepBase {

    DraculaBase driveBase = new DraculaBase();

    double y = 0.0;
    double x = 0.0;
    double r = 0.0;
    double yCommand = 0.0;
    double xCommand = 0.0;
    double theta = 0.;
    double creepSpeed = .1;
    double distanceFromBackDrop = 5.;
    boolean fieldCentric = true;
    boolean drivingModeToggled = false;
    boolean drivingSpeedToggled = false;
    boolean diagnosticMode = false;
    boolean driveFast = true;
    double lastSavedAngle = HeadingHolder.getHeading();// field heading of robot at start of telex
    double deadZone = .02;
    double speedFactor = 1.5;   // higher number reduces the speed

/*
============================================================================================
              driving Controls:
                  Left Stick - Movement (forward, back, left, right)
                  Right Stick - Rotation (clockwise, counterclockwise)
                  DPad - Slow movement

             1. arm                                             left Bumper extend, left Trigger retract
             2. pick up pixels                                  a button
             3. position Gripper just above line 1              b button
             4. position Gripper just above line 2              y button
             5. position Gripper just above line 3              right bumper + y button
             6. face the backdrop                               right bumper + dpad_left
             7. face away from the backdrop                     right bumper + dpad_downn
             8. lift the arm to prepare for pixels              right bumper + dpad_up
             9.  quickly lower arm                              back button
             10. Reset GYRO to NORTH                            left stick button
             11. creep in direction of dpad buttons             dpad
             12.  Release the pixel                             right trigger
             13.  second driver releases and raises the lift    gamepad 2 - Y
             14.  second driver releases the drone              gamepad 2 - b
============================================================================================
*/

    @Override
    protected void run_9808() {

    }

    @Override
    public void runOpMode() {
        initialize();
        teleOpMode();
    }

    @Override
     public void initialize() {
        driveBase.init(hardwareMap, this);// initialize hardware
        driveBase.setSolidBlueLED();
        driveBase.arm.setPower(.8);
        driveBase.lift.setPower(.8);
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.holder.setPosition(driveBase.holderClosed);
        updateTelemetry();
    }

    private void teleOpMode() {
        driveBase.runtime.reset();
        driveBase.setBlueHeartbeatLED();

        while (opModeIsActive()) {
            // get the steering commands from either gamepad #1 or #2
            yCommand = (-gamepad1.left_stick_y - gamepad2.left_stick_y) / speedFactor; // forward and backward with respect to robot
            // (note: The joystick goes negative when pushed up, so we negate it)
            xCommand = (gamepad1.left_stick_x + gamepad2.left_stick_x) / speedFactor;  // left and right with respect to robot
            r = (-gamepad1.right_stick_x - gamepad2.right_stick_x) / 3 * speedFactor;        // spin cw or ccw

            createSteeringDeadzoneForJoystick();

            retrieveRobotHeadingFromIMU();

            setInputsBasedOnFieldCentricDriving();

            toggleDrivingMode();

            toggleDrivingSpeed();

            dpadCreep();

//================= gripper and arm down to collect/release the grip -- button a
            gripperAndArmDown();

//================= Raise arm to deliver pixel into lower zone on backdrop
//          + orient perpendicular to the backdrop, at the delivery distance.
            gripperAndArmToDeliveryPosition();

//================= lower and retract the arm after delivering pixels -- back button
            gripperAndArmToReadyPosition();

// ----->>> Use the left bumper and right trigger to raise/lower the arm
            armManualControl();

// ----->>> display parameters in diagnostic mode
            displayDiagnostics();


//==================== put everything in motion.. calculate tilt adjustment ==========================
            driveBase.applyMecPower2(x, y, r);
        }
    }

    private void armManualControl() {
        if (gamepad2.left_trigger > .1 || gamepad1.left_trigger > .1) {
            driveBase.armNewTargetPosition = driveBase.arm.getCurrentPosition() - driveBase.armIncrement;
            //driveBase.armNewTargetPosition -= .8*driveBase.armIncrement;
            if (driveBase.armNewTargetPosition < driveBase.armLowered) {
                driveBase.armNewTargetPosition = driveBase.armLowered;
            }
            driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
            sleep(50);

        } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
            driveBase.armNewTargetPosition = driveBase.arm.getCurrentPosition() + driveBase.armIncrement;
            if (driveBase.armNewTargetPosition > driveBase.armup) {
                driveBase.armNewTargetPosition = driveBase.armup;
            }
            driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
            sleep(50);
        }
        if (gamepad2.right_trigger > .1 || gamepad1.right_trigger > .1) {
            driveBase.grip.setPosition(driveBase.gripClosed);
            sleep(200);
        }
        if (gamepad1.x) {
            driveBase.grip.setPosition(driveBase.gripOpened);
            sleep(200);
        }
    }

    private void gripperAndArmToReadyPosition() {
        if (gamepad1.back || gamepad2.back) {
            driveBase.tilt.setPosition(driveBase.tiltToCarry);
            sleep(300);
            driveBase.armNewTargetPosition = driveBase.armLowered;
            driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
            driveBase.grip.setPosition(driveBase.gripClosed);
        }
    }

    private void gripperAndArmToDeliveryPosition() {
        if (gamepad1.b && !gamepad1.right_bumper) {
            driveBase.gyroTurn(.5, 90);
            distanceFromBackDrop = driveBase.frontDistance();// prepare to drive back
            if (distanceFromBackDrop < 25) {
                driveBase.tankDriveCorrected(.3, (10. - distanceFromBackDrop), 90);
                driveBase.gyroTurn(.5, 90);
                driveBase.armToLow();
                driveBase.tilt.setPosition(driveBase.tiltToRelease);
            }
        }

        if (gamepad1.y && !gamepad1.right_bumper) {
            driveBase.gyroTurn(.5, 90);
            distanceFromBackDrop = driveBase.frontDistance();// prepare to drive back
            if (distanceFromBackDrop < 25) {
                driveBase.tankDriveCorrected(.3, (10. - distanceFromBackDrop), 90);
                driveBase.gyroTurn(.5, 90);

                driveBase.armToMid();
                driveBase.tilt.setPosition(driveBase.tiltToRelease);
            }
        }

        if (gamepad1.y && gamepad1.right_bumper) {
            driveBase.gyroTurn(.5, 90);
            distanceFromBackDrop = driveBase.frontDistance();// prepare to drive back
            if (distanceFromBackDrop < 25) {
                driveBase.tankDriveCorrected(.3, (10. - distanceFromBackDrop), 90);
                driveBase.gyroTurn(.5, 90);

                driveBase.armToTop();
                driveBase.tilt.setPosition(driveBase.tiltToRelease);
            }
        }
    }

    private void gripperAndArmDown() {
        if (gamepad2.y && !gamepad2.right_bumper) {

            driveBase.lift.setPower(.8);
            driveBase.liftRelease.setPosition(driveBase.liftReleaseOpen);
            driveBase.lift.setTargetPosition(driveBase.liftUp + 1400);

            int lastPosition = 0;
            int currentPosition = 0;
            int loops = 0;
            while (driveBase.lift.isBusy()) {
                currentPosition = driveBase.lift.getCurrentPosition();
                if (Math.abs(currentPosition - lastPosition) < 2) {
                    loops++;
                }
                if (loops > 5) {
                    driveBase.liftNewTargetPosition = driveBase.lift.getCurrentPosition();
                    driveBase.lift.setTargetPosition(driveBase.liftNewTargetPosition);
                    loops = 0;
                }
                lastPosition = currentPosition;
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
            driveBase.gyroTurn(.6, -85);
            driveBase.holder.setPosition(driveBase.holderOpen);
            sleep(500);
            driveBase.droneRelease.setPosition(driveBase.droneReleaseOpen);
        }
    }

    private void dpadCreep() {
        //================= dpad CREEPing  ( forward, rear, left, right)

        if ((gamepad1.dpad_up && !gamepad1.right_bumper) || gamepad2.dpad_up) {
            y = creepSpeed;
        }
        if ((gamepad1.dpad_down && !gamepad1.right_bumper) || gamepad2.dpad_down) {
            y = -creepSpeed;
        }
        if ((gamepad1.dpad_left && !gamepad1.right_bumper) || gamepad2.dpad_left) {
            x = -creepSpeed;
        }
        if ((gamepad1.dpad_right && !gamepad1.right_bumper) || gamepad2.dpad_right) {
            x = creepSpeed;
        }

        if (gamepad1.dpad_up && gamepad1.right_bumper) {
            driveBase.tilt.setPosition(driveBase.tiltToPick);
            driveBase.grip.setPosition(driveBase.gripClosed);
            driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
        }
        if (gamepad1.dpad_left && gamepad1.right_bumper) {
            driveBase.gyroTurn(.6, 0);
            driveBase.grip.setPosition(driveBase.gripClosed);
            driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
            driveBase.tilt.setPosition(driveBase.tiltToPick);

            if (driveBase.rightDistanceToWall() < 48) {
                driveBase.DriveSideways(.5, driveBase.rightDistanceToWall() - 16.5);
                driveBase.gyroTurn(.6, 0);
                driveBase.tankDrive(.4, driveBase.frontLeftDistance() - 8);
            }

        }


        if (gamepad1.dpad_right && gamepad1.right_bumper) {
            driveBase.gyroTurn(.6, -90);
            driveBase.grip.setPosition(driveBase.gripClosed);
            driveBase.arm.setTargetPosition(driveBase.armPickingPosition);
            driveBase.tilt.setPosition(driveBase.tiltToPick);

            if (driveBase.leftDistanceToWall() < 48) {
                driveBase.DriveSideways(.5, -(driveBase.leftDistanceToWall() - 17));
                driveBase.tankDrive(.5, driveBase.frontLeftDistance() - 8);
            }
        }
    }

    private void toggleDrivingSpeed() {
        //================= toggle the driving speed by pressing the left joystick  =============================

        if (gamepad1.left_stick_button && !drivingSpeedToggled) {
            driveBase.runtime.reset();
            driveFast = !driveFast;
            drivingSpeedToggled = true;
        } else if (!gamepad1.left_stick_button && !gamepad2.left_stick_button) {
            drivingSpeedToggled = false;
        }

        if (gamepad2.left_stick_button) {
            driveBase.imu.resetYaw();
            HeadingHolder.setHeading(0);
            driveBase.setSolidGoldLED();
        }

        if (driveFast) {
            speedFactor = 1.3;
        } else {
            speedFactor = 2.2;
        }
    }

    private void toggleDrivingMode() {
        //================= toggle the driving mode by pressing the right joystick  =============================

        if (gamepad1.right_stick_button && !drivingModeToggled) {
            fieldCentric = !fieldCentric;
            drivingModeToggled = true;
        } else if (!gamepad1.right_stick_button) {
            drivingModeToggled = false;
        }
    }

    private void setInputsBasedOnFieldCentricDriving() {
        if (fieldCentric) {
            x = xCommand * Math.cos(theta) + yCommand * Math.sin(theta);
            y = yCommand * Math.cos(theta) - xCommand * Math.sin(theta);
        } else {
            x = xCommand * xCommand * xCommand;
            y = yCommand * yCommand * yCommand;
        }

        if (!diagnosticMode) {
            if (fieldCentric) {
                telemetry.addLine("RightStickButton -> change Mode");
                telemetry.addLine("CURRENT ---------> FIELD-CENTRIC");
                telemetry.addLine("                                ");
                telemetry.addLine("                                ");
            } else {
                telemetry.addLine("RightStickButton -> change Mode");
                telemetry.addLine("CURRENT ---> ROBOT-CENTRIC");
                telemetry.addLine("                                ");
            }
        }
    }

    private void retrieveRobotHeadingFromIMU() {
        theta = driveBase.getFieldHeading() * Math.PI / 180.;// convert to 0-2Pi angle
        if (theta < 0) {
            theta = theta + 2. * Math.PI;
        }
    }

    private void createSteeringDeadzoneForJoystick() {
        if (Math.abs(yCommand) < deadZone) {
            yCommand = 0.;
        }
        if (Math.abs(xCommand) < deadZone) {
            xCommand = 0.;
        }
        if (Math.abs(r) < deadZone) {
            r = 0.;
        }
    }

    private void updateTelemetry() {
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
    }
}

