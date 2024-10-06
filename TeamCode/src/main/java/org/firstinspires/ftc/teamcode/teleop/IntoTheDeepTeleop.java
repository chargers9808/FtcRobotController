package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

@TeleOp(name = "Teleop", group = "Linear Opmode")
@Disabled

public class IntoTheDeepTeleop extends IntoTheDeepBase {
    //TODO: Defile controller operations
    /*
    ============================================================================================
    Driving Controls:
        Left Stick - Movement (forward, back, left, right)
        Right Stick - Rotation (clockwise, counterclockwise)
        DPad - Slow movement

                             ***METHODS IN IntoTheDeepBase***
        A :                     sweeperIn()
        B :                     sweeperOut()
        X :
        Y :
        Left bumper:
        Right bumper:
        A + Left bumper:
        A + Right bumper:
    ============================================================================================
    */

    double lastSavedAngle = HeadingHolder.getHeading();
    boolean fieldCentric = true;

    protected void initialize() {
        driveBase.init(hardwareMap, this);// initialize hardware
        driveBase.setSolidBlueLED();
        driveBase.arm.setPower(0.0);
        driveBase.slide.setPower(0.0);
        //driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        //driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        //driveBase.holder.setPosition(driveBase.holderClosed);
        updateTelemetry();
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

    @Override
    protected void run_9808() {
        while (opModeIsActive()) {
            teleOpMode();
        }
    }

    private void teleOpMode() {
        driveBase.runtime.reset();
        driveBase.setBlueHeartbeatLED();

        while (opModeIsActive()) {
            //Example code
            if (gamepad1.a) {
                sweeperInOn();
                sleep(200);
                sweeperInOff();
            }
            if (gamepad1.b) {
                sweeperOutOn();
                sleep(200);
                sweeperOutOff();
            }
        }
    }

    public void verticalArmMovement() {
        if (gamepad2.left_trigger > .1 || gamepad1.left_trigger > .1) {
            driveBase.armNewTargetPosition = driveBase.arm.getCurrentPosition() - driveBase.armIncrement;
            //driveBase.armNewTargetPosition -= .8*driveBase.armIncrement;
            if (driveBase.armNewTargetPosition < driveBase.armLowered) {
                driveBase.armNewTargetPosition = driveBase.armLowered;
            }
            // need to set the arm motor power first.
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
    }
}
