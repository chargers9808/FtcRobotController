package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

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

    @Override
    public void runOpMode() {
        initialize();
        teleOpMode();
    }

    private void initialize() {
        driveBase.init(hardwareMap, this);// initialize hardware
        driveBase.setSolidBlueLED();
        driveBase.arm.setPower(.8);
        driveBase.lift.setPower(.8);
        driveBase.droneRelease.setPosition(driveBase.droneReleaseClosed);
        driveBase.liftRelease.setPosition(driveBase.liftReleaseClosed);
        driveBase.holder.setPosition(driveBase.holderClosed);
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
            telemetry.addData("heading:   ", driveBase.robotFieldHeading());
            telemetry.addLine("Waiting for START....");
            telemetry.update();
        }
    }

    private void teleOpMode() {
        driveBase.runtime.reset();
        driveBase.setBlueHeartbeatLED();

        while (opModeIsActive()) {
            //Example code
            if(gamepad1.a){
                sweeperInOn();
                sleep(200);
                sweeperInOff();
            }
            if(gamepad1.b){
                sweeperOutOn();
                sleep(200);
                sweeperOutOff();
            }
        }
    }
}
