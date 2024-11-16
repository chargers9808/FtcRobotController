package org.firstinspires.ftc.teamcode.intothedeep.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.intothedeep.IntoTheDeepBase;

@TeleOp(name = "ITDTeleop", group = "Linear Opmode")

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

    private final double DEFAULT_ARM_POWER = 0.1;
    private final double NO_POWER = 0.0;
    private final int ARM_INCREMENT = 75;
    private final int SLIDE_INCREMENT = 75;
    private final double ARM_POWER = 0.8;
    private final double SLIDE_POWER = 0.8;

    protected double lastSavedAngle = HeadingHolder.getHeading();
    protected DriverControls controller = new DriverControls();

    @Override
    protected void initialize() {

        // Set diagnostic mode if GP1.a is pressed
        if( gamepad1.a) {
            diagnosticMode = true;
        }

        // Reset the Gyro if GP2.LS is pressed
        if (gamepad2.left_stick_button) {
            controller.resetGyro(driveBase);
        }
        updateTelemetry();
    }

    protected void pre_initialize() {
        driveBase.arm.setPower( DEFAULT_ARM_POWER );
        driveBase.slide.setPower( NO_POWER );
        controller.init(this);

    }

    private void updateTelemetry() {
        if (controller.fieldCentric) {
            telemetry.addLine(getColorString() + " TeleOp :  FIELD Centric");
        } else {
            telemetry.addLine(getColorString() + " TeleOp :  ROBOT Centric");
        }

        telemetry.addLine("Driving Speed is HIGH");
        telemetry.addLine("Press Driver 2 Left Stick button at any time to reset the gyro");

        telemetry.addLine("Diagnostic Mode (press A to toggle): " + diagnosticMode);
        telemetry.addData("Gyro initialized to:   ", lastSavedAngle);
        telemetry.addData("heading:   ", driveBase.getFieldHeading());
    }

    @Override
    protected void run_9808() {
        setStaticLED();
        while (opModeIsActive()) {
            displayDiagnostics();
            controller.updateSpeedFactor();
            controller.calculateDriveControls();
            controller.calculateDPadCreep();
            controller.updateDriveMode();
            controller.creepSpeed();
            processSweeper();

            // Preset to score
            if( gamepad1.y) {
                score(Basket.TOP);
            }

            if (gamepad1.x){
                pickup();
            }

            if (gamepad1.a) {
             sweeperOut();
            }

            if( gamepad1.b) {
                travel();
            }

            processArm();
            processSlide();
            controller.move(driveBase);
        }
        HeadingHolder.setHeading(driveBase.getFieldHeading());
    }

    private void processArm() {
        if( gamepad1.left_bumper ) {
            driveBase.incrementMotorSafe(driveBase.arm,-1 * ARM_INCREMENT, ARM_POWER, armTravelPosition, armLowered);
        } else if (controller.triggered(gamepad1.left_trigger)) {
            driveBase.incrementMotorSafe(driveBase.arm, ARM_INCREMENT, ARM_POWER, armTravelPosition, armLowered);
        }
    }

    private void processSlide() {
        if( gamepad1.right_bumper ) {
            driveBase.incrementMotorSafe(driveBase.slide, SLIDE_INCREMENT, SLIDE_POWER, slideOut, slideIn);
        } else if (controller.triggered(gamepad1.right_trigger)) {
            driveBase.incrementMotorSafe(driveBase.slide, -1 * SLIDE_INCREMENT, SLIDE_POWER, slideOut, slideIn);
        }
    }

    private void processSweeper() {
        if (gamepad2.a) {
            sweeperIn();
        } else if (gamepad2.b) {
            sweeperOut();
        }
//        else {
//            sweeperOff();
//        }
    }
}
