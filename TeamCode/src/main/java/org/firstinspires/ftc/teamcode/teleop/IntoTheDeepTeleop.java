package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;

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
    private final int ARM_INCREMENT = 20;
    private final int SLIDE_INCREMENT = 20;

    protected double lastSavedAngle = HeadingHolder.getHeading();
    protected DriverControls controller = new DriverControls();

    @Override
    protected void initialize() {
        // Set diagnostic mode if GP1.a is pressed
        if( gamepad1.a) {
            diagnosticMode = true;
        }

        // Reset the Gyro if GP2.LS is pressed
        controller.resetGyro(driveBase);
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
            processSweeper();

            // Preset to score
            if( gamepad1.y) {
                score(Basket.TOP);
            }

            if (gamepad1.x)
                pickup();

            if (gamepad1.a)
                prepareToTravel();

            if( gamepad1.b) {
                travel();
            }

            processArm();
            processSlide();
            controller.move(driveBase);
        }
    }

    private void processArm() {
        if( gamepad2.left_bumper ) {
            driveBase.incrementMotorSafe(driveBase.arm,-1 * ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
        } else if (controller.triggered(gamepad2.left_trigger)) {
            driveBase.incrementMotorSafe(driveBase.arm, ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
        }
    }

    private void processSlide() {
        if( gamepad2.right_bumper ) {
            driveBase.incrementMotorSafe(driveBase.slide, SLIDE_INCREMENT, .8, driveBase.slideOut, driveBase.slideIn);
        } else if (controller.triggered(gamepad2.right_trigger)) {
            driveBase.incrementMotorSafe(driveBase.slide, -1 * SLIDE_INCREMENT, .8, driveBase.slideOut, driveBase.slideIn);
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
