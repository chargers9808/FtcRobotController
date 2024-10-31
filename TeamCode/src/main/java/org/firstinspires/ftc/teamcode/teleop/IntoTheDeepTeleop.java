package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

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
        //driveBase.arm.setPower( DEFAULT_ARM_POWER );
        //driveBase.slide.setPower( NO_POWER );
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
//            if( gamepad1.y) {
//                score(Basket.TOP);
//            }

            if (gamepad1.x) {
                score();
                //moveSlide();\//\
            }

            if(gamepad1.y){
                retractSlide();
            }

            if(gamepad1.a){
                extendIntake();
            }

            if(gamepad1.b){
                retractIntake();
            }

//            if (gamepad1.b)
//                prepareToTravel();
//
//            if( gamepad1.a) {
//                travel();
//            }//

            processArm();
            processSlide();
            controller.move(driveBase);
        }
    }

    private void testRotateAndMove() {

    }

    private void processArm() {
//        if( gamepad2.left_bumper ) {
//            driveBase.incrementMotorSafe(driveBase.arm,-1 * ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
//        } else if (controller.triggered(gamepad2.left_trigger)) {
//            driveBase.incrementMotorSafe(driveBase.arm, ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
//        }
    }

    private void extendIntake(){
        driveBase.slide.setTargetPosition(977);
        driveBase.slide.setPower(0.8);

        driveBase.flipper.setPosition(driveBase.flipperOut);
        sleep(1250);
        driveBase.flipper.setPosition(driveBase.flipperOut2);


        driveBase.intake.setPower(0.3);
    }

    private void retractIntake(){

        driveBase.intake.setPower(0);
        driveBase.flipper.setPosition(driveBase.flipperIn1);

        driveBase.slide.setTargetPosition(0);
        driveBase.slide.setPower(0.8);

        sleep(500);

        driveBase.flipper.setPosition(driveBase.flipperIn);
    }

    private void score(){
        driveBase.intake.setPower(-0.3);
        sleep(1000);
        driveBase.intake.setPower(0.0);

        // lift
    }

    private void retractSlide(){
        driveBase.slide.setTargetPosition(0);
        driveBase.slide.setPower(0.8);
    }

    private void moveSlide() {
        driveBase.slide.setTargetPosition(977);
        driveBase.slide.setPower(0.8);
    }

    private void processSlide() { //  977
        if( gamepad1.right_bumper ) {
            driveBase.incrementMotorSafe(driveBase.slide, 30, 0.5, 0 , 90);
            sleep(30); // 2814 max
        } else if (controller.triggered(gamepad1.right_trigger)) {
            driveBase.incrementMotorSafe(driveBase.slide, -30, 0.5, 0 , 90);
            sleep(30); // 2814 max
            //bvlldriveBase.incrementMotorSafe(driveBase.slide, -1 * SLIDE_INCREMENT, .8, driveBase.slideOut, driveBase.slideIn);
        }
    }

    private void processSweeper() {
//        if (gamepad2.a) {
//            sweeperIn();
//        } else if (gamepad2.b) {
//            sweeperOut();
//        }
//        else {
//            sweeperOff();
//        }
    }
}
