package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.DataHolder;

@TeleOp(name = "ITDTeleop", group = "Linear Opmode")

public class IntoTheDeepTeleop extends IntoTheDeepBase {
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

    protected double lastSavedAngle = DataHolder.getHeading();
    protected DriverControls controller = new DriverControls();
    private int gripRotation = 0;

    private final double BUTTON_TIMEOUT = .25;
    private double lastPressedTimeA;
    private double lastPressedTimeStart;

    @Override
    protected void initialize() {

        // Set diagnostic mode if GP1.a is pressed
        if( gamepad1.a) {
            diagnosticMode = true;
        }

        // Reset the Gyro if GP2.LS is pressed
        updateTelemetry();
        gripRotation = 0;
    }

    protected void pre_initialize() {
        driveBase.arm.setPower( DEFAULT_ARM_POWER );
        driveBase.slide.setPower( NO_POWER );
        controller.init(this);

    }

    private boolean buttonPressed( boolean button, double lastPressed) {
        return button && (getRuntime() >= (lastPressed + BUTTON_TIMEOUT));
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
        telemetry.addData("Grip Rotation Position :", driveBase.gripRotation.getPosition());
    }

    @Override
    protected void opModeDiagnostics() {
        telemetry.addData("Gripper storage: ", gripRotation);
    }

    @Override
    protected void run_9808() {
        setStaticLED();
        setGripRotation(gripRotation );
        lastPressedTimeA = getRuntime() - 1000;
        lastPressedTimeStart = getRuntime() - 1000;
        while (opModeIsActive()) {
            displayDiagnostics();
            controller.calculateDriveControls();
            controller.calculateDPadCreep();
            controller.updateDriveMode();
            controller.creepSpeed();

            // Preset to score
            if( gamepad1.y) {
                score(Basket.TOP);
            }

            if (gamepad1.x){
                pickup();
            }

            if ( buttonPressed( gamepad1.a, lastPressedTimeA)) {
                gripRotation += 1;
                gripRotation %= GRIP_ROTATIONS.length;
                setGripRotation(gripRotation );
                lastPressedTimeA = getRuntime();
            }

            if( buttonPressed( gamepad1.start, lastPressedTimeStart)) {
                toggleGripper();
                lastPressedTimeStart = getRuntime();
            }

            if( gamepad1.b) {
                travel();
            }

            if (gamepad1.left_stick_button) {
                DataHolder.setHeading(0);
                driveBase.imu.resetYaw();
            }

            processArm();
            processSlide();
            controller.move(driveBase);
        }
        DataHolder.setHeading(driveBase.getFieldHeading());
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
}
