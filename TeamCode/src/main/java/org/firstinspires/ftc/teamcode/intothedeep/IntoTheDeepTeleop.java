package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.DataHolder;
import org.firstinspires.ftc.teamcode.FieldTracker;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;
import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

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
    private final int ARM_INCREMENT = 35;
    private final int SLIDE_INCREMENT = 75;
    private final double ARM_POWER = 0.8;
    private final double SLIDE_POWER = 0.8;
    private final int LIFT_INCREMENT = 250;
    private final double LIFT_POWER = .8;

    private boolean liftPos = false;

    protected double lastSavedAngle = DataHolder.getHeading();
    protected DriverControls controller = new DriverControls();
    private int gripRotation = 0;

    private final double BUTTON_TIMEOUT = .25;
    private double lastPressedTimeA;
    private double lastPressedTimeStart;
    private double lastPressedTimeLSB;

    @Override
    protected void initialize() {

        // Set diagnostic mode if GP1.a is pressed
        if( gamepad1.a) {
            diagnosticMode = true;
        }
        if( gamepad1.b) {
            resetOdometry();
        }

        // Reset the Gyro if GP2.LS is pressed
        updateTelemetry();
        gripRotation = 0;
    }

    protected void pre_initialize() {
        driveBase.arm.setPower( DEFAULT_ARM_POWER );
        driveBase.slide.setPower( NO_POWER );
        controller.init(this);
        diagnosticMode=true;

    }

    protected void resetOdometry() {
        initOdometry();

        if( driveBase.distanceToWall(DraculaBase.SensorDir.RIGHT) < 100 ) {
            FieldTracker.setBotRef(POS_OBS_START);
            //FieldTracker.findPosition(DraculaBase.SensorDir.RIGHT);
        } else if( driveBase.distanceToWall(DraculaBase.SensorDir.LEFT) < 100 ) {
            FieldTracker.setBotRef(POS_NET_START);
            //FieldTracker.findPosition(DraculaBase.SensorDir.LEFT);
        }
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
        if( DataHolder.getOdometryEnabled() ) {
            telemetry.addLine("Odometry initialized. Press B to re-initialize");
        } else {
            telemetry.addLine("Press B to initialize odometry");
        }
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
        lastPressedTimeLSB = getRuntime() - 1000;
        while (opModeIsActive()) {
            displayDiagnostics();
            if( controller.updateSpeedFactor() ) {
                if( controller.getCurrentSpeed() == controller.SPEED_FACTOR_SLOW ) {
                    driveBase.setLED( DraculaBase.LEDColor.BLUE );
                } else {
                    driveBase.setLED( DraculaBase.LEDColor.OFF );
                }
            }
            controller.calculateDriveControls();
            //controller.calculateDPadCreep();
            controller.updateDriveMode();
            controller.creepSpeed();

            // Preset to score
            if( gamepad1.y ) {
                processScore();
            }

            if (gamepad1.x){
                processPickup();
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

            processArm();
            processSlide();
            processLift();
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
        if(controller.triggered(gamepad1.right_trigger)) {
            driveBase.incrementMotorSafe(driveBase.slide, SLIDE_INCREMENT, SLIDE_POWER, slideOut, slideIn);
        } else if (gamepad1.right_bumper) {
            driveBase.incrementMotorSafe(driveBase.slide, -1 * SLIDE_INCREMENT, SLIDE_POWER, slideOut, slideIn);
        }
    }

    private void processLift() {
        if( gamepad1.dpad_left ) {
            driveBase.moveMotor(driveBase.lift, liftOut, LIFT_POWER, true);
        }
        if( gamepad1.dpad_right ) {
            driveBase.moveMotor(driveBase.lift, liftIn, LIFT_POWER, true);
        }
        if( gamepad1.dpad_up ) {
            driveBase.incrementMotorSafe(driveBase.lift, LIFT_INCREMENT, LIFT_POWER, liftIn, liftOut);
        }
        if( gamepad1.dpad_down ) {
            driveBase.incrementMotorSafe(driveBase.lift, -1* LIFT_INCREMENT, LIFT_POWER, liftIn, liftOut);
        }
    }

    private void processScore() {
        final double Y_BOUNDARY = 48;
        Pose2DGobilda pos = getLocation();
        if( pos.getY(DistanceUnit.INCH) < Y_BOUNDARY) {
            driveBase.setLED(Position.posColor(Position.Location.NET));
            scoreOdo(Basket.TOP);
        } else {
            driveBase.setLED(Position.posColor(Position.Location.OBSERVATION));
            hangSpecimen();
        }
        driveBase.setLED(DraculaBase.LEDColor.OFF);
    }

    private void processPickup() {
        final double Y_BOUNDARY = 90;
        Pose2DGobilda pos = getLocation();
        boolean pointed180 = ( pos.getHeading(AngleUnit.DEGREES) >= 135 && pos.getHeading(AngleUnit.DEGREES) <= 225 );
        if( pos.getY(DistanceUnit.INCH) < Y_BOUNDARY || !pointed180 ) {
            driveBase.setLED(Position.posColor(Position.Location.NET));
            pickup();
        } else {
            driveBase.setLED(Position.posColor(Position.Location.OBSERVATION));
            pickupSpecimenFromWall();
        }
        driveBase.setLED(DraculaBase.LEDColor.OFF);
    }
}
