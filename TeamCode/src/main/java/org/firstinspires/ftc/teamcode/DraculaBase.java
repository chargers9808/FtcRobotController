
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;



public class DraculaBase {

    // --------------  hardware devices  --------------
    public DcMotor leftFront,rightFront,leftRear,rightRear, arm,lift;
    public Servo grip,tilt,liftRelease,droneRelease,holder;
    public ColorSensor leftColor,rightColor;
    public DigitalChannel leftTouch,rightTouch;
    public DistanceSensor revRangeLeft,revRangeRight,revRangeLeftFront,revRangeRightFront;
    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public IMU imu;
    //public ModernRoboticsI2cRangeSensor frontRange;


    // --------------  Vision and TensorFLow
    public String ObjectName;
    public float leftEdge;
    public TfodProcessor tfod;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ChargerCustomModelCubes.tflite";
    private static final String[] LABELS =
            {
                    "red prop",
                    "blue prop"
            };
    public VisionPortal visionPortal;
    private Acceleration gravity;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public boolean targetVisible = false;
    public double detectionTime=0;
    public String propPosition;
    public double midPosition;
    public double cameraOffset=4;// how far is the camera from the robot center line?

    // --------------  drive system and controls
    static final double COUNTS_PER_REV_gobilda435 = 384.5;    // Gobilda 435 rpm motors
    static final double WHEEL_CIRCUMFERENCE = (96./25.4) * Math.PI;//  circumference in inches
    boolean speedControl;// set this for true=fast, false = slow
    static final double COUNTS_PER_REVOLUTION_TORQUENADO = 1440.;    // eg: torquenado Motor Encoder
    static final double COUNTS_PER_REV_gobilda312 = 537.7;    // Gobilda 312 rpm motors

    static final double COUNTS_PER_INCH_435 = COUNTS_PER_REV_gobilda435 / WHEEL_CIRCUMFERENCE;// counts/inch travelled
    double ticksPerInch = COUNTS_PER_REV_gobilda435 / WHEEL_CIRCUMFERENCE;// counts/inch travelled

    public double turnFactor=1.25;
    double y = 0.0;
    double x = 0.0;
    double r = 0.0;
    static final double driveSpeed = 0.9;
    double max = 1.0;
    double lfrontpower = 0.0;
    double rfrontpower = 0.0;
    double lrearpower = 0.0;
    double rrearpower = 0.0;

    // --------------  IMU related... orientation
    public double HEADING_THRESHOLD = 1.3;//set at 1.2 normall
    public double initialFieldheading;

// --------------  Servos, arm and lift parameters

    public double gripPosition=.75;
    public double gripClosed=.73;// changed on oct 17 for new claw
    public double gripOpened=.83;// changed on oct 17
    public double gripIncrement=.00003;


    public double tiltIncrement = .00005;
    public double tiltPosition=.0857;//.146
    public double tiltVertical=.0857;//.146
    public double tiltToPick=.080;//.0085
    public double tiltToCarry=.1897;//.25
    public double tiltToRelease=.2937;//.354

    public double droneReleasePosition=.2;
    public double droneReleaseOpen=.1;
    public double droneReleaseClosed=.34;
    public double droneReleaseIncrement=.00005;

    public double liftReleaseOpen=.73;
    public double liftReleaseClosed=.53;
    public double liftReleaseIncrement=.00005;
    public double liftReleasePosition=.5;

    public double holderOpen=.18;
    public double holderClosed=.26;
    public double holderIncrement=.00005;
    public double holderPosition=.25;

    public double liftPower = 0.8;
    int liftNewTargetPosition=0;
    public int liftUp=2200;
    int liftDown=0;
    int liftIncrement=30;



    double armPower = 0.8;
    public int armIncrement=30;
    public int armLowered=0;
    int armJustAboveFirstLine=700;
    public int armJustAboveSecondLine=1358;
    int armJustAboveThirdLine=2100;
    public int armup=2200;
    public int armPickingPosition=150;
    int armNewTargetPosition=20;
    int  lastArmPosition=0;
    int  currentArmPosition=0;


    public double lineUpOnLeftColumnBlue=22;
    public double lineUpOnCenterColumnBlue=28.5;
    public double lineUpOnRightColumnBlue=35;

    public double lineUpOnLeftColumnRed=40;
    public double lineUpOnCenterColumnRed=28.5;
    public double lineUpOnRightColumnRed=21;

// --------------  Java, Logic, object oriented...

    OpMode callingOpMode;
    boolean achanged = false;
    boolean ychanged = false;
    boolean xchanged = false;
    boolean bchanged = false;
    boolean endGame = false;
    HardwareMap hwMap = null;
    double theta;
    public ElapsedTime runtime = new ElapsedTime();
    final double SCALE_FACTOR = 255;
    protected static final double     P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    protected static final double P_DRIVE_COEFF = 0.05;


    /*        METHODS         */

    public void initIMU(HardwareMap ahwMap, OpMode _callingOpMode) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        callingOpMode = _callingOpMode;
        imu = callingOpMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


        blinkinLedDriver = callingOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        blinkinLedDriver.setPattern(pattern);
    }

    public void init(HardwareMap ahwMap, OpMode _callingOpMode) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        callingOpMode = _callingOpMode;

// Define and Initialize Motors and sensors

        rightFront = callingOpMode.hardwareMap.dcMotor.get("fr");
        leftFront = callingOpMode.hardwareMap.dcMotor.get("fl");
        rightRear = callingOpMode.hardwareMap.dcMotor.get("br");
        leftRear = callingOpMode.hardwareMap.dcMotor.get("bl");
        arm = callingOpMode.hardwareMap.dcMotor.get("arm");
        lift = callingOpMode.hardwareMap.dcMotor.get("lift");

        grip = callingOpMode.hardwareMap.get(Servo.class, "grip");
        holder = callingOpMode.hardwareMap.get(Servo.class, "holder");
        tilt = callingOpMode.hardwareMap.get(Servo.class, "tilt");
        liftRelease = callingOpMode.hardwareMap.get(Servo.class, "liftrelease");
        droneRelease = callingOpMode.hardwareMap.get(Servo.class, "dronerelease");


        revRangeLeft = callingOpMode.hardwareMap.get(DistanceSensor.class, "revrangeleft");
        revRangeRight = callingOpMode.hardwareMap.get(DistanceSensor.class, "revrangeright");
        //revRangeRear = callingOpMode.hardwareMap.get(DistanceSensor.class, "revrangerear");
        revRangeLeftFront = callingOpMode.hardwareMap.get(DistanceSensor.class, "revrangeleftfront");
        revRangeRightFront = callingOpMode.hardwareMap.get(DistanceSensor.class, "revrangerightfront");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0.);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setPower(0.);

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setPower(0.);

        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setPower(0.);

        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(.6);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(.8);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu = callingOpMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        blinkinLedDriver = callingOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        blinkinLedDriver.setPattern(pattern);

    }

    public void applyMecPower2(double x, double y, double r) {

        lfrontpower = +y + x - r;
        rfrontpower = y - x + r;
        lrearpower = y - x - r;
        rrearpower = +y + x + r;

// Normalize the values so none can exceed +/- 1.0.... set "max" to any power found to be > 1.0
        double max = 1.0;
        max = Math.max(max, Math.abs(rfrontpower));
        max = Math.max(max, Math.abs(lfrontpower));
        max = Math.max(max, Math.abs(rrearpower));
        max = Math.max(max, Math.abs(lrearpower));

// normalize all the powers to this max level (keeping proportions and signs)
        rrearpower = rrearpower / max;
        lrearpower = lrearpower / max;
        rfrontpower = rfrontpower / max;
        lfrontpower = lfrontpower / max;

// apply the normalized power levels to each motor
        leftFront.setPower(lfrontpower);
        rightFront.setPower(rfrontpower);
        leftRear.setPower(lrearpower);
        rightRear.setPower(rrearpower);
    }



    public void examineMarker( double timelimit) {
        runtime.reset();
        if (((LinearOpMode) callingOpMode).opModeIsActive()) {
            while (((LinearOpMode) callingOpMode).opModeIsActive()&& runtime.seconds() < timelimit) {
                if (tfod != null) {

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    ObjectName="none";
                    if (currentRecognitions != null) {
                        //callingOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : currentRecognitions) {
                            ObjectName=recognition.getLabel();
                            leftEdge=recognition.getLeft();
                            if(ObjectName.equals("Duck")){break;}
                        }
                        //callingOpMode.telemetry.update();
                        break;
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    public void gyroTurn(double speed, double targetAngle) {

        /*
         *  gyroTurn is a method to rotate to a field direction.
         *  Move will stop if either of these conditions occur:
         *  1) the current heading gets within the HEADING_THRESHOLD of the target angle
         *  2) Driver presses stop
         *
         * @param speed            Desired speed of turn.
         * @param Targetangle      Target Field Angle (in Degrees)
         */

        double error;
        error = targetAngle - robotFieldHeading();   // how far we need to turn

// ================== adjust for crossing the zenith angle, also turn thru the shortest angle
        if (error > 180.) {
            error -= 360.;
        } else if (error <= -180.) {
            error += 360.;
        }
// ==================

// loop until the absolute value of the "error" (the target angle - the robot heading) < HEADING THRESHOLD

        while (((LinearOpMode) callingOpMode).opModeIsActive() && Math.abs(error) > HEADING_THRESHOLD) {

            // ================== adjust for crossing the zenith angle, also turn thru the shortest angle
            error = targetAngle - robotFieldHeading();// how far we need to turn

            if (error > 180.) {
                error -= 360.;
            } else if (error <= -180.) {
                error += 360.;
            }
            // ==================

            if (error < 0) {
                r = -speed;
            } else {
                r = +speed;
            }         // rotate CW or CCW depending on how the motors are set up

            if (Math.abs(error) < 50.) {
                r = r * Math.abs(error) / 50.;
            }  // scale down the rotation speed proportionate to error

            if (Math.abs(r) <= .07) {
                r = .07 * Math.abs(r) / r;
            }           // set a minimum r (turning speed), preserving the sign of r

            x = 0.;             // make sure we are stopped while rotating
            y = 0.;
            applyMecPower();    // set the motor speed using r
        }
        r = 0;
        stopMotors();  // the turn is complete within the HEADING_THRESHOLD
        // HeadingHolder.setHeading(robotFieldHeading());
    }
//------------------------------------------------------------------------------------------

    public void stopMotors() {
        x = 0.;
        y = 0.;
        r = 0.;
        applyMecPower();
    }

    public void PositionArmToCarryPixel() {
        // from
        armNewTargetPosition= armLowered+30;
        ((LinearOpMode) callingOpMode).sleep(300);
        tilt.setPosition(tiltToCarry);
        ((LinearOpMode) callingOpMode).sleep(300);
        armNewTargetPosition= armLowered;
    }

    public double Turn(double speed, double targetAngle) {
        double error;   // the difference between the current heading and the target angle.
        double rotation = 0.;

        // ================== adjust for crossing the zenith angle, also turn thru the shortest angle
        error = targetAngle - robotFieldHeading();   // how far we need to turn

        if (error > 180.) {
            error -= 360.;
        } else if (error <= -180.) {
            error += 360.;
        }
        // ==================


        // loop until the absolute value of the "error" (the target angle - the robot heading) < HEADING THRESHOLD

        if (Math.abs(error) > HEADING_THRESHOLD) {

            if (error < 0) {
                rotation = -speed;
            } else {
                rotation = speed;
            }         // rotate CW or CCW depending on how the motors are set up

            if (Math.abs(error) < 40.) {
                rotation = rotation * Math.abs(error) / 40.;
            }  // scale down the rotation speed proportionate to error
            if (Math.abs(rotation) <= .15) {
                rotation = .15 * Math.abs(rotation) / rotation;
            }                               // set a minimum r (turning speed), preserving the sign of r


        } else if (Math.abs(error) <= HEADING_THRESHOLD) {
            rotation = 0.;
            stopMotors();
        }
        return rotation;
    }
    public void applyMecPower() {


        lfrontpower = +y + x - r;
        rfrontpower = y - x + r;
        lrearpower = y - x - r;
        rrearpower = +y + x + r;

// Normalize the values so none can exceed +/- 1.0.... set "max" to any power found to be > 1.0
        max = 1.0;
        max = Math.max(max, Math.abs(rfrontpower));
        max = Math.max(max, Math.abs(lfrontpower));
        max = Math.max(max, Math.abs(rrearpower));
        max = Math.max(max, Math.abs(lrearpower));

// normalize all the powers to this max level (keeping proportions and signs)
        rrearpower = rrearpower / max;
        lrearpower = lrearpower / max;
        rfrontpower = rfrontpower / max;
        lfrontpower = lfrontpower / max;

// apply the normalized power levels to each motor
        leftFront.setPower(lfrontpower);
        rightFront.setPower(rfrontpower);
        leftRear.setPower(lrearpower);
        rightRear.setPower(rrearpower);
    }

    public void initVision() {
        initTfod();

        if (tfod != null) {
            visionPortal.setProcessorEnabled(tfod, true);
            tfod.setZoom(1.);
        }
    }
    public void pickUpPixel()
    {
        //arm.setTargetPosition(armPickingPosition+500);// move the arm to a slightly raised position
        // while(arm.isBusy()){}
        arm.setPower(armPower-.2);
        tilt.setPosition(tiltToPick);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        while(arm.isBusy()){

        }
        grip.setPosition(gripOpened);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered+50);
        ((LinearOpMode) callingOpMode).sleep(300);
        tankDrive(.3,-3);
        arm.setTargetPosition(armLowered+100);

        tilt.setPosition(tiltToCarry);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        arm.setPower(armPower);
    }

    public void newpickUpPixel() {
        int loops=0;
        boolean stillMoving=true;
        lastArmPosition=arm.getCurrentPosition();
        //arm.setTargetPosition(armPickingPosition+500);// move the arm to a slightly raised position
        // while(arm.isBusy()){}
        arm.setPower(armPower-.2);
        tilt.setPosition(tiltToPick);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        lastArmPosition = arm.getCurrentPosition();
        loops = 0;
        while(arm.isBusy() && stillMoving) {
            ((LinearOpMode) callingOpMode).sleep(10);// pause to let the motor move
            currentArmPosition = arm.getCurrentPosition();
            // check for stall... if stalled set target position to current position
            if (Math.abs(currentArmPosition - lastArmPosition) < 2) {
                loops++;// count how many loops the arm has not moved
            }
            if (loops > 5) {
                arm.setTargetPosition((arm.getCurrentPosition()));//force completion of the motion
                loops = 0;
                stillMoving=false;// end the "while" loop
            }
            lastArmPosition = currentArmPosition;
        }

        grip.setPosition(gripOpened);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered+50);
        ((LinearOpMode) callingOpMode).sleep(300);
        tankDrive(.3,-3);
        arm.setTargetPosition(armLowered+100);

        tilt.setPosition(tiltToCarry);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        arm.setPower(armPower);
    }
    public void deliverPixelLower()
    {
        arm.setTargetPosition(armJustAboveFirstLine);
        while(arm.isBusy()){}
        tilt.setPosition(tiltToRelease);

    }

    public void armToLow()
    {
        arm.setTargetPosition(armJustAboveFirstLine);
        while(arm.isBusy()){}
        tilt.setPosition(tiltToRelease);
    }
    public void blueAutoDropPixel()
    {
        gyroTurn(.5,90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}

        if(frontDistance()<15) {
            tankDrive(.3, (frontDistance() - 7));
        }

        gyroTurn(.5,90);

    }



    public void blueAutoDropPixelNoDriving()
    {
        gyroTurn(.5,90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}
        tankDrive(.3, 3.0);
        gyroTurn(.5,90);

    }

    public void blueAutoDropPixelNoDrivingHigh()
    {
        gyroTurn(.5,90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(700);
        while(arm.isBusy()){}
        tankDrive(.3, 3.0);
        gyroTurn(.5,90);

    }

    public void redAutoDropPixelNoDrivingHigh()
    {
        gyroTurn(.5,-90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(700);
        while(arm.isBusy()){}
        tankDrive(.3, 3.0);
        gyroTurn(.5,-90);

    }
    public void redAutoDropPixelNoDriving()
    {
        gyroTurn(.5,-90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}
        tankDrive(.3, 3.0);
        gyroTurn(.5,-90);

    }

    public void blueAutoDropPixelOld()
    {
        gyroTurn(.5,90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}
        tankDrive(.3,(frontDistance()-7));
        gyroTurn(.5,90);

    }

    public void redAutoDropPixel()
    {
        gyroTurn(.5,-90);
        tilt.setPosition(tiltToRelease);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}

        if(frontDistance()<15) {
            tankDrive(.3, (frontDistance() - 7));
        }
        gyroTurn(.5,-90);

    }

    public void blueAutoDropPixelRightColumn()
    {
        gyroTurn(.5,90);
        tilt.setPosition(tiltToRelease);
        ((LinearOpMode) callingOpMode).sleep(500);
        arm.setTargetPosition(450);
        while(arm.isBusy()){}
        tankDrive(.3,frontDistance()-5.5);
        gyroTurn(.5,90);

    }
    public void blueAutoDropPixelRightColumnFromWing()
    {
        gyroTurn(.5,90);
        DriveSidewaysCorrected(.3,-30,90);
        tilt.setPosition(tiltToRelease);
        gyroTurn(.5,90);
        tankDrive(.3,frontDistance()-5.5);
        DriveSideways(.3,10);
        gyroTurn(.5,90);
        arm.setTargetPosition(455);
        while(arm.isBusy()){}

    }

    public void blueAutoDropPixelCenterColumnFromWing()
    {
        gyroTurn(.5,90);
        DriveSidewaysCorrected(.3,-32,90);
        tilt.setPosition(tiltToRelease);
        gyroTurn(.5,90);
        tankDrive(.3,frontDistance()-5.5);

        gyroTurn(.5,90);
        arm.setTargetPosition(455);
        while(arm.isBusy()){}

    }
    public void blueAutoDropPixelLeftColumnFromWing()
    {
        gyroTurn(.5,90);
        DriveSidewaysCorrected(.3,-35,90);
        tilt.setPosition(tiltToRelease);
        gyroTurn(.5,90);

        arm.setTargetPosition(445);
        while(arm.isBusy()){}
        tankDrive(.3,frontDistance()-5.5);

    }

    public void redAutoDropPixelRightColumn()
    {
        gyroTurn(.5,-90);
        tilt.setPosition(tiltToRelease);
        ((LinearOpMode) callingOpMode).sleep(500);
        arm.setTargetPosition(500);
        while(arm.isBusy()){}
        //DriveSidewaysCorrected(.3,-34,90);
        gyroTurn(.5,-90);
        tankDrive(.3,frontDistance()-5.);
        gyroTurn(.5,-90);

    }


    public void armToMid()
    {
        arm.setTargetPosition(armJustAboveSecondLine);
        while(arm.isBusy()){}
        tilt.setPosition(tiltToRelease);
    }
    public void armToTop()
    {
        arm.setTargetPosition(armJustAboveThirdLine);
        while(arm.isBusy()){}
        tilt.setPosition(tiltToRelease);
    }


    public void setLED(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setRedStrobeLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setSolidGreenLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setVioletLED() {

        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setSolidBlueLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setSolidRedLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setSolidGoldLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        blinkinLedDriver.setPattern(pattern);
    }

    public void setWhiteLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriver.setPattern(pattern);
    }

    public void setBlueHeartbeatLED() {

        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setGoldHeartbeatLED() {

        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        blinkinLedDriver.setPattern(pattern);

    }

    public void setRedHeartbeatLED() {

        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        blinkinLedDriver.setPattern(pattern);

    }
    public void turnOffLEDs() {

        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        blinkinLedDriver.setPattern(pattern);

    }


    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(callingOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
/*
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        callingOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            callingOpMode.telemetry.addData(""," ");
            callingOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            callingOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            callingOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

 */

    public void PropDetection() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        callingOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if(x<midPosition) {
                propPosition = "Middle";
            }
            else if(x>midPosition){
                propPosition = "Right";
            }
            else {propPosition = "Left";}

            callingOpMode.telemetry.addData(""," ");
            callingOpMode.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            callingOpMode.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            callingOpMode.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


    public double robotFieldHeading() {
        double theta;// this method returns the field heading of the robot

        //  this gets all the imu parameters... the "heading" is the "firstAngle + initialFieldHeading"
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //theta = orientation.getYaw(AngleUnit.DEGREES) + HeadingHolder.getHeading();  // initialized with the saved heading
        theta = orientation.getYaw(AngleUnit.DEGREES);  // initialized with the saved heading

        if (theta < 0) {
            theta = theta + 360.;
        }
        if (theta > 360) {
            theta = theta - 360.;
        } // correct for the "wrap around" of this angle
        // this makes the angle 0-360 CCW same as the field angle, instead of 0-180 and then -180 to 0  (going CCW)
        return theta;

    }

    public void DriveSideways(double speed, double distance) {

        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            moveCounts = -(int) (distance * COUNTS_PER_INCH_435);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // Turn On RUN_TO_POSITION


            newLeftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() - moveCounts;
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;

            // Set Targets
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

            while (((LinearOpMode) callingOpMode).opModeIsActive() && leftFront.isBusy()) {
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftRear.setPower(speed);
                rightRear.setPower(speed);
            }
            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void setMotorsRWE() {


        leftFront.setDirection(DcMotor.Direction.REVERSE); //
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(0.);


        rightFront.setDirection(DcMotor.Direction.REVERSE);//
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setPower(0.);


        leftRear.setDirection(DcMotor.Direction.REVERSE); //
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setPower(0.);


        rightRear.setDirection(DcMotor.Direction.REVERSE);//
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setPower(0.);

    }


    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.08, 0.11, 0.15, 0.18, 0.22, 0.25,
                0.30, 0.36, 0.42, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }


    public void tankDrive(double speed, double distance){

        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double countsperinch=COUNTS_PER_INCH_435;  // convert from int to double
        double driveError;
        double Kp=.011;
        double speedReduction=1.;
        double approach;
        double scale=47.5/45.;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            if(distance<0){scale=1.0;}
            moveCounts = (int) (distance * countsperinch*scale);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            newLeftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() + moveCounts;
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;

            // Set Targets
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

// Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

// ==================

            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(speed);

            while (((LinearOpMode) callingOpMode).opModeIsActive() && leftFront.isBusy()) {}
            // Stop all motion;

            stopMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void tankDriveCorrected(double speed, double distance,double orientation){

        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double countsperinch=COUNTS_PER_INCH_435;  // convert from int to double
        double driveError;
        double Kp=.011;
        double speedReduction=1.;
        double approach;
        double scale=47.5/45.;
        double error;
        double correction;
        double P_COEFF=.03;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            if(distance<0){scale=1.0;}
            moveCounts = -(int) (distance * countsperinch*scale);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            newLeftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() + moveCounts;
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;

            // Set Targets
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

// Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

// ==================

            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(speed);

            while (((LinearOpMode) callingOpMode).opModeIsActive() && leftFront.isBusy()&& rightFront.isBusy()&& rightRear.isBusy()&& leftRear.isBusy()) {
                error = orientation - robotFieldHeading();

                while (error > 180) error = (error - 360);
                while (error <= -180) error = (error + 360);

                correction = Range.clip(error * P_COEFF, -1, 1);

                if(distance<0){correction *= -1.;}

                leftFront.setPower(speed-correction);
                rightFront.setPower(speed+correction);
                leftRear.setPower(speed-correction);
                rightRear.setPower(speed+correction);

                //((LinearOpMode) callingOpMode).sleep(20);
            }
            // Stop all motion;

            stopMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void updateDriveMotors(double frontleft, double frontright, double backleft, double backright) {
        rightFront.setPower(-frontright);
        rightRear.setPower(-backright);
        leftFront.setPower(frontleft);
        leftRear.setPower(backleft);
    }

    public void driveStraight(double inches, float heading, double power)  throws InterruptedException {
        double error;               //The number of degrees between the true heading and desired heading
        double correction;          //Modifies power to account for error
        double leftPower;           //Power being fed to left side of bot
        double rightPower;          //Power being fed to right side of bot
        double max;                  //To be used to keep powers from exceeding 1
        long loops = 0;
        heading = (int) normalize360(heading);

        resetEncoders(true);
        int target = (int) (inches * ticksPerInch);

        power = Range.clip(power, -1.0, 1.0);


        while (Math.abs(target) > Math.abs(leftFront.getCurrentPosition())  && ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - robotFieldHeading();

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            if (power<0){correction*=-.1;}

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }
            updateDriveMotors(leftPower, rightPower, leftPower, rightPower);


            if ((loops % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , robotFieldHeading());
                callingOpMode.telemetry.addData("encoder" , leftFront.getCurrentPosition());
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

        }
        updateDriveMotors(0, 0, 0, 0);
        ((LinearOpMode) callingOpMode).sleep(500);
    }
    public void resetEncoders(boolean isSpeed) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(isSpeed) {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            speedControl  = true;
        }
        else {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            speedControl  = false;
        }
    }

    public void driveStraightStall(double timeout, float heading, double power) throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double leftPower;                                       //Power being fed to left side of bot
        double rightPower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        long loops = 0;
        long loopsD = 0;
        double target = callingOpMode.getRuntime()+timeout;
        int pos = 0;
        int posLast;
        heading = (int) normalize360(heading);

        resetEncoders(false);

        power = Range.clip(power, -1.0, 1.0);


        while (target > callingOpMode.getRuntime() && ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - robotFieldHeading();

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }
            updateDriveMotors(leftPower, rightPower, leftPower, rightPower);

            if ((loops % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , r);
                callingOpMode.telemetry.addData("encoder" , pos);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            posLast = pos;
            pos = leftFront.getCurrentPosition();

            loops++;

            if ((loops > 10) && (Math.abs(pos-posLast) < 1)){
                if(loopsD<18){
                    loopsD++;
                } else {
                    callingOpMode.telemetry.addData("reaqftlpshdg", "sduiouxzfgbnho");
                    break;
                }
            } else {
                loopsD = 0;
            }

        }
        updateDriveMotors(0, 0, 0, 0);
        ((LinearOpMode) callingOpMode).sleep(100);
    }

    protected float normalize360(float val) {
        while (val > 360 || val < 0) {

            if (val > 360) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }


    public void strafe(double inches, float heading, double power, double timeout)  throws InterruptedException {
        double targetT = callingOpMode.getRuntime()+timeout;
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double frontPower;                                       //Power being fed to left side of bot
        double backPower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        long loops = 0;
        heading = (int) normalize360(heading);
        float zRotation= (float) robotFieldHeading();

        resetEncoders(false);
        int target = (int) (inches * ticksPerInch);

        power = Range.clip(power, -1.0, 1.0);


        while (Math.abs(target) > Math.abs(leftFront.getCurrentPosition())  && ((LinearOpMode) callingOpMode).opModeIsActive() && callingOpMode.getRuntime()<targetT) {

            zRotation= (float) robotFieldHeading();

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * .022, -1, 1);
            if(power<0){correction*=-1;}

            frontPower = power - correction;
            backPower = power + correction;

            max = Math.max(Math.abs(frontPower), Math.abs(backPower));
            if (max > 1.0) {
                backPower /= max;
                frontPower /= max;
            }

            if ((loops % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , zRotation);
                callingOpMode.telemetry.addData("encoder" , leftFront.getCurrentPosition());
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            updateDriveMotors(-frontPower, frontPower, backPower, -backPower);

            loops++;

        }
        updateDriveMotors(0, 0, 0, 0);
        ((LinearOpMode) callingOpMode).sleep(100);
    }

    public void improvedGyroTurn(double targetAngle) {

        /*
         *  gyroTurn is a method to rotate to a field direction.
         *  Move will stop if either of these conditions occur:
         *  1) the current heading gets within the HEADING_THRESHOLD of the target angle
         *  2) Driver presses stop
         *
         * @param speed            Desired speed of turn.
         * @param Targetangle      Target Field Angle (in Degrees)
         */

        double error= targetAngle - robotFieldHeading();   // how far we need to turn
        double lastHeading=robotFieldHeading();
        double T_P_COEFF = 0.011;
        double T_I_COEFF = 0.000;
        double T_D_COEFF = 0.00000;
        double deltaT;
        double derivativeT = 0;
        double integralT = 0;
        double deltaDT;
        long loops = 0;

        double lastTime = callingOpMode.getRuntime();

// ================== adjust for crossing the zenith angle, also turn thru the shortest angle
        if (error > 180.) {
            error -= 360.;
        } else if (error <= -180.) {
            error += 360.;
        }
// ==================

// loop until the absolute value of the "error" (the target angle - the robot heading) < HEADING THRESHOLD

        while (((LinearOpMode) callingOpMode).opModeIsActive() && Math.abs(error) > HEADING_THRESHOLD) {

            // ================== adjust for crossing the zenith angle, also turn thru the shortest angle
            error = targetAngle - robotFieldHeading();// how far we need to turn

            if (error > 180.) {
                error -= 360.;
            } else if (error <= -180.) {
                error += 360.;
            }
            // ==================

            deltaT = callingOpMode.getRuntime()-lastTime;
            lastTime = callingOpMode.getRuntime();
            deltaDT = robotFieldHeading()-lastHeading;
            lastHeading = robotFieldHeading();
            derivativeT =  deltaDT/deltaT;

            if(Math.abs(error*T_P_COEFF)<1){
                integralT += error*deltaT;
            } else {
                integralT = 0;
            }

            r = (error*T_P_COEFF) + (integralT*T_I_COEFF) - (derivativeT*T_D_COEFF);
            r = Range.clip(r, -1, 1.0);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("heading", robotFieldHeading());
                callingOpMode.telemetry.addData("error", error);
                callingOpMode.telemetry.addData("applied turn power", r);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }
            loops++;

            x = 0.;             // make sure we are stopped while rotating
            y = 0.;
            applyMecPower2(x,y,r);    // set the motor speed using r
            ((LinearOpMode) callingOpMode).sleep(10);
        }
        r = 0;
        stopMotors();  // the turn is complete within the HEADING_THRESHOLD

    }

    public void DriveSidewaysCorrected(double speed, double distance, double directionOfFront)  {

        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double correction;
        //double initialHeading=robotFieldHeading();

        // resetEncoders(false);
        int target = (int) (distance * ticksPerInch);

        speed = Range.clip(speed, -1.0, 1.0);
        double error;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            moveCounts = -(int) (distance * COUNTS_PER_INCH_435);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // Turn On RUN_TO_POSITION


            newLeftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
            newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = rightRear.getCurrentPosition() - moveCounts;
            newLeftRearTarget = leftRear.getCurrentPosition() + moveCounts;

            // Set Targets
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is running.

            while (((LinearOpMode) callingOpMode).opModeIsActive() && rightFront.isBusy() && leftFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
                error = directionOfFront - robotFieldHeading();

                while (error > 180) error = (error - 360);
                while (error <= -180) error = (error + 360);

                correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

                if(distance>0){correction *= -1.;}

                leftFront.setPower(speed+correction);
                rightFront.setPower(speed+correction);
                leftRear.setPower(speed-correction);
                rightRear.setPower(speed-correction);

                //((LinearOpMode) callingOpMode).sleep(20);

            }
            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (((LinearOpMode) callingOpMode).opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            callingOpMode.telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.


        leftFront.setPower(rightSpeed);
        rightFront.setPower(rightSpeed);
        leftRear.setPower(leftSpeed);
        rightRear.setPower(leftSpeed);

        // Display it for the driver.
        callingOpMode.telemetry.addData("Target", "%5.2f", angle);
        callingOpMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        callingOpMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        callingOpMode.telemetry.update();

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robotFieldHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public double leftDistanceToWall(){
        double lrange=500;
        lrange=revRangeLeft.getDistance(DistanceUnit.INCH);
        return lrange;
    }

/*
    public double frontDistance(){
        double frontrange=500;
        frontrange=revRangeFront.getDistance(DistanceUnit.INCH);
        return frontrange;
    }
    */

    public double frontLeftDistance(){
        double frontrange=500;
        frontrange = revRangeLeftFront.getDistance(DistanceUnit.INCH);

        return frontrange;
    }
    public double frontRightDistance(){
        double rfrontrange=500;
        rfrontrange = revRangeRightFront.getDistance(DistanceUnit.INCH);
        return rfrontrange;
    }

    public double frontDistance(){
        double frontrange=500;
        double tempRangeL=revRangeLeftFront.getDistance(DistanceUnit.INCH);
        double tempRangeR=revRangeRightFront.getDistance(DistanceUnit.INCH);
        // try three times to get a correct front distance measurement... otherwise park
        if(tempRangeL>15 && tempRangeR>15){
            ((LinearOpMode) callingOpMode).sleep(50);//small wait and measure again
            tempRangeL=revRangeLeftFront.getDistance(DistanceUnit.INCH);
            tempRangeR=revRangeRightFront.getDistance(DistanceUnit.INCH);
        }
        if(tempRangeL>15 && tempRangeR>15){
            ((LinearOpMode) callingOpMode).sleep(50);//small wait and measure again
            tempRangeL=revRangeLeftFront.getDistance(DistanceUnit.INCH);
            tempRangeR=revRangeRightFront.getDistance(DistanceUnit.INCH);
        }

        if (tempRangeL <= tempRangeR && tempRangeL<15) {
            frontrange = tempRangeL;
        } else if (tempRangeR < tempRangeL && tempRangeR<15){
            frontrange = tempRangeR;
        }

        return frontrange;
    }


    public double rightDistanceToWall(){
        double rrange=500;
        rrange=revRangeRight.getDistance(DistanceUnit.INCH);
        return rrange;
    }
/*
    public double leftFrontHue(){
        double hue=0;
        Color.RGBToHSV((int) (leftColor.red() * SCALE_FACTOR),
                (int) (leftColor.green() * SCALE_FACTOR),
                (int) (leftColor.blue() * SCALE_FACTOR),
                hsvValues);
        hue= (double) hsvValues[0];
        return hue;
    }

    public double rightFrontHue(){
        double hue=0;
        Color.RGBToHSV((int) (rightColor.red() * SCALE_FACTOR),
                (int) (rightColor.green() * SCALE_FACTOR),
                (int) (rightColor.blue() * SCALE_FACTOR),
                hsvValues);
        hue=(double) hsvValues[0];
        return hue;
    }

 */

    public void setDistanceToRightWall(double power, double standOffDistanceInches) {
        setMotorsRWE();

        while (((LinearOpMode) callingOpMode).opModeIsActive() && Math.abs(rightDistanceToWall() - standOffDistanceInches) > 1) {  //repeat until the Rev 2M sensor reports correct distance to right wall
            callingOpMode.telemetry.addData("rightDistance", rightDistanceToWall());
            double reducedPower = (rightDistanceToWall() - standOffDistanceInches) / standOffDistanceInches;

            if(Math.abs(reducedPower)<.1){reducedPower=Math.abs(reducedPower)*.1/reducedPower;}

            applyMecPower2(power*reducedPower,0,0);
            callingOpMode.telemetry.update();
        }
        // OK now wall is within  distance threshold --- all stop
        stopMotors();
    }

    public void setPowerShotPosition(double fromLeft, double fromRear) {
        //setMotorsRWE();

        //Turn(.7,-90);
        DriveSidewaysCorrected(.6,(-leftDistanceToWall()+fromLeft),0.);
        //     tankDrive(.8,(-rearDistanceToWall()+fromRear));
    }


    public void setDistanceToLeftWall(double power, double standOffDistanceInches) {
        setMotorsRWE();

        while (((LinearOpMode) callingOpMode).opModeIsActive() && Math.abs(leftDistanceToWall() - standOffDistanceInches) > 1) {  //repeat until the Rev 2M sensor reports correct distance to right wall
            callingOpMode.telemetry.addData("leftDistance", leftDistanceToWall());
            double reducedPower = (-leftDistanceToWall() + standOffDistanceInches) / standOffDistanceInches;

            if(Math.abs(reducedPower)<.1){reducedPower=Math.abs(reducedPower)*.1/reducedPower;}

            applyMecPower2(power*reducedPower,0,0);
            callingOpMode.telemetry.update();
        }
        // OK now wall is within  distance threshold --- all stop
        stopMotors();

    }

    public void setDistanceToShootPins(double power, double standOffDistanceInches) {
        //setMotorsRWE();

        if ( Math.abs(leftDistanceToWall() - standOffDistanceInches) > 1) {  //repeat until the Rev 2M sensor reports correct distance to right wall
            //callingOpMode.telemetry.addData("leftDistance", leftDistanceToWall());
            double reducedPower = (-leftDistanceToWall() + standOffDistanceInches) / standOffDistanceInches;

            if(Math.abs(reducedPower)<.1){reducedPower=Math.abs(reducedPower)*.1/reducedPower;}

            applyMecPower2(power*reducedPower,0,0);
            //callingOpMode.telemetry.update();
        }
        // OK now wall is within  distance threshold --- all stop
        //stopMotors();

    }

    public void smallArmAdjustment(int adjustment) {
        armNewTargetPosition=arm.getCurrentPosition()+adjustment;
        arm.setTargetPosition(armNewTargetPosition);
        ((LinearOpMode) callingOpMode).sleep(500);
    }


  // ------------------------- Blue Backdrop Pixel Plow ---------------------------------
    public void plowFromBlueBackdropStartToLeftSpike() { // Left
        tankDrive(.5,2);
        gyroTurn(.5,25);
        tankDrive(.5,17.45); 
        tankDrive(.5,-10);
        gyroTurn(.5,90);
    }
    public void plowFromBlueBackdropStartToCenterSpike() { // Center
        tankDrive(.5,2);
        gyroTurn(.3,10);
        tankDrive(.3,10);
        gyroTurn(.3,0);
        tankDrive(.5,14); 
        gyroTurn(.3,-5);
        tankDrive(.5,-12);
        gyroTurn(.5,90);

    }
    public void plowFromBlueBackdropStartToRightSpike(){ // Right
        tankDrive(.5,2);
        gyroTurn(.3,10);
        tankDrive(.5,10);
        gyroTurn(.3,0);
        tankDrive(.3,3);
        gyroTurn(.3,-15);
        tankDrive(.3,5);
        gyroTurn(.3,-45);
        tankDrive(.3,5);

        tankDrive(.5,-10);
        gyroTurn(.5,90);
    }
    // ---------------------------------- Red Wing Pixel Plow --------------------------------
    public void plowFromRedWingStartToCenterSpike() { // Center
        tankDrive(.5,2);
        gyroTurn(.3,10);
        tankDrive(.3,25.5);
        //gyroTurn(.3,0);
        //tankDrive(.3,13);
        gyroTurn(.3,-5);
        tankDrive(.5,-12);
        gyroTurn(.5,90);

    }
    public void plowFromRedWingStartToLeftSpike() { // Left
        tankDrive(.5,2);
        gyroTurn(.5,25);
        tankDrive(.5,19);
        tankDrive(.5,-10);
        gyroTurn(.5,0);
    }
    public void plowFromRedWingStartToRightSpike(){ // Right
        tankDrive(.5,2);
        gyroTurn(.3,10);
        tankDrive(.5,10);
        gyroTurn(.3,0);
        tankDrive(.3,3);
        gyroTurn(.3,-15);
        tankDrive(.3,5);
        gyroTurn(.3,-45);
        tankDrive(.3,5);

        tankDrive(.5,-10);
        gyroTurn(.5,90);
    }

//------------------------------ Red Backdrop Pixel Plow ------------------------------------
    public void plowFromRedRightStartToRightSpike() { // Right
        gyroTurn(.4,-16); // -3 angle 2/17/24
        tankDrive(.5,19.5); // +0.5 distance 2/17/24
        tankDrive(.3,-10);
        gyroTurn(.6,-90);
    }

    public void plowFromRedRightStartToCenterSpike(){ // Center
        tankDrive(.5,25);
        tankDrive(.3,-10);
        gyroTurn(.6,-90);
    }
    public void plowFromRedRightStartToLeftSpike(){ // Left
        tankDrive(.5,8);
        gyroTurn(.3,20);
        tankDrive(.3,8);
        gyroTurn(.3,40);
        tankDrive(.3,6.5); // +0.5 distance 2/17/24
        tankDrive(.5,-15);
        gyroTurn(.5,-90);
    }
    //------------------------- Blue Wing Pixel Plow -----------------------------
    public void plowFromBlueRightStartToRightSpike() { // Right
        tankDrive(.5,1);
        gyroTurn(.4,-13);
        tankDrive(.5,19);
        tankDrive(.3,-18); // AJB changed from -10 on 2/24

    }

    public void plowFromBlueRightStartToCenterSpike(){ // Center
        tankDrive(.3,26); // AJB changed from distance 25 on 2/24 and speed from .6 on 2/27
        tankDrive(.3,-10);

    }
    public void plowFromBlueRightStartToLeftSpike(){ // Left
        tankDrive(.5,8);
        gyroTurn(.3,20);
        tankDrive(.3,8);
        gyroTurn(.3,40);
        tankDrive(.3,6.5);
        tankDrive(.5,-10);

    }
}
