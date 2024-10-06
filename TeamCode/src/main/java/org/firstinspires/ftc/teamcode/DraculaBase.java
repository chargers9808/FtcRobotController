package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

public class DraculaBase {
    //region hardware devices
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, slide;
    public Servo grip, tilt, liftRelease, droneRelease, holder;
    public DistanceSensor revRangeLeft, revRangeRight, revRangeFront, revRangeRear;
    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public IMU imu;
    GoBildaPinpointDriver odometryComputer;
    //endregion

    // --------------  Vision and TensorFLow
    public double cameraOffsetFromCenterline = 4;// how far is the camera from the robot center line?

    // --------------  drive system and controls
    static final double COUNTS_PER_REV_gobilda435 = 384.5;    // Gobilda 435 rpm motors
    static final double WHEEL_CIRCUMFERENCE = (96. / 25.4) * Math.PI;//  circumference in inches
    static final double COUNTS_PER_INCH_435 = COUNTS_PER_REV_gobilda435 / WHEEL_CIRCUMFERENCE;// counts/inch travelled

    double y = 0.0;
    double x = 0.0;
    double r = 0.0;
    double max = 1.0;
    double lfrontpower = 0.0;
    double rfrontpower = 0.0;
    double lrearpower = 0.0;
    double rrearpower = 0.0;

    // --------------  IMU related... orientation
    public double HEADING_THRESHOLD = 1.3;//set at 1.2 normall

// --------------  Servos, arm and lift parameters

    public double gripPosition = .75;
    public double gripClosed = .73;// changed on oct 17 for new claw
    public double gripOpened = .83;// changed on oct 17

    public double tiltIncrement = .00005;
    public double tiltPosition = .0857;//.146
    public double tiltVertical = .0857;//.146
    public double tiltToPick = .080;//.0085
    public double tiltToCarry = .1897;//.25
    public double tiltToRelease = .2937;//.354

    public double droneReleasePosition = .2;
    public double droneReleaseOpen = .1;
    public double droneReleaseClosed = .34;

    public double liftReleaseOpen = .73;
    public double liftReleaseClosed = .53;
    public double liftReleasePosition = .5;

    public double holderOpen = .18;
    public double holderClosed = .26;
    public double holderPosition = .25;

    public int liftNewTargetPosition = 0;
    public int liftUp = 2200;
    public int liftIncrement = 30;

    public int slideNewTargetPosition = 20;
    public int slideOut = -1580;
    public int slideIncrement = 20;
    public int slideIn = 20;


    double armPower = .8;
    public int armIncrement = 20;
    public int armLowered = -50;
    public int armTravelPosition = -2000;
    public int armScoringPositon =  -1730;
    public int armup = 2200;
    public int armPickingPosition = 150;
    public int armNewTargetPosition = 50;

    public int armJustAboveSecondLine = 1;
    public int armJustAboveFirstLine = 1;
    public int armJustAboveThirdLine = 1;
    public int armForwardLimit = 40;
    public int armBackLimit = 60;
// --------------  Java, Logic, object oriented...

    OpMode callingOpMode;
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    protected static final double P_DRIVE_COEFF = 0.05;

    public void init(HardwareMap hardwareMap, OpMode _callingOpMode) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        callingOpMode = _callingOpMode;

        initAllMotors();
        setAllServos();
        setAllDistanceSensors();

        imu = getHardwareMap().get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        blinkinLedDriver = callingOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        blinkinLedDriver.setPattern(pattern);

        //gyroTurn(0.1, 260);
    }

    private HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setAllDistanceSensors(){
        revRangeLeft = getDistanceSensor("leftdistance");
        revRangeRight = getDistanceSensor("rightdistance");
        revRangeFront = getDistanceSensor("frontdistance");
        revRangeRear = getDistanceSensor("reardistance");
    }
    private DistanceSensor getDistanceSensor(String deviceName) {
        // Can't find distance sensor class in hardware map
        return getHardwareMap().get(DistanceSensor.class, deviceName);
    }

    private void setAllServos(){
//        grip = getCrServo("intake");
//        holder = getServo("holder");
//        tilt = getServo("tilt");
//        liftRelease = getServo("liftrelease");
//        droneRelease = getServo("dronerelease");
    }
    private Servo getCrServo(String deviceName) {
        return getHardwareMap().servo.get(deviceName);
    }

    private DcMotor getDcMotor(String deviceName) {
        return getHardwareMap().dcMotor.get(deviceName);
    }

    private void intiOdometryComputer(){
        odometryComputer = getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        odometryComputer.setOffsets(-84.0, -168.0);
        odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryComputer.resetPosAndIMU();

    }


    private void initAllMotors(){
        frontLeft = initMotor(getDcMotor("fl"), DcMotor.Direction.REVERSE, 0.0);
        frontRight = initMotor(getDcMotor("fr"), DcMotor.Direction.FORWARD, 0.0);
        backLeft = initMotor(getDcMotor("bl"), DcMotor.Direction.REVERSE, 0.0);
        backRight = initMotor(getDcMotor("br"), DcMotor.Direction.FORWARD, 0.0);

        arm = initMotor(getDcMotor("arm"), DcMotorSimple.Direction.FORWARD, 0.6, 0);
        slide = initMotor(getDcMotor("slide"), DcMotorSimple.Direction.FORWARD, 0.8, 0);
    }

    private DcMotor initMotor(DcMotor dcMotor, DcMotorSimple.Direction direction, double power) {
        dcMotor.setDirection(direction);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setPower(power);
        return dcMotor;
    }

    private DcMotor initMotor(DcMotor dcMotor, DcMotorSimple.Direction direction, double power, int position) {
        initMotor(dcMotor, direction, power);

        dcMotor.setTargetPosition(position);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return dcMotor;
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
        frontLeft.setPower(lfrontpower);
        frontRight.setPower(rfrontpower);
        backLeft.setPower(lrearpower);
        backRight.setPower(rrearpower);
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
        error = targetAngle - getFieldHeading();   // how far we need to turn

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
            error = targetAngle - getFieldHeading();// how far we need to turn

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


        setWheelMotorPower(0.0, 0.0, 0.0, 0.0);
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
        rrearpower /= max;
        lrearpower /= max;
        rfrontpower /= max;
        lfrontpower /= max;

// apply the normalized power levels to each motor

        setWheelMotorPower(lfrontpower, rfrontpower, lrearpower, rrearpower);
    }

    private void setWheelMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontRightPower);
        frontRight.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private double normalizeMaxPower(double maxPower, double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        return maxPower;
    }

    public void pickUpPixel() {
        arm.setPower(armPower - .2);
        tilt.setPosition(tiltToPick);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        while (arm.isBusy()) {
        }
        grip.setPosition(gripOpened);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered + 50);
        ((LinearOpMode) callingOpMode).sleep(300);
        tankDrive(.3, -3);
        arm.setTargetPosition(armLowered + 100);

        tilt.setPosition(tiltToCarry);
        ((LinearOpMode) callingOpMode).sleep(300);
        arm.setTargetPosition(armLowered);
        arm.setPower(armPower);
    }

    public void armToLow() {
        arm.setTargetPosition(armJustAboveFirstLine);
        while (arm.isBusy()) {
        }
        tilt.setPosition(tiltToRelease);
    }

    public void armToMid() {
        arm.setTargetPosition(armJustAboveSecondLine);
        while (arm.isBusy()) {
        }
        tilt.setPosition(tiltToRelease);
    }

    public void armToTop() {
        arm.setTargetPosition(armJustAboveThirdLine);
        while (arm.isBusy()) {
        }
        tilt.setPosition(tiltToRelease);
    }

    /**
     * Sets the active LED color {@link RevBlinkinLedDriver.BlinkinPattern}
     *
     * @param pattern the LED pattern to set
     */

    //region LED
    public void setLED(RevBlinkinLedDriver.BlinkinPattern pattern) {
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

    public void setBlueHeartbeatLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        blinkinLedDriver.setPattern(pattern);
    }

    public void setRedHeartbeatLED() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        blinkinLedDriver.setPattern(pattern);
    }
    //endregion


    public double getFieldHeading() {
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
            newLeftFrontTarget = frontLeft.getCurrentPosition() - moveCounts;
            newRightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
            newRightRearTarget = backRight.getCurrentPosition() - moveCounts;
            newLeftRearTarget = backLeft.getCurrentPosition() + moveCounts;

            // Set Targets
            frontLeft.setTargetPosition(newLeftFrontTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backLeft.setTargetPosition(newLeftRearTarget);
            backRight.setTargetPosition(newRightRearTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

            while (((LinearOpMode) callingOpMode).opModeIsActive() && frontLeft.isBusy()) {
                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
            }
            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void tankDrive(double speed, double distance) {
        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double scale = 47.5 / 45.;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            if (distance < 0) {
                scale = 1.0;
            }
            moveCounts = (int) (distance * COUNTS_PER_INCH_435 * scale);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            newLeftFrontTarget = frontLeft.getCurrentPosition() + moveCounts;
            newRightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
            newRightRearTarget = backRight.getCurrentPosition() + moveCounts;
            newLeftRearTarget = backLeft.getCurrentPosition() + moveCounts;

            // Set Targets
            frontLeft.setTargetPosition(newLeftFrontTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backLeft.setTargetPosition(newLeftRearTarget);
            backRight.setTargetPosition(newRightRearTarget);

// Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

// ==================
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

//            while (((LinearOpMode) callingOpMode).opModeIsActive() && frontLeft.isBusy()) {
//            }
            stopMotors();

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void tankDriveCorrected(double speed, double distance, double orientation) {
        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double scale = 47.5 / 45.;
        double error;
        double correction;
        double P_COEFF = .03;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            if (distance < 0) {
                scale = 1.0;
            }
            moveCounts = -(int) (distance * COUNTS_PER_INCH_435 * scale);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            newLeftFrontTarget = frontLeft.getCurrentPosition() + moveCounts;
            newRightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
            newRightRearTarget = backRight.getCurrentPosition() + moveCounts;
            newLeftRearTarget = backLeft.getCurrentPosition() + moveCounts;

            // Set Targets
            frontLeft.setTargetPosition(newLeftFrontTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backLeft.setTargetPosition(newLeftRearTarget);
            backRight.setTargetPosition(newRightRearTarget);

// Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is are running.

// ==================

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            while (((LinearOpMode) callingOpMode).opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && backLeft.isBusy()) {
                error = orientation - getFieldHeading();

                while (error > 180) error = (error - 360);
                while (error <= -180) error = (error + 360);

                correction = Range.clip(error * P_COEFF, -1, 1);

                if (distance < 0) {
                    correction *= -1.;
                }

                frontLeft.setPower(speed - correction);
                frontRight.setPower(speed + correction);
                backLeft.setPower(speed - correction);
                backRight.setPower(speed + correction);

                //((LinearOpMode) callingOpMode).sleep(20);
            }
            // Stop all motion;

            stopMotors();

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void DriveSidewaysCorrected(double speed, double distance, double directionOfFront) {

        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int moveCounts;
        double correction;

        speed = Range.clip(speed, -1.0, 1.0);
        double error;

        if (((LinearOpMode) callingOpMode).opModeIsActive()) {    // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            moveCounts = -(int) (distance * COUNTS_PER_INCH_435);
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // Turn On RUN_TO_POSITION
            newLeftFrontTarget = frontLeft.getCurrentPosition() - moveCounts;
            newRightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
            newRightRearTarget = backRight.getCurrentPosition() - moveCounts;
            newLeftRearTarget = backLeft.getCurrentPosition() + moveCounts;

            // Set Targets
            frontLeft.setTargetPosition(newLeftFrontTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backLeft.setTargetPosition(newLeftRearTarget);
            backRight.setTargetPosition(newRightRearTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // keep looping while we are still active, and the left front motor is running.

            while (((LinearOpMode) callingOpMode).opModeIsActive() && frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                error = directionOfFront - getFieldHeading();

                while (error > 180) error = (error - 360);
                while (error <= -180) error = (error + 360);

                correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

                if (distance > 0) {
                    correction *= -1.;
                }

                frontLeft.setPower(speed + correction);
                frontRight.setPower(speed + correction);
                backLeft.setPower(speed - correction);
                backRight.setPower(speed - correction);

                //((LinearOpMode) callingOpMode).sleep(20);

            }
            // Stop all motion;
            stopMotors();

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public double leftDistanceToWall() {
        return revRangeLeft.getDistance(DistanceUnit.INCH);
    }

    public double frontDistanceToWall() {
        return revRangeFront.getDistance(DistanceUnit.INCH);
    }

    public double rearDistanceToWall() {
        return revRangeRear.getDistance(DistanceUnit.INCH);
    }

    public double rightDistanceToWall() {
        return revRangeRight.getDistance(DistanceUnit.INCH); // check for distanceOutOfRange
    }
//    public double frontDistance() {
//        double frontrange = 500;
//        double tempRangeL = frontLeftDistance();
//        double tempRangeR = rearDistance();
//
//        // try three times to get a correct front distance measurement... otherwise park
////        for (int i = 0; i <= 3; i++) {
////            if (tempRangeL > 15 && tempRangeR > 15) {
////                ((LinearOpMode) callingOpMode).sleep(50);//small wait and measure again
////                tempRangeL = frontLeftDistance();
////                tempRangeR = rearDistance();
////            }
////        }
//
//        if (tempRangeL <= tempRangeR && tempRangeL < 15) {
//            frontrange = tempRangeL;
//        } else if (tempRangeR < tempRangeL && tempRangeR < 15) {
//            frontrange = tempRangeR;
//        }
//
//        return frontrange;
//    }
    // ------------------------- Blue Backdrop Pixel Plow ---------------------------------
    public void plowFromBlueBackdropStartToLeftSpike() { // Left
        tankDrive(.5, 2);
        gyroTurn(.5, 25);
        tankDrive(.5, 17.45);
        tankDrive(.5, -10);
        gyroTurn(.5, 90);
    }

    public void plowFromBlueBackdropStartToCenterSpike() { // Center
        tankDrive(.5, 2);
        gyroTurn(.3, 10);
        tankDrive(.3, 10);
        gyroTurn(.3, 0);
        tankDrive(.5, 14);
        gyroTurn(.3, -5);
        tankDrive(.5, -12);
        gyroTurn(.5, 90);
    }

    public void plowFromBlueBackdropStartToRightSpike() { // Right
        tankDrive(.5, 2);
        gyroTurn(.3, 10);
        tankDrive(.5, 10);
        gyroTurn(.3, 0);
        tankDrive(.3, 3);
        gyroTurn(.3, -15);
        tankDrive(.3, 5);
        gyroTurn(.3, -45);
        tankDrive(.3, 5);

        tankDrive(.5, -10);
        gyroTurn(.5, 90);
    }

    // ---------------------------------- Red Wing Pixel Plow --------------------------------
    public void plowFromRedWingStartToCenterSpike() { // Center
        tankDrive(.5, 2);
        gyroTurn(.3, 10);
        tankDrive(.3, 25.5);
        gyroTurn(.3, -5);
        tankDrive(.5, -12);
        gyroTurn(.5, 90);
    }

    public void plowFromRedWingStartToLeftSpike() { // Left
        tankDrive(.5, 2);
        gyroTurn(.5, 25);
        tankDrive(.5, 19);
        tankDrive(.5, -10);
        gyroTurn(.5, 0);
    }

    public void plowFromRedWingStartToRightSpike() { // Right
        tankDrive(.5, 2);
        gyroTurn(.3, 10);
        tankDrive(.5, 10);
        gyroTurn(.3, 0);
        tankDrive(.3, 3);
        gyroTurn(.3, -15);
        tankDrive(.3, 5);
        gyroTurn(.3, -45);
        tankDrive(.3, 5);
        tankDrive(.5, -10);
        gyroTurn(.5, 90);
    }

    //------------------------------ Red Backdrop Pixel Plow ------------------------------------
    public void plowFromRedRightStartToRightSpike() { // Right
        gyroTurn(.4, -16); // -3 angle 2/17/24
        tankDrive(.5, 19.5); // +0.5 distance 2/17/24
        tankDrive(.3, -10);
        gyroTurn(.6, -90);
    }

    public void plowFromRedRightStartToCenterSpike() { // Center
        tankDrive(.5, 25);
        tankDrive(.3, -10);
        gyroTurn(.6, -90);
    }

    public void plowFromRedRightStartToLeftSpike() { // Left
        tankDrive(.5, 8);
        gyroTurn(.3, 20);
        tankDrive(.3, 8);
        gyroTurn(.3, 40);
        tankDrive(.3, 6.5); // +0.5 distance 2/17/24
        tankDrive(.5, -15);
        gyroTurn(.5, -90);
    }

    //------------------------- Blue Wing Pixel Plow -----------------------------
    public void plowFromBlueRightStartToRightSpike() { // Right
        tankDrive(.5, 1);
        gyroTurn(.4, -13);
        tankDrive(.5, 19);
        tankDrive(.3, -18); // AJB changed from -10 on 2/24
    }

    public void plowFromBlueRightStartToCenterSpike() { // Center
        tankDrive(.3, 26); // AJB changed from distance 25 on 2/24 and speed from .6 on 2/27
        tankDrive(.3, -10);
    }

    public void plowFromBlueRightStartToLeftSpike() { // Left
        tankDrive(.5, 8);
        gyroTurn(.3, 20);
        tankDrive(.3, 8);
        gyroTurn(.3, 40);
        tankDrive(.3, 6.5);
        tankDrive(.5, -10);
    }

    /**
     * Move a motor safely to the target
     * @param motor Motor to move
     * @param increment Target position
     * @param power Motor power
     * @param min Minimum value for the motor
     * @param max Maximum value for the motor
     * @return Actual target used
     */
    public int incrementMotorSafe(DcMotor motor, int increment, double power, int min, int max) {
        motor.setPower( power );
        int target = motor.getCurrentPosition() + increment;
        // Bound the target between the min and max values
        target = Math.max( Math.min(target,max), min );
        motor.setTargetPosition(target);
        return target;
    }
}
