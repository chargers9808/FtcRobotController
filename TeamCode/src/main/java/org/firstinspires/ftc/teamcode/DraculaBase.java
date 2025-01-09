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
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

public class DraculaBase {
    public enum LEDColor {
        OFF,
        RED,    // DOES NOT WORK
        ORANGE, // WORKS
        YELLOW, // WORKS
        SAGE,   // WORKS
        GREEN,  // WORKS
        AZURE,  // WORKS
        BLUE,   // WORKS
        INDIGO, // DOES NOT WORK
        VIOLET, // WORKS
        WHITE   // WORKS
    }

    public enum SensorDir {
        FRONT,
        RIGHT,
        LEFT,
        REAR
    }
    //region hardware devices
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, slide;
    public Servo grip, gripRotation, led;
    public DistanceSensor revRangeLeft, revRangeRight, revRangeFront, revRangeRear;
    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public IMU imu;
    public GoBildaPinpointDriver odometryComputer;
    //endregion

    // --------------  drive system and controls
    static final double COUNTS_PER_REV_gobilda435 = 384.5;    // Gobilda 435 rpm motors
    //static final double WHEEL_CIRCUMFERENCE = (96. / 25.4) * Math.PI;//  circumference in inches
    // the new wheels have a diameter of 104mm

    static final double WHEEL_CIRCUMFERENCE = (104. / 25.4) * Math.PI;//  circumference in inches

    static final double COUNTS_PER_INCH_435 = COUNTS_PER_REV_gobilda435 / WHEEL_CIRCUMFERENCE;// counts/inch travelled

    static final double DEFAULT_ODO_DIST_THRESHOLD = 1.0;
    static final double DEFAULT_ODO_HEADING_THRESHOLD = 1.0;

    // Set to indicate which LED driver to use
    public static final boolean useBlinkinDriver = false;

    double y = 0.0;
    double x = 0.0;
    double r = 0.0;
    double max = 1.0;

    // --------------  IMU related... orientation
    public double HEADING_THRESHOLD = 1.3;//set at 1.2 normally

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
        grip = getHardwareMap().servo.get("clawServo");
        gripRotation = getHardwareMap().servo.get("rotationServo");
        led = getHardwareMap().servo.get("led");
    }
    private Servo getCrServo(String deviceName) {
        return getHardwareMap().servo.get(deviceName);
    }

    private DcMotor getDcMotor(String deviceName) {
        return getHardwareMap().dcMotor.get(deviceName);
    }

    public void initOdometryComputer(double xOffset, double yOffset, GoBildaPinpointDriver.GoBildaOdometryPods podType){
        odometryComputer = getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        odometryComputer.setOffsets(xOffset, yOffset);
        odometryComputer.setEncoderResolution(podType);
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryComputer.resetPosAndIMU();
        DataHolder.setOdometry(odometryComputer);
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
        double lfrontpower = +y + x - r;
        double rfrontpower = y - x + r;
        double lrearpower = y - x - r;
        double rrearpower = +y + x + r;

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

    public double calculateTurn( double speed, double targetAngle) {
        return calculateTurn( speed, targetAngle, getFieldHeading());
    }

    public double calculateTurn( double speed, double targetAngle, double currentAngle) {
        double delta = targetAngle - currentAngle;// how far we need to turn

        if (delta > 180.) {
            delta -= 360.;
        } else if (delta <= -180.) {
            delta += 360.;
        }
        // ==================

        if (delta < 0) {
            r = -speed;
        } else {
            r = +speed;
        }         // rotate CW or CCW depending on how the motors are set up

        if (Math.abs(delta) < 40.) {
            r = r * Math.abs(delta) / 40.;
        }  // scale down the rotation speed proportionate to error

        if (Math.abs(r) <= .1) {
            r = .1 * Math.abs(r) / r;
        }           // set a minimum r (turning speed), preserving the sign of r
        return r;
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
            applyMecPower2(x,y,r);    // set the motor speed using r
        }
        r = 0;
        stopMotors();  // the turn is complete within the HEADING_THRESHOLD
        // HeadingHolder.setHeading(robotFieldHeading());
    }

    public void gyroTurnWait(double speed, double targetAngle) {
        gyroTurn(speed, targetAngle);
        waitForMotor(frontRight);
        waitForMotor(frontLeft);
        waitForMotor(backLeft);
        waitForMotor(backRight);
    }

//------------------------------------------------------------------------------------------

    public void stopMotors() {
        x = 0.;
        y = 0.;
        r = 0.;

        setWheelMotorPower(0.0, 0.0, 0.0, 0.0);
    }

    private void setWheelMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontRightPower);
        frontRight.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
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

    /**
     * Sets the active LED color on the goBILDA LED
     *
     * @param color the LED color to set
     */
    public void setLED(LEDColor color) {
        double value = 0.0;
        switch( color ) {
            case OFF:
                value = 0.0;
                break;
            case RED:
                value = 0.277;
                break;
            case ORANGE:
                value = 0.333;
                break;
            case YELLOW:
                value = 0.388;
                break;
            case SAGE:
                value = 0.444;
                break;
            case GREEN:
                value = 0.500;
                break;
            case AZURE:
                value = 0.555;
                break;
            case BLUE:
                value = 0.611;
                break;
            case VIOLET:
                value = 0.722;
                break;
            case WHITE:
                value = 1.0;
        }
        led.setPosition( value );
    }

    public double getFieldHeading() {
        double theta;// this method returns the field heading of the robot

        //  this gets all the imu parameters... the "heading" is the "firstAngle + initialFieldHeading"
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        theta = orientation.getYaw(AngleUnit.DEGREES) + DataHolder.getHeading();  // initialized with the saved heading
//        theta = orientation.getYaw(AngleUnit.DEGREES);  // initialized with the saved heading

        if (theta < 0) {
            theta = theta + 360.;
        }
        if (theta > 360) {
            theta = theta - 360.;
        } // correct for the "wrap around" of this angle
        // this makes the angle 0-360 CCW same as the field angle, instead of 0-180 and then -180 to 0  (going CCW)
        return theta;
    }

    public void driveSideways(double speed, double distance) {
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

            while (((LinearOpMode) callingOpMode).opModeIsActive() && frontLeft.isBusy()) {
            }
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

    public void driveSidewaysCorrected(double speed, double distance, double directionOfFront) {

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

    public double distanceToWall( SensorDir sensor ) {
        switch( sensor ) {
            case FRONT:
                return frontDistanceToWall();
            case REAR:
                return rearDistanceToWall();
            case LEFT:
                return leftDistanceToWall();
            case RIGHT:
                return rightDistanceToWall();
        }
        return 0.0;
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

    /**
     * Move a motor and wait for it to finish
     *
     * @param motor Motor to move
     * @param target Target position
     * @param power Motor power
     */
    public void moveMotor(DcMotor motor, int target, double power, boolean wait) {
        motor.setPower(power);
        motor.setTargetPosition(target);
        if (wait) {
            waitForMotor(motor);
        }
    }

    /**
     * Wait for the current motor to finish moving
     * @param motor Motor to wait for
     */
    public void waitForMotor(DcMotor motor) {
        final int MOTOR_WAIT_THRESHOLD = 2;
        //while( Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > MOTOR_WAIT_THRESHOLD);
        while (motor.isBusy());
    }

    /**
     * Drive Sideways until distance sensor reads distance
     * @param speed speed of motor
     * @param distance target distance
     * @param rightSensor Are we using the Right Sensor
     *
     */
    public void driveSidewaysUntil(double speed, double distance, boolean rightSensor ){
        DistanceSensor sensor;
        double mult = 1;
        if ( rightSensor ) {
            sensor = revRangeRight;
        } else {
            sensor = revRangeLeft;
            mult *= -1;
        }
        double currentDistance = sensor.getDistance(DistanceUnit.INCH);
        while (currentDistance == DistanceSensor.distanceOutOfRange) {
            driveSidewaysCorrected(speed, 6 * mult, getFieldHeading());
            currentDistance = sensor.getDistance(DistanceUnit.INCH);
        }
        driveSideways(speed, (sensor.getDistance(DistanceUnit.INCH)-distance)*mult);
    }

    /**
     * Drive Forward until distance sensor reads distance
     * @param speed speed of motor
     * @param distance target distance
     * @param frontSensor Are we using the Front Sensor?
     * @param loopOnInvalid Continue to run to the distance if rae is out of range
     */
    public void tankDriveUntil(double speed, double distance, boolean frontSensor, boolean loopOnInvalid) {
        DistanceSensor sensor;
        double mult = 1;
        if (frontSensor) {
            sensor = revRangeFront;
        } else {
            sensor = revRangeRear;
            mult *= -1;
        }
        double currentDistance = sensor.getDistance(DistanceUnit.INCH);
        if( loopOnInvalid ) {
            while (currentDistance > (.8*DistanceSensor.distanceOutOfRange)) {
                tankDrive(speed, 6 * mult);
                currentDistance = sensor.getDistance(DistanceUnit.INCH);
            }
        }
        if( currentDistance < (.8*DistanceSensor.distanceOutOfRange) ) {
            tankDrive(speed, (sensor.getDistance(DistanceUnit.INCH) - distance) * mult);
        }
    }

    public void driveTo( double speed, Pose2DGobilda pos) {
        driveTo( speed, pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
    }

    public void driveTo( double speed, double targetX, double targetY, double targetHeading ) {
        driveTo( speed, targetX, targetY, targetHeading, DEFAULT_ODO_DIST_THRESHOLD, DEFAULT_ODO_HEADING_THRESHOLD);
    }

    public void driveTo( double speed, double targetX, double targetY, double targetHeading, double distThreshold, double headingThreshold) {
        final double dist_slow_thresh = 0.054;
        final double min_speed = 0.1;

        double distanceToTarget, directionToTarget, headingOffset;

        double vel, velX, velY;

        double vStrafeRobot, vForwardRobot, rotation;

        double currentX, currentY, currentHeadingUnnormlized;

        Pose2DGobilda pos;

        double startTime = runtime.time();

        do {
            odometryComputer.bulkUpdate();
            pos = odometryComputer.getPosition();

            currentX=pos.getX(DistanceUnit.INCH);
            currentY=pos.getY(DistanceUnit.INCH);

            // Calculate the offset from the current heading to the target heading
            currentHeadingUnnormlized = pos.getHeading(AngleUnit.DEGREES);
            headingOffset = targetHeading - currentHeadingUnnormlized;

            // Calculate current direct distance to the target location
            distanceToTarget=Math.hypot(Math.abs(targetX - currentX),
                    Math.abs(targetY - currentY));

            // Calculate radian angle to target location
            directionToTarget=Math.atan2(targetY-currentY,targetX-currentX);

            // Set velocity to the requested speed, until close to the target,
            // then slow the bot, but never slower than 0.1 speed
            vel=Math.max( Math.min(speed,distanceToTarget*dist_slow_thresh), min_speed );

            velX= vel*Math.cos(directionToTarget);
            velY= vel*Math.sin(directionToTarget);

            // now let's do the coordinate transformation to the robot
            // coordinate system.. which is rotated relative to the field by the
            // current heading
            vStrafeRobot=velX*Math.sin(pos.getHeading(AngleUnit.RADIANS))-velY*Math.cos(pos.getHeading(AngleUnit.RADIANS));
            vForwardRobot=velX*Math.cos(pos.getHeading(AngleUnit.RADIANS))+velY*Math.sin(pos.getHeading(AngleUnit.RADIANS));

            rotation = 0.0;
            if( Math.abs(headingOffset) >= headingThreshold ) {
                rotation = calculateTurn(speed, targetHeading, currentHeadingUnnormlized);;
            }

            callingOpMode.telemetry.addLine("Target: " + targetX + " " + targetY);
            callingOpMode.telemetry.addLine("X              : " + pos.getX(DistanceUnit.INCH));
            callingOpMode.telemetry.addLine("Y              : " + pos.getY(DistanceUnit.INCH));
            callingOpMode.telemetry.addLine("Heading        : " + pos.getHeading(AngleUnit.DEGREES));
            callingOpMode.telemetry.addLine("Dir to Target  : " + Math.toDegrees(directionToTarget));
            callingOpMode.telemetry.addLine("dist: " + Math.abs(distanceToTarget));
            callingOpMode.telemetry.addLine("head: " + Math.abs(headingOffset));
            callingOpMode.telemetry.update();

            applyMecPower2(vStrafeRobot, vForwardRobot, rotation);
        } while(((Math.abs(distanceToTarget) > distThreshold) /*|| (Math.abs(headingOffset) > headingThreshold)*/) && ((LinearOpMode) callingOpMode).opModeIsActive() );

        applyMecPower2(0,0,0);
        gyroTurn(speed, targetHeading);
    }
}
