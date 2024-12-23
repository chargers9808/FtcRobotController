package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.GameBase;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

public abstract class IntoTheDeepBase extends LinearOpMode9808 implements GameBase {
    /**
     * Target basket to score in
     */
    public enum Basket {
        TOP,
        MID,
        BOTTOM
    }

    /**
     * Power setting for sweeping in a sample
     */
    private final double SWEEPER_IN = -1;
    /**
     * Power setting for releasing a sample from sweeper
     */
    private final double SWEEPER_OUT = 1;
    /**
     * Power setting to stop sweeper
     */
    private final double SWEEPER_OFF = 0;

    /**
     * Delay between starting sweeper out and stopping sweeper
     */
    protected final long DELAY_SCORE = 500;

    /**
     * Postion when slide is extended
     */
    public int slideOut = -1575; //-1550;
    /**
     * Position when slide is retracted
     */
    public int slideIn = -20;
    /**
     * Position when slide is partially extended to sweep
     */
    public int slideCollectPosition = -1150;

    /**
     * Position when arm is lowered
     */
    public int armLowered = -250;
    /**
     * Position when arm is ready for travel
     */
    public int armTravelPosition = -1990;
    /**
     * Position when arm is ready to score
     */
    public int armScoringPosition =  -1730; //-1730 4:19 11/23/24
    /**
     * Position for arm to clear the submersible bar (parallel to mat)
     */
    public int armCollectPositionUp = -460;
    /**
     * Position for arm when collecting
     */
    public int armCollectPositionDown = -250;

    /**
     * Position for arm when collecting in Auto
     */
    public int armCollectPositionCollect = -100;
    public int armCollectPositionMat = -60;


    /**
     * Counter used for pickup mode toggle
     */
    public int pickupPrimary = 0;

    /**
     * Flag for diagnostic mode
     */
    public boolean diagnosticMode = false;


    public boolean beforeTravel = false;

    /**
     * Sweeper motor
     */
    protected CRServo intake;

    protected abstract void initialize();
    protected abstract void pre_initialize();

    @Override
    protected Alliance getAlliance() {
        return new Alliance(Alliance.Color.UNKNOWN);
    }

    public void hardwareSetup( DraculaBase driveBase) {
        driveBase.init(hardwareMap, this);
        intake = this.hardwareMap.crservo.get("intake");
        sweeperOff();
        HeadingHolder.setHeading(HeadingHolder.getHeading());
        driveBase.imu.resetYaw();

        driveBase.initOdometryComputer( -84.0, -168.0 );
    }

    /**
     * Code that will be run once prior to the initialization loop
     */
    protected void pre_init_9808() {
        hardwareSetup(driveBase);

        pre_initialize();

        // Set LED state
        setLEDHeartbeat();
    }

    /**
     * Code that will run repeatedly while in Init mode
     */
    protected void init_9808() {
        // Call init for the specific opmode
        initialize();

        telemetry.addLine("ready for START");
        telemetry.addLine( "Alliance: " + getAlliance().getColorString());
        telemetry.update();
    }

    public void sweeperIn(){
        intake.setPower( SWEEPER_IN );
    }

    public void sweeperOut(){
        intake.setPower( SWEEPER_OUT );
    }

    public void sweeperOff(){
        intake.setPower( SWEEPER_OFF );
    }

    public double getSensorOffset( DraculaBase.SensorDir sensor ) {
        switch ( sensor ) {
            case RIGHT:
            case LEFT:
                return 9.0;
            case FRONT:
                return 9.0;
            case REAR:
                return 9.0;
        }
        return 0.0;
    }

    protected void updateOdometryObservation() {
        double currX, currY;
        double currentHeading = driveBase.getFieldHeading();
        DraculaBase.SensorDir sensorX, sensorY;
        if( currentHeading <= 315 && currentHeading >= 225) {
            currentHeading = 270.0;
            sensorY = DraculaBase.SensorDir.FRONT;
            sensorX = DraculaBase.SensorDir.RIGHT;
        } else if ( currentHeading >= 135 && currentHeading <= 225 ) {
            currentHeading = 180.0;
            sensorY = DraculaBase.SensorDir.LEFT;
            sensorX = DraculaBase.SensorDir.FRONT;
        } else {
            currentHeading = 0.0;
            sensorY = DraculaBase.SensorDir.RIGHT;
            sensorX = DraculaBase.SensorDir.REAR;
        }
        driveBase.gyroTurn(.8, currentHeading);
        currY = 144 - ( driveBase.distanceToWall(sensorY) + getSensorOffset(sensorY) );
        currX = driveBase.distanceToWall(sensorX) + getSensorOffset(sensorX);
        Pose2DGobilda pos = new Pose2DGobilda(DistanceUnit.INCH, currX, currY, AngleUnit.DEGREES, currentHeading);
        driveBase.odometryComputer.setPosition( pos );
    }

    protected void updateOdometryNet() {
        double currX, currY;
        double currentHeading = driveBase.getFieldHeading();
        DraculaBase.SensorDir sensorX, sensorY;
        if( currentHeading <= 135 && currentHeading >= 45) {
            currentHeading = 90.0;
            sensorY = DraculaBase.SensorDir.FRONT;
            sensorX = DraculaBase.SensorDir.LEFT;
        } else if ( currentHeading >= 135 && currentHeading <= 225 ) {
            currentHeading = 180.0;
            sensorY = DraculaBase.SensorDir.RIGHT;
            sensorX = DraculaBase.SensorDir.FRONT;
        } else {
            currentHeading = 0.0;
            sensorY = DraculaBase.SensorDir.LEFT;
            sensorX = DraculaBase.SensorDir.REAR;
        }
        driveBase.gyroTurn(.8, currentHeading);
        currY = driveBase.distanceToWall(sensorY) + getSensorOffset(sensorY);
        currX = driveBase.distanceToWall(sensorX) + getSensorOffset(sensorX);
        Pose2DGobilda pos = new Pose2DGobilda(DistanceUnit.INCH, currX, currY, AngleUnit.DEGREES, currentHeading);
        driveBase.odometryComputer.setPosition( pos );
    }

    /**
     * Score an item in the specified basket
     * @param pos Target basket
     */
    public void score(Basket pos) {
        driveBase.stopMotors();
        switch( pos ) {
            case TOP:
                //Arm -80
                driveBase.moveMotor(driveBase.arm, (armScoringPosition -110), 0.4, false);
                //Slide
                driveBase.moveMotor(driveBase.slide, slideOut, 0.6, false);
                break;
            case MID:
            case BOTTOM:
                // TODO: Implement preset for lower baskets
        }

        // Determine if we should point towards the wall, or away
        double targetAngle = 0;
        double currentHeading = driveBase.getFieldHeading();
        if (currentHeading > 90 && currentHeading < 270 ) {
            targetAngle = 180;
        }
        if( currentHeading >= 70 && currentHeading <= 110) {
            targetAngle = 90;
        }
        telemetry.addData("heading", driveBase.getFieldHeading());
        telemetry.addData("target", targetAngle);
        telemetry.update();

        // Move the bot
        driveBase.gyroTurnWait(.5, targetAngle);
        if( targetAngle != 90 ) {
            driveBase.tankDriveUntil(.5, 6, targetAngle == 180, false);// determine the correct distance for this
            sleep(50);
            //dis 8; 4:32 11/13
            driveBase.driveSidewaysUntil(.5, 10, targetAngle == 180);// determine the correct distance for this
            sleep(50);
        } else {
            driveBase.tankDriveUntil( .5, 9.5, true, false);
            sleep(50);
            //dis 8; 4:34 11/13
            driveBase.driveSidewaysUntil(.5, 10, false);
            sleep(50);
        }
        driveBase.gyroTurnWait(.5,140);

        driveBase.tankDrive(.2,3);
        driveBase.moveMotor(driveBase.arm, (armScoringPosition), 0.1, false);

        //Outtake
        sweeperOut();
        sleep( DELAY_SCORE );
        sweeperOff();

        // Back up from baskets
        driveBase.tankDrive( 0.5, -5);
        driveBase.gyroTurn(.5,90);

        // Retract the slide
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
        // Retract the arm
        driveBase.moveMotor(driveBase.arm, armLowered-50, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armLowered, 0.2, false);
    }
    public void pickup() {
        if (pickupPrimary == 0) {
            driveBase.moveMotor(driveBase.arm, (armCollectPositionUp), 0.6, true);
        }
        if ((pickupPrimary %2) == 0) {
            driveBase.moveMotor(driveBase.slide, slideCollectPosition, .8, true);
            driveBase.moveMotor(driveBase.arm, armCollectPositionDown, .4, false);
            driveBase.setLED(DraculaBase.LEDColor.WHITE);
        }
        else {
            driveBase.moveMotor(driveBase.slide, slideCollectPosition-300, .8, true);
            driveBase.moveMotor(driveBase.arm, armCollectPositionDown -10, .4, false);
            driveBase.setLED(DraculaBase.LEDColor.YELLOW);
        }
        sweeperIn();

        beforeTravel = true;
        pickupPrimary++;
    }

    public void prepareToTravel() {
        pickupPrimary = 0;
        //sweeperOff();
        driveBase.moveMotor(driveBase.arm, (armCollectPositionUp), 0.6, false);
        driveBase.moveMotor(driveBase.slide, slideIn, .8, false);
        sleep(150);
    }

    public void travel() {
        if (beforeTravel == true) {
            prepareToTravel();
            driveBase.tankDrive(.5, -8);
        }
        sweeperIn();

        if (driveBase.arm.getCurrentPosition() > -20){
        driveBase.moveMotor(driveBase.arm, armLowered, 0.4, false);
        }
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.8, false);
        driveBase.slide.setPower(0);

        sweeperOff();

        beforeTravel = false;
    }

    public void autoSamples(Double sampleDistance, Double forwardDrive, Double sidewaysDrive) {
        driveBase.driveSidewaysUntil(.5,  11, false);
        
        driveBase.moveMotor(driveBase.arm, armCollectPositionUp, .4, false);
        sleep(50);
        driveBase.tankDrive(.5, driveBase.frontDistanceToWall() - (sampleDistance));
        sleep(50);
        driveBase.gyroTurnWait(.5,90);
        driveBase.driveSideways(.5, sidewaysDrive - driveBase.leftDistanceToWall());
        sleep(50);
        driveBase.gyroTurnWait(.5,90);


        sweeperIn();
        driveBase.moveMotor(driveBase.arm, armCollectPositionCollect, .5, false);
        while (driveBase.arm.isBusy());
        driveBase.tankDrive(.1, forwardDrive/2);
        driveBase.moveMotor(driveBase.arm, armCollectPositionMat, .1, false);
        driveBase.tankDrive(.1, forwardDrive/2);
        sleep(200);

        travel();
        driveBase.driveSidewaysUntil(.5,  10, false);
        driveBase.driveSidewaysUntil(.2,  8, false);
        driveBase.tankDriveUntil(.5, 8, true, false);

        driveBase.gyroTurn(.5, 0);
        score(Basket.TOP); // First Pickup
    }

    public void displayDiagnostics() {
        if (diagnosticMode) {
            telemetry.addLine();
            //Drive Motors
            telemetry.addData("Left Front     : ", driveBase.frontLeft.getCurrentPosition());
            telemetry.addData("Right Front    : ", driveBase.frontRight.getCurrentPosition());
            telemetry.addData("Left Rear      : ", driveBase.backLeft.getCurrentPosition());
            telemetry.addData("Right Rear     : ", driveBase.backRight.getCurrentPosition());
            telemetry.addLine();
            //Function Motors
            telemetry.addData("arm motor      : ", driveBase.arm.getCurrentPosition());
            telemetry.addData("slide motor    : ", driveBase.slide.getCurrentPosition());
            telemetry.addLine();
            //Function Servos

            //Distance Sensors
            telemetry.addData("right Distance : ", driveBase.rightDistanceToWall());
            telemetry.addData("left distance  : ", driveBase.leftDistanceToWall());
            telemetry.addData("front distance : ", driveBase.frontDistanceToWall());
            telemetry.addData("rear distance  : ", driveBase.rearDistanceToWall());
            telemetry.addLine();
            //Gyro heading
            if( driveBase.hasOdometry ) {
                Pose2DGobilda pos = driveBase.odometryComputer.getPosition();
                telemetry.addLine("X              : " + pos.getX(DistanceUnit.INCH));
                telemetry.addLine("Y              : " + pos.getY(DistanceUnit.INCH));
                telemetry.addLine("Heading        : " + pos.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addData("Heading        : ", driveBase.getFieldHeading());
            }
            telemetry.update();
        }
        telemetry.addData("run-time       : ", (driveBase.runtime.seconds()));
    }
}
