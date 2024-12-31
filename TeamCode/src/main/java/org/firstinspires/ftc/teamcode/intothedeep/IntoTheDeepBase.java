package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.GameBase;
import org.firstinspires.ftc.teamcode.DataHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;
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
     * Grip positions
     */
    public enum Grip_Position {
        GRIP_0DEG,
        GRIP_45DEG,
        GRIP_90DEG,
        GRIP_135DEG,
        GRIP_NONE
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

    protected final double GRIPPER_OPEN = .85;
    protected final double GRIPPER_CLOSED = .55;

    protected final double gripRotateIncrement = .01;
    protected static final double[] GRIP_ROTATIONS = {
      0.05, // 0 Degrees
      0.25, // 45 Degrees
      0.38, // 90 Degrees
      0.87  // 135 Degrees
    };

    protected boolean gripperOpen = false;

    /**
     * Postion when slide is extended
     */
    public int slideOut = -1575;
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
    public int armLowered = -75 ; //-175
    /**
     * Position when arm is ready for travel
     */
    public int armTravelPosition = -1990;
    /**
     * Position when arm is ready to score
     */
    public int armScoringPosition = -1670;// -1700; //-1730; //-1730 4:19 11/23/24
    /**
     * Position for arm to clear the submersible bar (parallel to mat)
     */
    public int armCollectPositionUp = -460;
    /**
     * Position for arm when collecting
     */
    public int armCollectPositionDown = -275; //-285

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
        sweeperOff();

        DataHolder.setHeading(DataHolder.getHeading());
        driveBase.imu.resetYaw();

        driveBase.initOdometryComputer( -146.0, -44.5, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD );
        driveBase.odometryComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
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

    protected void setGripRotation( Grip_Position pos ) {
        switch( pos ) {
            case GRIP_0DEG:
                setGripRotation( 0 );
                break;
            case GRIP_45DEG:
                setGripRotation( 1 );
                break;
            case GRIP_90DEG:
                setGripRotation( 2 );
                break;
            case GRIP_135DEG:
                setGripRotation( 3 );
                break;
        }
    }

    protected void setGripRotation( int pos ) {
        driveBase.gripRotation.setPosition(GRIP_ROTATIONS[pos]);
    }

    protected void openGripper() {
        gripperOpen = true;
        driveBase.grip.setPosition( GRIPPER_OPEN );
    }

    protected void closeGripper() {
        gripperOpen = false;
        driveBase.grip.setPosition( GRIPPER_CLOSED );
    }

    protected void toggleGripper() {
        if( gripperOpen ) {
            closeGripper();
        } else {
            openGripper();
        }
    }

    public void sweeperIn(){
        //intake.setPower( SWEEPER_IN );
    }

    public void sweeperOut(){
        //intake.setPower( SWEEPER_OUT );
    }

    public void sweeperOff(){
        //intake.setPower( SWEEPER_OFF );
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
        telemetry.addLine("New X          : " + pos.getX(DistanceUnit.INCH));
        telemetry.addLine("New Y          : " + pos.getY(DistanceUnit.INCH));
        telemetry.addLine("New Heading    : " + pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        driveBase.odometryComputer.setPosition( pos );
        driveBase.odometryComputer.bulkUpdate();
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

    protected void scoreSample() {
        //Outtake
        openGripper();
        sleep(200);
        setGripRotation(Grip_Position.GRIP_0DEG);

        driveBase.moveMotor(driveBase.arm, armTravelPosition, .1, false);
    }

    protected void score(Basket pos) {
        driveBase.setLED( DraculaBase.LEDColor.ORANGE );
        driveBase.stopMotors();
        setGripRotation(Grip_Position.GRIP_90DEG);
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
            driveBase.tankDriveUntil(.5, 12, targetAngle == 180, false);// 6
            sleep(50);
            //dis 8; 4:32 11/13
            driveBase.driveSidewaysUntil(.5, 13, targetAngle == 180); //10
            sleep(50);
        } else {
            driveBase.tankDriveUntil( .5, 10, true, false); //9.5
            sleep(50);
            //dis 8; 4:34 11/13
            driveBase.driveSidewaysUntil(.5, 15, false); //10
            sleep(50);
        }
        driveBase.gyroTurnWait(.5,140);

        driveBase.tankDrive(.2,7); //6
        driveBase.moveMotor(driveBase.arm, (armScoringPosition), 0.1, false);

        scoreSample();

        driveBase.moveMotor(driveBase.arm, armTravelPosition, .1, false);

        // Back up from baskets
        driveBase.tankDrive( 0.5, -7);
        driveBase.setLED( DraculaBase.LEDColor.GREEN );
        driveBase.gyroTurn(.5,90);

        // Retract the slide
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
        // Retract the arm
        driveBase.moveMotor(driveBase.arm, armLowered-50, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armLowered, 0.2, false);

    }

    /**
     * Score an item in the specified basket
     * @param pos Target basket
     */
    public void scoreSweeper(Basket pos) {
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
            driveBase.setLED(DraculaBase.LEDColor.WHITE);
        }
        else {
            driveBase.moveMotor(driveBase.slide, slideCollectPosition-300, .8, true);
            driveBase.setLED(DraculaBase.LEDColor.YELLOW);
        }
        openGripper();

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
        if (beforeTravel) {
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
            //Gyro heading
            if( driveBase.hasOdometry ) {
                driveBase.odometryComputer.bulkUpdate();
                Pose2DGobilda pos = driveBase.odometryComputer.getPosition();
                telemetry.addLine("X              : " + pos.getX(DistanceUnit.INCH));
                telemetry.addLine("Y              : " + pos.getY(DistanceUnit.INCH));
                telemetry.addLine("PP Heading     : " + pos.getHeading(AngleUnit.DEGREES));
            }
            telemetry.addData("Gyro Heading   : ", driveBase.getFieldHeading());
            telemetry.addLine();

            //Drive Motors
            if( false ) {
                telemetry.addData("Left Front     : ", driveBase.frontLeft.getCurrentPosition());
                telemetry.addData("Right Front    : ", driveBase.frontRight.getCurrentPosition());
                telemetry.addData("Left Rear      : ", driveBase.backLeft.getCurrentPosition());
                telemetry.addData("Right Rear     : ", driveBase.backRight.getCurrentPosition());
                telemetry.addLine();
            }
            //Function Motors
            telemetry.addData("arm motor      : ", driveBase.arm.getCurrentPosition());
            telemetry.addData("slide motor    : ", driveBase.slide.getCurrentPosition());
            telemetry.addData("Grip Rotation Position :", driveBase.gripRotation.getPosition());
            telemetry.addData( "Gripper Position", driveBase.grip.getPosition());
            telemetry.addData( "Gripper open?: ", gripperOpen);
            telemetry.addLine();
            //Function Servos

            //Distance Sensors
            telemetry.addData("right Distance : ", driveBase.rightDistanceToWall());
            telemetry.addData("left distance  : ", driveBase.leftDistanceToWall());
            telemetry.addData("front distance : ", driveBase.frontDistanceToWall());
            telemetry.addData("rear distance  : ", driveBase.rearDistanceToWall());
            telemetry.addLine();
            opModeDiagnostics();
        }
        telemetry.addData("run-time       : ", (driveBase.runtime.seconds()));
        telemetry.update();
    }
    protected void opModeDiagnostics() {

    }
}
