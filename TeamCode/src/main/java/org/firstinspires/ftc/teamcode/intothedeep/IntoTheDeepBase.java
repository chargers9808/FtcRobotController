package org.firstinspires.ftc.teamcode.intothedeep;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.FieldTracker;
import org.firstinspires.ftc.teamcode.GameBase;
import org.firstinspires.ftc.teamcode.DataHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;
import org.firstinspires.ftc.teamcode.intothedeep.auto.Position;

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

    protected Pose2DGobilda POS_NET_SCORE = new Pose2DGobilda(
            DistanceUnit.INCH, 14.5, 15.5,
            AngleUnit.DEGREES, 135); //135
    protected Pose2DGobilda POS_NET_SCORE_EARLY = new Pose2DGobilda(
            DistanceUnit.INCH, 18, 18,
            AngleUnit.DEGREES, 135); //135
    protected Pose2DGobilda POS_NET_SCORE_END = new Pose2DGobilda(
            DistanceUnit.INCH, 22, 22,
            AngleUnit.DEGREES, 90);
    protected Pose2DGobilda POS_OBS_PICKUP = new Pose2DGobilda(
            DistanceUnit.INCH, 11, 106, //x10 y106
            AngleUnit.DEGREES, 270);

    protected Pose2DGobilda POS_NET_START = new Pose2DGobilda(DistanceUnit.INCH, 9, 44, AngleUnit.DEGREES, 0);
    protected Pose2DGobilda POS_OBS_START = new Pose2DGobilda(DistanceUnit.INCH, 9, 92, AngleUnit.DEGREES, 0);

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

    protected final double GRIPPER_OPEN = .75; //.85
    protected final double GRIPPER_CLOSED = .50; //.50

    protected static final double[] GRIP_ROTATIONS = {
            0.55,   // 0 Degrees
            0.72,   // 45 Degrees
            0.89,   // 90 Degrees
            0.40   // 135 Degrees
    };

    protected boolean gripperOpen = true;

    /**
     * Position when the lift is retracted
     */
    protected int liftIn = 600;

    /**
     * Position when the lift is fully extended
     */
    protected int liftOut = 3200;

    /**
     * Postion when slide is extended
     */
    public int slideOut = -1550; //-1575
    /**
     * Position when slide is retracted
     */
    public int slideIn = -220;
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
    public int armTravelPosition = -1890; //1990
    /**
     * Position when arm is ready to score
     */
    public int armScoringPosition = -1670;// -1700; //-1730; //-1730 4:19 11/23/24
    /**
     * Position for arm to clear the submersible bar (parallel to mat)
     */
    public int armCollectPositionUp = -360; //-440
    public int armPickupPosition = -330; //-440
    /**
     * Position for arm when collecting
     */
    public int armCollectPositionDown = -50;
    public int armCollectPositionDownBoth = -260; //-240


    protected int armCollectPositionWallDown = -568; //-620 Lower
    protected int armCollectPositionWallUp = armCollectPositionWallDown + 75;


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
     * Counter used for score mode toggle
     */

    public boolean scoreStart = true;

    /**
     * Flag for diagnostic mode
     */
    public boolean diagnosticMode = false;

    public boolean beforeTravel = false;

    protected abstract void initialize();
    protected abstract void pre_initialize();

    @Override
    protected Alliance getAlliance() {
        return new Alliance(Alliance.Color.UNKNOWN);
    }

    public void hardwareSetup( DraculaBase driveBase) {
        driveBase.init(hardwareMap, this);
        driveBase.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DataHolder.setHeading(DataHolder.getHeading());
        driveBase.imu.resetYaw();
        if( DataHolder.getOdometryEnabled() ) {
            driveBase.odometryComputer = DataHolder.getOdometryComputer();
        }
    }
    protected void initOdometry() {
        driveBase.initOdometryComputer( -44.5, -146.0, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD );
        driveBase.odometryComputer.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        FieldTracker.initialize(driveBase,
                9,
                9,
                9,
                true
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

    protected void setGripAngle( double angle ) {
        driveBase.gripRotation.setPosition( angle );
    }

    protected void openGripper() {
        gripperOpen = true;
        driveBase.grip.setPosition( GRIPPER_OPEN );
    }

    protected void closeGripper() {
        gripperOpen = false;
        driveBase.grip.setPosition( GRIPPER_CLOSED );
        sleep(150);
    }

    protected void toggleGripper() {
        if( gripperOpen ) {
            closeGripper();
        } else {
            openGripper();
        }
    }

    protected void scoreSample() {
        //Outtake
        openGripper();
        sleep(250); //200
        setGripRotation(Grip_Position.GRIP_0DEG);
        sleep(50); //150

        driveBase.moveMotor(driveBase.arm, armTravelPosition, .1, false);
//      driveBase.tankDrive(.3, -2);
        driveBase.driveTo( .3, POS_NET_SCORE);
    }

    protected void scoreOdo(Basket pos) {
        driveBase.setLED(DraculaBase.LEDColor.ORANGE);
        if (scoreStart) {
            driveBase.stopMotors();
            setGripRotation(Grip_Position.GRIP_90DEG);
            driveBase.moveMotor(driveBase.arm, (armScoringPosition - 110), 0.1, false);
            switch (pos) {
                case TOP:
                    //Arm -80
                    driveBase.moveMotor(driveBase.arm, (armScoringPosition), 0.4, false);
                    //Slide
                    driveBase.moveMotor(driveBase.slide, slideOut, 0.6, false);
                    break;
                case MID:
                case BOTTOM:
                    // TODO: Implement preset for lower baskets
            }
            driveBase.driveTo(0.5, POS_NET_SCORE_EARLY);
            sleep(50);
            driveBase.driveTo(0.5, POS_NET_SCORE, 2, 3);
            driveBase.moveMotor(driveBase.slide, slideOut, 0.6, false);

            driveBase.setLED(DraculaBase.LEDColor.GREEN);
            scoreStart = false;
        } else {
            scoreSample();

            driveBase.moveMotor(driveBase.arm, armTravelPosition, .1, false);

            // Back up from baskets
            driveBase.driveTo(0.5, POS_NET_SCORE_END);
            driveBase.setLED(DraculaBase.LEDColor.GREEN);

            // Retract the slide
            driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
            // Retract the arm
            driveBase.moveMotor(driveBase.arm, armLowered - 50, 0.8, false);
            driveBase.moveMotor(driveBase.arm, armLowered, 0.2, false);

            scoreStart = true;
        }
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

    protected void hangSpecimen() {
        // From travel
        setGripRotation(Grip_Position.GRIP_0DEG);
        driveBase.moveMotor( driveBase.slide, -175, 0.5, false);
        sleep(100);
        driveBase.moveMotor( driveBase.arm, -1380, 0.5, true);
        driveBase.moveMotor( driveBase.slide, slideIn, 0.5, false);

        driveBase.moveMotor( driveBase.arm, -900, 0.4, false );
        driveBase.tankDrive(.1, -1);
        sleep(500); //400
        openGripper();

    }

    protected void pickupSpecimenFromFloor() {
        setGripRotation(Grip_Position.GRIP_0DEG);
        openGripper();
        driveBase.moveMotor(driveBase.arm, armCollectPositionDown-55, .4, true);
        sleep(200); //300
        closeGripper();
    }

    protected void pickupSpecimenFromFloorTeleop() {
        driveBase.moveMotor(driveBase.slide, -1070, 0.5, false);
        driveBase.gyroTurn(0.5, 270);
         //-1455
        setGripRotation(Grip_Position.GRIP_0DEG);
        openGripper();
        driveBase.driveTo( 0.5, POS_OBS_PICKUP);
        pickupSpecimenFromFloor();
    }

    protected void pickupSpecimenFromWall() {
        driveBase.moveMotor( driveBase.slide, slideIn, 0.5, false);
        driveBase.moveMotor( driveBase.arm, armCollectPositionWallUp, 0.5, false );
        driveBase.moveMotor( driveBase.arm, armCollectPositionWallDown, 0.5, true );
        sleep(250);
    }

    public void pickup() {
        driveBase.stopMotors();
        if (pickupPrimary == 0) {
            driveBase.moveMotor(driveBase.arm, (armPickupPosition), 0.6, true);
        }
        if ((pickupPrimary %2) == 0) {
            driveBase.moveMotor(driveBase.slide, slideCollectPosition, .8, true);
        }
        else {
            driveBase.moveMotor(driveBase.slide, slideOut, .8, true);
        }
        openGripper();

        beforeTravel = true;
        pickupPrimary++;
    }

    public void prepareToTravel() {
        pickupPrimary = 0;
        driveBase.moveMotor(driveBase.arm, (armCollectPositionUp), 0.6, false);
        driveBase.moveMotor(driveBase.slide, slideIn, .8, false);
        sleep(150);
    }

    public void travel() {
        if (beforeTravel) {
            prepareToTravel();
            driveBase.tankDrive(.5, -8);
        }

        if (driveBase.arm.getCurrentPosition() > -20){
            driveBase.moveMotor(driveBase.arm, armLowered, 0.4, false);
        }
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.8, false);
        driveBase.slide.setPower(0);

        beforeTravel = false;
    }

    public void displayDiagnostics() {
        if (diagnosticMode) {
            telemetry.addLine();
            //Gyro heading
            if( DataHolder.getOdometryEnabled() ) {
                driveBase.odometryComputer.bulkUpdate();
                Pose2DGobilda pos = driveBase.odometryComputer.getPosition();
                telemetry.addData("X", pos.getX(DistanceUnit.INCH));
                telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
                telemetry.addData("PP Heading", pos.getHeading(AngleUnit.DEGREES));
                Pose2DGobilda fieldPos = FieldTracker.botToField(pos);
                telemetry.addData("X (Field)", fieldPos.getX(DistanceUnit.INCH));
                telemetry.addData("Y (Field", fieldPos.getY(DistanceUnit.INCH));
                telemetry.addData("PP Heading", fieldPos.getHeading(AngleUnit.DEGREES));
            }
            telemetry.addData("Gyro Heading   : ", driveBase.getFieldHeading());
            telemetry.addLine();

            //Function Motors
            telemetry.addData("arm motor      : ", driveBase.arm.getCurrentPosition());
            telemetry.addData("slide motor    : ", driveBase.slide.getCurrentPosition());
            telemetry.addData("lift motor    : ", driveBase.lift.getCurrentPosition());
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
    protected void opModeDiagnostics() {    }

    protected void outputPosition() {
        driveBase.odometryComputer.bulkUpdate();
        Pose2DGobilda pos = FieldTracker.getPosition();
        telemetry.addData("X", pos.getX(DistanceUnit.INCH));
        telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
        telemetry.addLine("----------------------------");
        telemetry.update();
        sleep(10000);
    }
}
