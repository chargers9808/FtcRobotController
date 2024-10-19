package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.GameBase;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;

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
    private final long DELAY_SCORE = 350;

    /**
     * Postion when slide is extended
     */
    public int slideOut = -1580;
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
    public int armScoringPosition =  -1730;
    /**
     * Position for arm to clear the submersible bar (parallel to mat)
     */
    public int armCollectPositionUp = -460;
    /**
     * Position for arm when collecting
     */
    public int armCollectPositionDown = -250;

    /**
     * Counter used for pickup mode toggle
     */
    public int pickupPrimary = 0;

    /**
     * Flag for diagnostic mode
     */
    public boolean diagnosticMode = false;
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
        HeadingHolder.setHeading(0.0);
        driveBase.imu.resetYaw();
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

    /**
     * Score an item in the specified basket
     * @param pos Target basket
     */
    public void score(Basket pos) {
        driveBase.stopMotors();
        switch( pos ) {
            case TOP:
                //Arm
                driveBase.moveMotor(driveBase.arm, (armScoringPosition -75), 0.4, false);
                //Slide
                driveBase.moveMotor(driveBase.slide, slideOut, 0.6, false);
                break;
            case MID:
            case BOTTOM:
                // TODO: Implement preset for lower baskets
        }
        // Move the bot
        driveBase.gyroTurn(.5,180);
        driveBase.waitForMotor(driveBase.frontRight);
        driveBase.tankDrive(.5,driveBase.frontDistanceToWall()-5);// determine the correct distance for this
        driveBase.driveSideways(.5,driveBase.rightDistanceToWall()-9);// determine the correct distance for this
        driveBase.gyroTurn(.5,140);
        driveBase.waitForMotor(driveBase.frontRight);

        driveBase.moveMotor(driveBase.arm, (armScoringPosition), 0.1, false);
        driveBase.tankDrive(.2,2);

        //Outtake
        sweeperOut();
        sleep( DELAY_SCORE );
        sweeperOff();

        // Back up from baskets
        driveBase.tankDrive( 0.5, -2);
        driveBase.gyroTurn(.5,90);
        driveBase.tankDrive( 0.5, -5);

        // Retract the slide
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, false);
        // Retract the arm
        driveBase.moveMotor(driveBase.arm, armLowered-50, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armLowered, 0.2, false);
    }

    public void pickup() {
        if (pickupPrimary == 0) {
            driveBase.tankDrive(.5, driveBase.frontDistanceToWall() - 9);
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

        pickupPrimary++;
    }

    public void prepareToTravel() {
        pickupPrimary = 0;
        sweeperOff();
        driveBase.moveMotor(driveBase.arm, (armCollectPositionUp), 0.6, false);
        driveBase.moveMotor(driveBase.slide, slideIn, .8, true);
    }

    public void travel() {
        sweeperIn();
        sleep( 50 );
        sweeperOff();
        if (driveBase.arm.getCurrentPosition() > -20){
        driveBase.moveMotor(driveBase.arm, armLowered, 0.4, false);
        }
        driveBase.moveMotor(driveBase.slide, slideIn, 0.8, true);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.8, false);
        driveBase.slide.setPower(0);

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
            telemetry.addData("heading        :   ", driveBase.getFieldHeading());
            telemetry.update();
        }
        telemetry.addData("run-time           : ", (driveBase.runtime.seconds()));
    }
}
