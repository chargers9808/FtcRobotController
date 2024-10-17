package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public abstract class IntoTheDeepBase extends LinearOpMode9808 implements GameBase {

    public enum Basket {
        TOP,
        MID,
        BOTTOM
    }

    public enum Collect {
        Out,
        Middle,
        In,
    }

    /**
     * Sweeper powers
     */
    private final double SWEEPER_IN = -1;
    private final double SWEEPER_OUT = 1;
    private final double SWEEPER_OFF = 0;

    /**
     * Delays
     */
    private final long DELAY_SCORE = 350;

    public boolean diagnosticMode = false;
    public int PickupPrimary = 0;

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
                driveBase.moveMotor(driveBase.arm, (driveBase.armScoringPositon-75), 0.4, false);
                //Slide
                driveBase.moveMotor(driveBase.slide, driveBase.slideOut, 0.6, false);
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

        driveBase.moveMotor(driveBase.arm, (driveBase.armScoringPositon), 0.1, false);
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
        driveBase.moveMotor(driveBase.slide, driveBase.slideIn, 0.8, false);
        // Retract the arm
        driveBase.moveMotor(driveBase.arm, driveBase.armLowered-50, 0.8, false);
        driveBase.moveMotor(driveBase.arm, driveBase.armLowered, 0.2, false);
    }

    public void pickup() {
        if (PickupPrimary == 0) {
            driveBase.tankDrive(.5, driveBase.frontDistanceToWall() - 9);
            driveBase.moveMotor(driveBase.arm, (driveBase.armCollectPositonUp), 0.6, true);
        }
        if ((PickupPrimary%2) == 0) {
            driveBase.moveMotor(driveBase.slide, driveBase.slideCollectPosition, .8, true);
            driveBase.moveMotor(driveBase.arm, driveBase.armCollectPositonDown, .4, false);
            driveBase.setLED(DraculaBase.LEDColor.WHITE);
        }
        else {
            driveBase.moveMotor(driveBase.slide, driveBase.slideCollectPosition-300, .8, true);
            driveBase.moveMotor(driveBase.arm, driveBase.armCollectPositonDown-10, .4, false);
            driveBase.setLED(DraculaBase.LEDColor.YELLOW);
        }
        sweeperIn();

        PickupPrimary++;
    }

    public void prepareToTravel() {
        PickupPrimary = 0;
        sweeperOff();
        driveBase.moveMotor(driveBase.arm, (driveBase.armCollectPositonUp ), 0.6, false);
        driveBase.moveMotor(driveBase.slide, driveBase.slideIn, .8, true);
    }

    public void travel() {
        sweeperIn();
        sleep( 50 );
        sweeperOff();
        if (driveBase.arm.getCurrentPosition() > -20){
        driveBase.moveMotor(driveBase.arm, driveBase.armLowered, 0.4, false);
        }
        driveBase.moveMotor(driveBase.slide, driveBase.slideIn, 0.8, true);
        driveBase.moveMotor(driveBase.arm, driveBase.armTravelPosition, 0.8, false);
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
