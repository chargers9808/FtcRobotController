package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.auto.Position;

public abstract class IntoTheDeepBase extends LinearOpMode9808 implements GameBase {

    public enum Basket {
        TOP,
        MID,
        BOTTOM
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
    private final long DELAY_SCORE = 5000;
    public boolean diagnosticMode = false;
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
        switch( pos ) {
            case TOP:
                //Arm
                driveBase.armNewTargetPosition = driveBase.armScoringPositon;
                driveBase.arm.setPower(.4);
                driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
                //Slide
                driveBase.slideNewTargetPosition = driveBase.slideOut;
                driveBase.slide.setPower(.6);
                driveBase.slide.setTargetPosition(driveBase.slideNewTargetPosition);
                break;
            case MID:
            case BOTTOM:
                // TODO: Implement preset for lower baskets
        }
        //Outtake
        sweeperOut();
        sleep( DELAY_SCORE );
        sweeperOff();
    }

    public void travel() {
        //Sweeper
        sweeperOff();
        //Arm
        driveBase.armNewTargetPosition = driveBase.armTravelPosition;
        driveBase.arm.setPower(.8);
        driveBase.arm.setTargetPosition(driveBase.armNewTargetPosition);
        //Slide
        driveBase.slideNewTargetPosition = driveBase.slideIn;
        driveBase.slide.setPower(.8);
        driveBase.slide.setTargetPosition(driveBase.slideNewTargetPosition);
    }

    public void displayDiagnostics() {
        if (diagnosticMode) {
//            telemetry.addData("Left Front     : ", driveBase.frontLeft.getCurrentPosition());
//            telemetry.addData("Right Front    : ", driveBase.frontRight.getCurrentPosition());
//            telemetry.addData("Left Rear      : ", driveBase.backLeft.getCurrentPosition());
//            telemetry.addData("Right Rear     : ", driveBase.backRight.getCurrentPosition());
            telemetry.addData("arm motor      : ", driveBase.arm.getCurrentPosition());
//            telemetry.addData("lift motor     : ", driveBase.lift.getCurrentPosition());
//            telemetry.addData("tiltServo      : ", driveBase.tiltPosition);
//            telemetry.addData("gripServo      : ", driveBase.gripPosition);
//            telemetry.addData("right Distance : ", driveBase.rightDistanceToWall());
//            telemetry.addData("left distance  : ", driveBase.leftDistanceToWall());
//            telemetry.addData("left front distance : ", driveBase.frontLeftDistance());
//            telemetry.addData("right front distance : ", driveBase.frontRightDistance());
            telemetry.update();
        }
        telemetry.addData("run-time : ", (driveBase.runtime.seconds()));
    }
}
