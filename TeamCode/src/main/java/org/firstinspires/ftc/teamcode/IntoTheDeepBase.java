package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.Position;

public abstract class IntoTheDeepBase extends LinearOpMode9808 implements GameBase {

    public boolean diagnosticMode = false;

    public void hardwareSetup( DraculaBase driveBase) {
        HeadingHolder.setHeading(0.0);
        driveBase.imu.resetYaw();
    }

    protected Position getPosition() {
        return null;
    }

    @Override
    protected Alliance getAlliance() {
        return null;
    }

    public void sweeperInOn(){
        //TODO: turn on sweeper in
    }

    public void sweeperInOff(){
        //TODO: turn off sweeper in
    }

    public void sweeperOutOn(){
        //TODO: turn on sweeper out
    }

    public void sweeperOutOff(){
        //TODO: turn off sweeper out
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

    /**
     * Prepare the bot for initialization and running
     */
    protected void pre_init_9808() {
        driveBase.init(hardwareMap, this);
        // Set LED state
        setLEDHeartbeat();
    }

    /**
     * Perform bot initialization
     */
    protected void init_9808() {
        // Call init for the specific opmode
        initialize();

        telemetry.addLine("ready for START");
        telemetry.addLine( "Alliance: " + getAlliance().getColorString());
        telemetry.addLine( "Position " + getPosition().getPositionString());
        telemetry.update();
    }


    protected abstract void initialize();
}
