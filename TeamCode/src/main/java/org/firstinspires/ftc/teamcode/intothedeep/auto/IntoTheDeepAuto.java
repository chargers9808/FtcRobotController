package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.DataHolder;
import org.firstinspires.ftc.teamcode.FieldTracker;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;
import org.firstinspires.ftc.teamcode.intothedeep.IntoTheDeepBase;

abstract public class IntoTheDeepAuto extends IntoTheDeepBase {

    protected double sample1Offset = 22.0;
    //15
    protected double sample2Offset = 14.0;
    protected double forwardDistance = 7.0;

    protected final int AUTO_ARM_SCORE_POS = -1600; //-1700
    protected final double SCORE_FRONT_DIST = 5.15;
    protected final double SCORE_LEFT_DIST = 8.34;

    abstract protected Position getPosition();
    @Override
    protected void pre_initialize() {

    }

    @Override
    protected void initialize() {
        driveBase.imu.resetYaw();
        DataHolder.setAll(0.0,0,0);

        initOdometry();
        DataHolder.setOdometry(driveBase.odometryComputer);

//        FieldTracker.findPosition( getPosition().getSensor() );
        if( getPosition().getLocation() == Position.Location.NET) {
            FieldTracker.setBotRef(POS_NET_START);
        } else {
            FieldTracker.setBotRef(POS_OBS_START);
        }

        closeGripper();
        sleep(100);
        driveBase.setLED( getPosition().getColor() );
        telemetry.addLine( "Position: " + getPosition().getPositionString());
        telemetry.addData( "Left Distance", driveBase.leftDistanceToWall());
        telemetry.addData( "Heading", driveBase.odometryComputer.getHeading() );
    }

    protected void scoreAuto() {
        scoreAuto( 135.0 );
    }

    protected void scoreAuto( double dropAngle ) {
        // NOTE:
        // Expects to start this at the position
        // x = SCORE_LEFT_DIST
        // y = SCORE_FRONT_DIST
        setGripRotation(Grip_Position.GRIP_90DEG);
        driveBase.gyroTurn(0.5, dropAngle);
//      driveBase.tankDrive(.1, 2);
        driveBase.driveTo(.1, 14, 15, dropAngle);
        sleep(100); //200
        driveBase.moveMotor(driveBase.arm, AUTO_ARM_SCORE_POS-80, 0.5, true); //power:.2
        sleep(150);
        scoreSample();
    }

    abstract protected void run_auto();

    /**
     * Run AUTO opmode
     */
    @Override
    protected void run_9808() {
        if( opModeIsActive() ) {
            resetRuntime();
            displayDiagnostics();
            // Call the run code for the specific opmode
            run_auto();
        }
        finish();
    }

    /**
     * Finalize AUTO phase in prep for Teleop
     */
    private void finish() {
        DataHolder.setAll(
                driveBase.getFieldHeading(),
                driveBase.arm.getCurrentPosition(),
                driveBase.slide.getCurrentPosition()
        );
        driveBase.moveMotor(driveBase.arm, 0, 1, false);
        driveBase.moveMotor(driveBase.slide, 0, 1, false);
        driveBase.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveBase.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        driveBase.arm.setPower(0);
        driveBase.slide.setPower(0);
        setLEDHeartbeat();
        telemetry.addData( "Runtime", getRuntime());
        telemetry.addData("Path", "Complete");

        telemetry.update();
        while (opModeIsActive());
    }
}
