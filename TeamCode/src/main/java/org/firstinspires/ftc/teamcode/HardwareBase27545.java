package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

public class HardwareBase27545 extends DraculaBase {

    @Override
    void initRobotSpecificMotors() {
        intake = getDcMotor("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        lift = getDcMotor("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setTargetPosition(0);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = getDcMotor("slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setTargetPosition(0);
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    void initRobotSpecificHardware() {
        intiOdometryComputer();
    }
    private void intiOdometryComputer(){
        odometryComputer = getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        odometryComputer.setOffsets(-84.0, -168.0);
        odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryComputer.resetPosAndIMU();
    }

    public void positionToFieldCentric(double targetPosX, double targetPosY, double targetHeading, double posThreshold, double headingThreshold) {
        double heading = odometryComputer.getHeading();
        double distanceX = targetPosX - odometryComputer.getXOffset();
        double distanceY = targetPosY - odometryComputer.getYOffset();
        double headingError = targetHeading - heading;

        double speedX;
        double speedY;
        double speedR;

        double cosTheta;
        double sinTheta;

        double fieldX;
        double fieldY;

        double frontLeftWheelMotorPower;
        double frontRightWheelMotorPower;
        double backLeftWheelMotorPower;
        double backRightWheelMotorPower;

        while (((LinearOpMode) callingOpMode).opModeIsActive() &&
                Math.abs(headingError) > headingThreshold &&
                Math.abs(distanceX) > targetPosX + posThreshold &&
                Math.abs(distanceY) > targetPosY + posThreshold
        ) {
            heading = odometryComputer.getHeading();
            distanceX = targetPosX - odometryComputer.getXOffset();
            distanceY = targetPosY - odometryComputer.getYOffset();
            headingError = targetHeading - heading;

            speedX = Math.signum(distanceX);
            speedY = Math.signum(distanceY);
            speedR = headingError > 180.0 ? Math.signum(headingError - 360.0) : Math.signum(headingError + 360.0);

            if (Math.abs(headingError) < 50.) {
                speedR *= Math.abs(headingError) / 50.;
            }

            cosTheta = Math.cos(heading);
            sinTheta = Math.sin(heading);

            fieldX = speedX * cosTheta - speedY * sinTheta;
            fieldY = speedX * sinTheta + speedY * cosTheta;

            frontLeftWheelMotorPower = fieldY + speedR + fieldX;
            frontRightWheelMotorPower = fieldY - speedR - fieldX;
            backLeftWheelMotorPower = fieldY + speedR - fieldX;
            backRightWheelMotorPower = fieldY - speedR + fieldX;

            max = 1.0;
            max = Math.max(max, Math.abs(frontLeftWheelMotorPower));
            max = Math.max(max, Math.abs(frontRightWheelMotorPower));
            max = Math.max(max, Math.abs(backLeftWheelMotorPower));
            max = Math.max(max, Math.abs(backRightWheelMotorPower));

            frontLeftWheelMotorPower /= max;
            frontRightWheelMotorPower /= max;
            backLeftWheelMotorPower /= max;
            backRightWheelMotorPower /= max;

            setWheelMotorPower(frontLeftWheelMotorPower, frontRightWheelMotorPower, backLeftWheelMotorPower, backRightWheelMotorPower);
        }
        stopMotors();
    }
}
