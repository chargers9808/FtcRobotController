package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gobilda.Pose2D;

import java.util.Locale;

@TeleOp(group = "Test", name = "Odo")
public class OdometryTest extends LinearOpMode9808 {
    GoBildaPinpointDriver odometryComputer;
    double oldTime = 0;

    @Override
    protected void pre_init_9808() {
        driveBase.init(hardwareMap,this);
    }

    @Override
    protected void init_9808() {

    }

    @Override
    protected void run_9808() {
        intiOdometryComputer();

    }

    private void intiOdometryComputer(){
        odometryComputer = driveBase.getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        odometryComputer.setOffsets(-84.0, -168.0);
        odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryComputer.resetPosAndIMU();
        telemetry.addLine("X offset: " + odometryComputer.getXOffset());
        telemetry.addLine("Y offset: " + odometryComputer.getYOffset());
        telemetry.update();

        while(opModeIsActive()) {
            odometryComputer.bulkUpdate();
            if (gamepad1.a) {
                odometryComputer.resetPosAndIMU();
            }
            if (gamepad1.b) {
                odometryComputer.recalibrateIMU();
            }
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odometryComputer.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = odometryComputer.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odometryComputer.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odometryComputer.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }

    @Override
    protected Alliance getAlliance() {
        return null;
    }


    public double getFieldHeading() {
        return 0.0;
    }

    public void turnFieldCentric(double speed, double headingAngle){

    }
}
