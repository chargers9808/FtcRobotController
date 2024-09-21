package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;

@TeleOp(group = "Test", name = "Locate Motors")
public class LocateMotors extends LinearOpMode9808 {

    @Override
    protected void run_9808() {
        while (opModeIsActive()) {
            if(!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                telemetry.addLine("Locate Motors");
                telemetry.addLine("Press A button to Run the Front Left");
                telemetry.addLine("Press B button to Run the Front Right");
                telemetry.addLine("Press X button to Run the Rear Left");
                telemetry.addLine("Press Y button to Run the Rear Right");
                telemetry.update();
                driveBase.stopMotors();
            }
            else if(gamepad1.a){
                driveBase.frontLeft.setPower(.4);
                telemetry.addLine("Left Front should be running");
                telemetry.update();
            }
            else if(gamepad1.b){
                driveBase.frontRight.setPower(.4);
                telemetry.addLine("Right Front should be running");
                telemetry.update();
            }
            else if(gamepad1.x){
                driveBase.backLeft.setPower(.4);
                telemetry.addLine("Left Rear should be running");
                telemetry.update();
            }
            else if(gamepad1.y){
                driveBase.backRight.setPower(.4);
                telemetry.addLine("Right Rear should be running");
                telemetry.update();
            }


        }   //          end of "While Opmode is active"
    }

    @Override
    protected void pre_init_9808() {
        driveBase.init(hardwareMap,this);// initialize hardware
    }

    @Override
    protected void init_9808() {

    }

    @Override
    protected Alliance getAlliance() {
        return null;
    }
}
