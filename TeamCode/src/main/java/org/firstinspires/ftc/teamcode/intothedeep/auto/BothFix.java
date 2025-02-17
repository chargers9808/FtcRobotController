package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Auto 9808 (Fixed)", group = "Auto")
public class BothFix extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }

    protected void collectAndScore( double angle, int slidePos, Grip_Position gripPos, double driveDist, double scoreAngle) {
        driveBase.gyroTurn(0.5, angle );
        setGripRotation(gripPos);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-250, .7, false); //250
        sleep(50);
        driveBase.tankDrive(0.5, driveDist);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-50, 0.5, true);
        //driveBase.moveMotor(driveBase.slide, slidePos, 0.5, true);
        //sleep(100);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth, 0.3, false);

        sleep(50); //250
        closeGripper();
        sleep(175); //300

        driveBase.moveMotor(driveBase.arm, AUTO_ARM_SCORE_POS-200, 0.8, true);
        driveBase.tankDrive(0.5, -1 * driveDist);
        scoreAuto(scoreAngle);
    }

    protected void run_auto() {
//        driveBase.moveMotor(driveBase.arm, armTravelPosition+100, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition+1350, 0.8, true);
        driveBase.moveMotor(driveBase.slide, slideOut, .6, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.5, false);

        // Score preloaded
//        driveBase.tankDrive(.5, 6);
        driveBase.driveTo(.5,20, 44, 0, 2, 45);
//        driveBase.driveTo(.5, 18, 22, 135, 2, 10);
//        driveBase.driveTo(.5, 16, 15, 135);
        driveBase.driveTo(.3, 14, 15, 135);

        scoreAuto(); // Loaded

        double driveDist = 2.5;

        collectAndScore( 4.0, slideOut, Grip_Position.GRIP_0DEG, driveDist, 140.0); //3.0
        collectAndScore( 344, slideOut, Grip_Position.GRIP_0DEG, driveDist+1, 140.0); //3 345

        driveBase.gyroTurn(0.5, 16);
        setGripRotation(2);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-250, 0.7, false); // - 40
        sleep(50);
        driveBase.tankDrive(0.5, driveDist+1);//3

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-100, 0.5, true); //-50

        driveBase.gyroTurn(.3, 21);
        setGripRotation(3);
        sleep(150);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth, 0.3, false);

        sleep(100); //250
        closeGripper();
        sleep(175); //300

        //driveBase.gyroTurn(0.5, 16); // 20
        driveBase.moveMotor(driveBase.arm, AUTO_ARM_SCORE_POS-200, 0.7, true);
        driveBase.tankDrive(0.5, -1 * driveDist+1); //3
        scoreAuto(135);
        sleep(50);

        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.7, false);
        driveBase.moveMotor(driveBase.slide, slideIn, .7, false);
        driveBase.tankDrive(.7, -4);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDown, 0.7, false);
        driveBase.gyroTurn(0.5, 0);
    }
}
