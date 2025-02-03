package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DraculaBase;

@Autonomous(name = "Net Auto 9808", group = "Auto")
public class Both extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }

    protected void collectAndScore( double angle, int slidePos, Grip_Position gripPos, double driveDist, double scoreAngle, boolean score) {
        driveBase.gyroTurn(0.5, angle );
        setGripRotation(gripPos);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-275, .8, false);
        sleep(50);
        driveBase.tankDrive(0.5, driveDist);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-50, .5, true);
        //driveBase.moveMotor(driveBase.slide, slidePos, 0.5, true);
        //sleep(100);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth, 0.3, false);

        sleep(50); //250
        closeGripper();
        sleep(175); //300

        if( score ) {
            driveBase.moveMotor(driveBase.arm, AUTO_ARM_SCORE_POS-200, 0.6, true);
            driveBase.tankDrive(0.5, -1 * driveDist);
            scoreAuto(scoreAngle);
        }
    }

    protected void park() {
        driveBase.gyroTurn(.5, 350);
        driveBase.moveMotor(driveBase.arm, armCollectPositionUp, 0.4, false);
        driveBase.moveMotor(driveBase.slide, slideIn, 0.4, false);
        driveBase.tankDrive(0.5, 20);
    }

    protected void run_auto() {
//        driveBase.moveMotor(driveBase.arm, armTravelPosition+100, 0.8, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition+1100, 0.8, true);
        driveBase.moveMotor(driveBase.slide, slideOut, .6, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.4, false);

        // Score preloaded
//        driveBase.tankDriveUntil(.5, SCORE_LEFT_DIST-2, false, false);
//        driveBase.gyroTurn(0.5, 90.0);
//        driveBase.tankDriveUntil(.4, SCORE_FRONT_DIST+5, true, false);
//        driveBase.driveSidewaysUntil(.5, SCORE_LEFT_DIST, false);
        driveBase.tankDriveUntil(.5, 6, false, false);
        driveBase.driveTo(.4, 16, 14, 135); //y15 10:25 1/18

        scoreAuto(); // Loaded

        collectAndScore( 3.0, slideOut, Grip_Position.GRIP_0DEG, 2, 140.0, true);
        collectAndScore( 345, slideOut, Grip_Position.GRIP_0DEG, 3, 140.0, true); //3
//        collectAndScore( 21.5, slideOut, Grip_Position.GRIP_135DEG, 2, 140.0, true);

        driveBase.gyroTurn(0.5, 17);
        setGripRotation(2);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-50, 0.7, false); //pwr .5
        sleep(50);
        driveBase.tankDrive(0.5, 3); //3
        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth-50, 0.5, true);

        setGripRotation(3);
        driveBase.gyroTurn(.3, 22);
        sleep(150);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDownBoth, 0.2, false);

        sleep(50); //250
        closeGripper();
        sleep(175); //300

        driveBase.gyroTurn(0.5, 20);
        driveBase.moveMotor(driveBase.arm, AUTO_ARM_SCORE_POS-200, 0.6, true);
        driveBase.tankDrive(0.5, -1 * 3); //3
        scoreAuto(135);
        sleep(50);

        //park();
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.6, false);
        driveBase.moveMotor(driveBase.slide, +100, .6, false);
        driveBase.tankDrive(.7, -2);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDown, 0.6, false);
        driveBase.gyroTurn(0.5, 0);
    }
}
