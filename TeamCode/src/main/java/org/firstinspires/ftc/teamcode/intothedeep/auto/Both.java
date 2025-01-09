package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DraculaBase;

@Autonomous(name = "Pickup Two 9808", group = "Auto")
public class Both extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.NET);
    protected Position getPosition() { return position; }

    protected void collectAndScore( double angle, int slidePos, Grip_Position gripPos, double driveDist, double scoreAngle, boolean score) {
        driveBase.gyroTurn(0.5, angle );
        setGripRotation(gripPos);

        driveBase.moveMotor(driveBase.arm, armCollectPositionDown-50, 0.5, false);
        sleep(50);
        driveBase.tankDrive(0.5, driveDist);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDown-50, 0.5, true);
        //driveBase.moveMotor(driveBase.slide, slidePos, 0.5, true);
        //sleep(100);
        driveBase.moveMotor(driveBase.arm, armCollectPositionDown, 0.2, false);

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
        driveBase.moveMotor(driveBase.arm, armTravelPosition+100, 0.8, false);
        driveBase.moveMotor(driveBase.slide, slideOut, .3, false);
        driveBase.moveMotor(driveBase.arm, armTravelPosition, 0.4, false);

        // Score preloaded
        driveBase.tankDriveUntil(.5, SCORE_LEFT_DIST-2, false, false);
        driveBase.gyroTurn(0.5, 90.0);
        driveBase.tankDriveUntil(.5, SCORE_FRONT_DIST+5, true, false);
        driveBase.driveSidewaysUntil(.5, SCORE_LEFT_DIST, false);

        scoreAuto(); // Loaded

        collectAndScore( 5.0, slideOut, Grip_Position.GRIP_0DEG, 2, 140.0, true);
        collectAndScore( 349.0, slideOut, Grip_Position.GRIP_45DEG, 2, 140.0, true);
        //collectAndScore( 46, slideOut, Grip_Position.GRIP_45DEG, 1, 140.0, true);

        //park();
        driveBase.gyroTurn(0.5, 0);
    }
}
