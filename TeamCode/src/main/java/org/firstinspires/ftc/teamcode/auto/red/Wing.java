package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.position.PositionBase;
import org.firstinspires.ftc.teamcode.auto.position.WingBase;

@Autonomous(name = "Red Wing (State)", group = "Auto")
public class Wing extends RedAuto {
    private final WingBase positionBase = new WingBase();
    protected PositionBase getPositionBase() { return positionBase; }

    protected void plow( String propLocation ) {
        switch (propLocation) {
            case "Left":
                driveBase.plowFromRedWingStartToLeftSpike();
                break;
            case "Right":
                driveBase.plowFromRedWingStartToRightSpike();
                break;
            case "Center":
                driveBase.plowFromRedWingStartToCenterSpike();
                break;
        }
    }

    @Override
    protected void driveToBackdrop( String propLocation, boolean parkOnly ) {
        double backwards = -1 * getBackdropDeg();
        double backdropDriveDistance = 0;
        double distanceFromWall;
        double backupDistance = 0;

        driveBase.tilt.setPosition(driveBase.tiltToCarry);
        driveBase.arm.setTargetPosition(driveBase.armLowered);
        while(driveBase.arm.isBusy());

        switch (propLocation) {
            case "Left":
                backdropDriveDistance = -67;
                backupDistance = -1;
                break;
            case "Right":
                backdropDriveDistance = -67;
                backupDistance = -1;
                break;
            case "Center":
                backdropDriveDistance = -77;
                driveBase.gyroTurn(.6,backwards);
                driveBase.tankDrive(.3,10); // To get behind the purple pixel
                backupDistance = 0;
                break;
        }

        driveBase.gyroTurn(.6,backwards);
        driveBase.tankDrive(.6,backupDistance);
        distanceFromWall = driveBase.leftDistanceToWall();
        driveBase.gyroTurn(.6, 0);
        if (modeSelection == Selection.PARK_BACKDROP) {
            sleep(1300);
        }
        driveBase.tankDrive(.6,midParkDistFromWall-4-distanceFromWall);
        driveBase.gyroTurn(.6,backwards);
        driveBase.tankDrive(.6,backdropDriveDistance);// drive under the stage door, into the middle
        driveBase.gyroTurn(.6, getBackdropDeg());
        sleep(200);
        distanceFromWall = driveBase.frontRightDistance();
        driveBase.tankDrive(.6,distanceFromWall-32);

        // Move from the current location 54in from the wall, to in front of the target tag
        if (!parkOnly) {
            driveBase.gyroTurn(.6, getBackdropDeg());
            driveBase.DriveSidewaysCorrected(.5, (midParkDistFromWall-22+cameraOffset()),getBackdropDeg());
            distanceFromWall = driveBase.rightDistanceToWall();
            sleep(200);
            driveBase.DriveSidewaysCorrected(.4, (distanceFromWall-getCurrentBackdropTagLocation()),getBackdropDeg());
            driveBase.gyroTurn(.6, getBackdropDeg());
            if (modeSelection == Selection.PARK_BACKDROP) {
                driveBase.tankDrive(.4,1);
            }
        }
    }
}
