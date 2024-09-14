package org.firstinspires.ftc.deprecated.centerstage.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.deprecated.centerstage.auto.position.PositionBase;
import org.firstinspires.ftc.deprecated.centerstage.auto.position.WingBase;

@Autonomous(name = "Blue Wing (State)", group = "Auto")
@Disabled
public class Wing extends CenterstageBlueAuto {
    private final WingBase positionBase = new WingBase();
    protected PositionBase getPositionBase() { return positionBase; }

    protected void plow( String propLocation ) {
        switch (propLocation) {
            case "Left":
                driveBase.plowFromBlueRightStartToLeftSpike();
                break;
            case "Right":
                driveBase.plowFromBlueRightStartToRightSpike();
                break;
            case "Center":
                driveBase.plowFromBlueRightStartToCenterSpike();
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
                backdropDriveDistance = -70; // changed from -68
                backupDistance = -1;
                break;
            case "Right":
                backdropDriveDistance = -70;
                backupDistance = -4.5;
                break;
            case "Center":
                backdropDriveDistance = -83;
                driveBase.gyroTurn(.6,backwards);
                driveBase.tankDrive(.3,10); // To get behind the purple pixel
                backupDistance = 0;
                break;
        }

        driveBase.gyroTurn(.6,backwards);
        driveBase.tankDrive(.6,backupDistance);
        distanceFromWall = driveBase.rightDistanceToWall();
        driveBase.gyroTurn(.6, 0);
        if (modeSelection == Selection.PARK_BACKDROP) {
            sleep(5000);
        }
        driveBase.tankDrive(.6,(midParkDistFromWall-4)-distanceFromWall);
        driveBase.gyroTurn(.6,backwards);
        driveBase.tankDrive(.6,backdropDriveDistance);// drive under the stage door, into the middle
        driveBase.gyroTurn(.6, getBackdropDeg());
        sleep(200);
        distanceFromWall = driveBase.frontRightDistance();
        driveBase.tankDrive(.6,distanceFromWall-32);

        // Move from the current location 50in from the wall, to in front of the target tag
        if (!parkOnly) {
            driveBase.gyroTurn(.6, getBackdropDeg());
            driveBase.DriveSidewaysCorrected(.5, -1*(midParkDistFromWall-22+cameraOffset()),getBackdropDeg());
            distanceFromWall = driveBase.leftDistanceToWall();
            sleep(200);
            driveBase.DriveSidewaysCorrected(.4, -1*(distanceFromWall-getCurrentBackdropTagLocation()),getBackdropDeg());
            driveBase.gyroTurn(.6, getBackdropDeg());
            if (modeSelection == Selection.PARK_BACKDROP) {
                driveBase.tankDrive(.4,1);
            }
        }
    }
}
