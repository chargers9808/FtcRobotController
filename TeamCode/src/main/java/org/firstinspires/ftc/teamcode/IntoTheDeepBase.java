package org.firstinspires.ftc.teamcode;

public class IntoTheDeepBase implements GameBase {
    public void hardwareSetup( DraculaBase driveBase) {
        HeadingHolder.setHeading(0.0);
        driveBase.imu.resetYaw();
    }
}
