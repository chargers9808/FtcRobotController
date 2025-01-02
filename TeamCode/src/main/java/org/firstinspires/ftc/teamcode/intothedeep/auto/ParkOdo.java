package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side Park Auto
 */
@Autonomous(name = "Park Odo", group = "Auto")
public class ParkOdo extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
        Pose2DGobilda pos;
        diagnosticMode = true;
        pos = driveBase.odometryComputer.getPosition();
        driveBase.tankDrive( .8, 4);
        driveBase.odometryComputer.bulkUpdate();
        travel();
        driveBase.driveTo(.5, 8.0, -36, 90.0);
        driveBase.moveMotor(driveBase.arm, armLowered, 0.5, true);
    }
}
