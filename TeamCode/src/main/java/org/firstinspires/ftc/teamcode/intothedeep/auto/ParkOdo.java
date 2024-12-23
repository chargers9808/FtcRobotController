package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side Auto
 */
@Autonomous(name = "Park Odo", group = "Auto")
public class ParkOdo extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
        driveBase.tankDrive( .8, 6);
        updateOdometryObservation();
        driveBase.driveTo(.8, 12.0, 120, 90.0);
        driveBase.gyroTurn(.8, 90);
        sleep(5000);
    }
}
