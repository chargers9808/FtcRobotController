package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side Auto
 */
@Autonomous(name = "Park Odo", group = "Auto")
@Disabled
public class ParkOdo extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
        Pose2DGobilda pos;
        diagnosticMode = true;
        pos = driveBase.odometryComputer.getPosition();
        driveBase.tankDrive( .8, 4);
        driveBase.odometryComputer.bulkUpdate();

        updateOdometryObservation();
        sleep(2000);

        driveBase.driveTo(.1, 12.0, 120, 90.0);
        //driveBase.driveTo(.1, 24.0, 120, 0.0);
        //positionTo2(12.0, -36.0, 90.0, 0.1);

        driveBase.odometryComputer.bulkUpdate();
        pos = driveBase.odometryComputer.getPosition();
        telemetry.addLine("X              : " + pos.getX(DistanceUnit.INCH));
        telemetry.addLine("Y              : " + pos.getY(DistanceUnit.INCH));
        telemetry.addLine("Heading        : " + pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        sleep(5000);
    }
}
