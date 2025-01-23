package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FieldTracker;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side Park Auto
 */
@Autonomous(name = "Observation Park 9808", group = "Auto")
public class ParkOdo extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.OBSERVATION);
    protected Position getPosition() { return position; }

    protected void run_auto() {
        Pose2DGobilda pos;
        diagnosticMode = true;
        displayDiagnostics();

        travel();
        driveBase.driveTo(0.2, new Pose2DGobilda(DistanceUnit.INCH, 15, 129, AngleUnit.DEGREES, 90));
        driveBase.moveMotor(driveBase.arm, armLowered, 0.5, true);
    }
}
