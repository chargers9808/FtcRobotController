package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Observation Side Auto
 */
@Autonomous(name = "Park 9808", group = "Auto")
@Disabled
public class ParkAuto extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
      driveBase.tankDrive( .8, 6);
      driveBase.driveSidewaysUntil(.8, 12, true);
      driveBase.gyroTurn(.8, 90);
      sleep(5000);
    }
}
