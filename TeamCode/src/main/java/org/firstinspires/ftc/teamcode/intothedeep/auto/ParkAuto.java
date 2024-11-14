package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.intothedeep.auto.blue.IntoTheDeepBlueAuto;

/**
 * Net side code for Red Alliance
 */
@Autonomous(name = "Park 9808", group = "Auto")
public class ParkAuto extends IntoTheDeepBlueAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }

    protected void run_auto() {
      driveBase.tankDrive( .8, 6);
      driveBase.driveSidewaysUntil(.8, 12, true);
      driveBase.gyroTurn(.8, 90);
      sleep(5000);
    }
}
