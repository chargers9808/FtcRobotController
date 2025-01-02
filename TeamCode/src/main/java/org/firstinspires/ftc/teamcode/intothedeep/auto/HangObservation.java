package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

/**
 * Observation Side scoring Auto
 */
@Autonomous(name = "Score Observation", group = "Auto")
public class HangObservation extends IntoTheDeepAuto {
    private final Position position = new Position(Position.Location.UNKNOWN);
    protected Position getPosition() { return position; }
    private int hangingCount = 0;

    protected void getSpecimenParallel() {
        driveBase.driveTo(.3, 2.5, -25, 270.0);
        pickupSpecimen( false );
        closeGripper();
        sleep(300);
        travel();
    }

    protected void scoreSpecimen() {
        driveBase.driveTo(.3, 32, hangingCount * -2, 0.0);
        hangSpecimen();
        hangingCount += 1;
    }

    protected void run_auto() {
        hangingCount = 0;
        diagnosticMode = true;
        travel();
        scoreSpecimen();
        //driveBase.driveTo(.3, 32, 0, 0.0);
        //hangSpecimen();
        getSpecimenParallel();
        scoreSpecimen();
        //driveBase.driveTo(.2, 32, -2, 0.0);
        //hangSpecimen();
    }
}