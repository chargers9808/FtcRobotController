package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.DraculaBase.*;

public class Position {
    public enum Location {
        UNKNOWN,
        NET,
        OBSERVATION
    }

    private final Location position;
    private final String positionString;
    private final RevBlinkinLedDriver.BlinkinPattern staticColor;
    private final SensorDir sensorDir;

    public Position() { this(Location.UNKNOWN); }

    public Position(Location position) {
        this.position = position;
        switch( position ) {
            case NET:
                this.positionString = "NET";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                this.sensorDir = SensorDir.LEFT;
                break;
            case OBSERVATION:
                this.positionString = "OBS";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                this.sensorDir = SensorDir.RIGHT;
                break;
            case UNKNOWN:
            default:
                this.positionString = "UNK";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                this.sensorDir = SensorDir.RIGHT;
                break;
        }
    }

    public String getPositionString() {
        return this.positionString;
    }

    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {
        return this.staticColor;
    }

    public SensorDir getSensor() {return this.sensorDir;}

}
