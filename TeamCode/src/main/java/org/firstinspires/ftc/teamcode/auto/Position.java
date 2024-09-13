package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Position {
    public enum Location {
        UNKNOWN,
        NET,
        OBSERVATION
    }

    private final Location position;
    private final String positionString;
    private final RevBlinkinLedDriver.BlinkinPattern staticColor;

    public Position() { this(Location.UNKNOWN); }

    public Position(Location position) {
        this.position = position;
        switch( position ) {
            case NET:
                this.positionString = "NET";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case OBSERVATION:
                this.positionString = "OBS";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case UNKNOWN:
            default:
                this.positionString = "UNK";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
        }
    }

    public String getPositionString() {
        return this.positionString;
    }

    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {
        return this.staticColor;
    }
}
