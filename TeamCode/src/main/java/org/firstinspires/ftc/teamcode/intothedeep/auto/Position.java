package org.firstinspires.ftc.teamcode.intothedeep.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.LED;

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
    private final LEDColor color;
    private final SensorDir sensorDir;

    public Position() { this(Location.UNKNOWN); }

    public Position(Location position) {
        this.position = position;
        switch( position ) {
            case NET:
                this.positionString = "NET";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                this.sensorDir = SensorDir.LEFT;
                this.color = LEDColor.ORANGE;
                break;
            case OBSERVATION:
                this.positionString = "OBS";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                this.sensorDir = SensorDir.RIGHT;
                this.color = LEDColor.VIOLET;
                break;
            case UNKNOWN:
            default:
                this.positionString = "UNK";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                this.sensorDir = SensorDir.RIGHT;
                this.color = LEDColor.WHITE;
                break;
        }
    }

    public String getPositionString() {
        return positionString;
    }

    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {
        return staticColor;
    }
    public LEDColor getColor() {
        return color; }

    public SensorDir getSensor() {
        return sensorDir;
    }

    public static LEDColor posColor(Location loc) {
        return new Position(loc).getColor();
    }
}
