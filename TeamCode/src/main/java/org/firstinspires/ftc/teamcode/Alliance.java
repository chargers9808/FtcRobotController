package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Alliance {
    public enum Color {
        UNKNOWN,
        BLUE,
        RED
    }
    private final Color alliance;
    private final String colorString;
    private final RevBlinkinLedDriver.BlinkinPattern staticColor;
    private final RevBlinkinLedDriver.BlinkinPattern heartbeatColor;

    public Alliance() {
        this(Color.UNKNOWN);
    }

    public Alliance(Color alliance ) {
        this.alliance = alliance;
        switch( alliance ) {
            case BLUE:
                this.colorString = "BLUE";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                this.heartbeatColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                break;
            case RED:
                this.colorString = "RED";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.RED;
                this.heartbeatColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                break;
            case UNKNOWN:
            default:
                this.colorString = "UNKNOWN";
                this.staticColor = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                this.heartbeatColor = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
                break;
        }
    }

    public String getColorString() {
        return this.colorString;
    }

    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {
        return this.staticColor;
    }

    public RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor() {
        return this.heartbeatColor;
    }
}
