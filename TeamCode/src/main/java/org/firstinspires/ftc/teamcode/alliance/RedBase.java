package org.firstinspires.ftc.teamcode.alliance;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class RedBase extends AllianceBase {
    @Override
    public String getColorString() {return "RED";}
    @Override
    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {return RevBlinkinLedDriver.BlinkinPattern.RED;}
    @Override
    public RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor() {return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;}

    @Override
    public double getBackdropDeg() { return -90.0; }
}
