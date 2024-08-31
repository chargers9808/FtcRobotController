package org.firstinspires.ftc.teamcode.alliance;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class BlueBase extends AllianceBase {
    @Override
    public String getColorString() {return "BLUE";}
    @Override
    public RevBlinkinLedDriver.BlinkinPattern getStaticColor() {return RevBlinkinLedDriver.BlinkinPattern.BLUE;}
    @Override
    public RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor() {return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;}
    @Override
    public double getBackdropDeg() { return 90.0; }
}
