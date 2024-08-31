package org.firstinspires.ftc.teamcode.alliance;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public abstract class AllianceBase {
    abstract public String getColorString();
    abstract public RevBlinkinLedDriver.BlinkinPattern getStaticColor();
    abstract public RevBlinkinLedDriver.BlinkinPattern getHeartbeatColor();
    abstract public double getBackdropDeg();
}
