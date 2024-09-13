package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.AutoBase;

abstract public class IntoTheDeepAuto extends AutoBase {
    protected void autoPreInit() {

    }

    protected void autoInit() {
        driveBase.setLED( getPosition().getStaticColor() );
    }

    protected void autoRun() {

    }
}
