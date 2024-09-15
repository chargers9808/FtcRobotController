package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

abstract public class IntoTheDeepAuto extends IntoTheDeepBase {
    protected void autoPreInit() {

    }

    protected void autoInit() {
        driveBase.setLED( getPosition().getStaticColor() );
    }

    protected void autoRun() {

    }
}
