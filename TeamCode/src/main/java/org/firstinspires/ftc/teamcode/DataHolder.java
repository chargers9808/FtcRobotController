
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//Class created to store the last known heading of the IMU
// and arm position at the end of autonomous

import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;

public class DataHolder {

    //lastHeading, accessed by get and setHeading. Starts at 0.
    private static double lastHeading = 0;
    private static int lastArmPosition = 0;
    private static int lastSlidePosition = 0;
    private static boolean hasOdometry = false;
    private static GoBildaPinpointDriver odometryComputer;

    //private static
    public static void setAll( double heading, int arm, int slide) {
        setHeading(heading);
        setArm(arm);
        setSlide(slide);
    }

    //Setter method
    public static void setHeading(double heading){
        lastHeading=heading;
    }

    //Getter method
    public static double getHeading(){
        return lastHeading;
    }

    //Setter method
    public static void setArm(int position){
        lastArmPosition=position;
    }

    //Getter method
    public static int getArm(){
        return lastArmPosition;
    }

    //Setter method
    public static void setSlide(int position){
        lastSlidePosition=position;
    }

    //Getter method
    public static int getSlide(){
        return lastSlidePosition;
    }

    public static void setOdometry(GoBildaPinpointDriver odometry) {
        odometryComputer = odometry;
        hasOdometry = true;
    }

    public static boolean getOdometryEnabled() {
        return hasOdometry;
    }

    public static GoBildaPinpointDriver getOdometryComputer() {
        return odometryComputer;
    }
}