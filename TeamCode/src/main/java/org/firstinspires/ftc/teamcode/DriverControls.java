package org.firstinspires.ftc.teamcode;

public class DriverControls {
    /**
     * Stick dead zone
     */
    private final double DEAD_ZONE = 0.02;

    /**
     * Distance to move with a D-pad creep
     */
    public final double CREEP_DIST = 0.1;
    /**
     * Drive speed fast
     */
    public final double SPEED_FACTOR_FAST = 1.3;
    /**
     * Drive speed slow
     */
    public final double SPEED_FACTOR_SLOW = 2.2;
    /**
     * Rotation speed slow
     */
    public final double SPEED_ROTATE_FACTOR_SLOW = 3.0;
    /**
     * Rotation speed fast
     */
    public final double SPEED_ROTATE_FACTOR_FAST = 1.0;

    /**
     * Trigger threshold
     */
    public final double TRIGGER_THRESHOLD = 0.1;

    protected LinearOpMode9808 opmode;
    private double speedFactor;
    private double rotationSpeed;
    public boolean fieldCentric;
    private double x = 0.0;
    private double y = 0.0;
    private double r = 0.0;
    private boolean togglingSpeed;
    private boolean togglingMode;

    /**
     * Initialize the driver controller
     * @param opmode The current opmode
     */
    public void init(LinearOpMode9808 opmode) {
        this.opmode = opmode;
        this.togglingSpeed = false;
        this.togglingMode = false;
        this.speedFactor = SPEED_FACTOR_FAST;
        this.rotationSpeed = SPEED_ROTATE_FACTOR_FAST;
        this.fieldCentric = true;
    }

    /**
     * Calculate driver controls for steering
     *
     * INPUTS: Left Stick
     */
    public void calculateDriveControls() {
        // get the steering commands from either gamepad #1 or #2
        double yCommand = (-opmode.gamepad1.left_stick_y - opmode.gamepad2.left_stick_y) / speedFactor; // forward and backward with respect to robot
        // (note: The joystick goes negative when pushed up, so we negate it)
        double xCommand = (opmode.gamepad1.left_stick_x + opmode.gamepad2.left_stick_x) / speedFactor;  // left and right with respect to robot
        r = (-opmode.gamepad1.right_stick_x - opmode.gamepad2.right_stick_x) / rotationSpeed;        // spin cw or ccw
        // create a steering "deadzone" near zero joystick deflection
        if (Math.abs(yCommand) < DEAD_ZONE) {
            yCommand = 0.;
        }
        if (Math.abs(xCommand) < DEAD_ZONE) {
            xCommand = 0.;
        }
        if (Math.abs(r) < DEAD_ZONE) {
            r = 0.;
        }
        // get the robot's heading from the IMU:
        double theta = opmode.driveBase.getFieldHeading() * Math.PI / 180.;// convert to 0-2Pi angle
        if (theta < 0) {
            theta = theta + 2. * Math.PI;
        }

        // for Field Centric, rotate the joystick commands into the frame of reference of the robot ("coordinate system rotation")
        x = xCommand * Math.cos(theta) + yCommand * Math.sin(theta);
        y = yCommand * Math.cos(theta) - xCommand * Math.sin(theta);
// or... for robot-centric steering, use the scaled joystick inputs directly

        if (!fieldCentric) { // make the joystick inputs non-linear to make it easier to control the rotation rate at slow speeds
            x = xCommand * xCommand* xCommand;
            y = yCommand * yCommand* yCommand;
        }
    }

    /**
     * Creep the bot with the D-Pad
     *
     * INPUT: D-pad (GP2 or GP1 w/o RB)
     */
    public void calculateDPadCreep() {
        if ((opmode.gamepad1.dpad_up && !opmode.gamepad1.right_bumper) || opmode.gamepad2.dpad_up ) { y=CREEP_DIST; }
        if ((opmode.gamepad1.dpad_down && !opmode.gamepad1.right_bumper)  || opmode.gamepad2.dpad_down) { y=-CREEP_DIST; }
        if ((opmode.gamepad1.dpad_left && !opmode.gamepad1.right_bumper)  || opmode.gamepad2.dpad_left) { x=-CREEP_DIST; }
        if ((opmode.gamepad1.dpad_right && !opmode.gamepad1.right_bumper)  || opmode.gamepad2.dpad_right) { x=CREEP_DIST; }
    }

    /**
     * Move the robot to a previously calculated position
     * @param driveBase
     */
    public void move(DraculaBase driveBase) {
        driveBase.applyMecPower2(x,y,r);
    }

    /**
     * Reset the Gyro
     *
     * INPUT: GP2.LS
     * @param driveBase
     */
    public void resetGyro(DraculaBase driveBase) {
        if (opmode.gamepad2.left_stick_button) {
            driveBase.imu.resetYaw();
            HeadingHolder.setHeading(0);
            driveBase.setLED(DraculaBase.LEDColor.GREEN);
        }
    }

    /**
     * Update drive mode based on driver input
     *
     * INPUT: GP1.RS
     */
    public void updateDriveMode() {
        if((opmode.gamepad1.back) && !togglingMode) {
            fieldCentric = !fieldCentric;
            togglingMode = true;
        } else if( !opmode.gamepad1.back) {
            togglingMode = false;
        }
    }

    public void creepSpeed() {
        if (opmode.gamepad1.right_stick_button || opmode.gamepad1.x) {
            rotationSpeed = SPEED_ROTATE_FACTOR_SLOW;
        }
        else if (opmode.gamepad1.b){
            rotationSpeed = SPEED_ROTATE_FACTOR_FAST;
        }
        else {
            rotationSpeed = SPEED_ROTATE_FACTOR_FAST;
        }
    }

    /**
     * Update speed factor based on driver input
     *
     * INPUT: GP1.LS
     */
    public void updateSpeedFactor() {
        if((opmode.gamepad1.left_stick_button) && !togglingSpeed) {
            if( speedFactor == SPEED_FACTOR_FAST ) {
                speedFactor = SPEED_FACTOR_SLOW;
            } else {
                speedFactor = SPEED_FACTOR_FAST;
            }
            togglingSpeed = true;
        } else if( !opmode.gamepad1.left_stick_button) {
            togglingSpeed = false;
        }
    }

    public boolean triggered(float trigger) {
        return trigger >= TRIGGER_THRESHOLD;
    }
}
