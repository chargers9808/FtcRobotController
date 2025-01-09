package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

public class FieldTracker {
    /**
     * Width of the bot
     */
    private static final double BOT_WIDTH = 18.0;
    /**
     * Depth of the bot
     */
    private static final double BOT_DEPTH = 18.0;

    /**
     * Reference to the DraculaBase
     */
    private static DraculaBase driveBase;

    /**
     * Offset from the rear distance sensor to the center line of the bot
     */
    private static double offsetX = 9.0;

    /**
     * Offset from the left distance sensor to the center line of the bot
     */
    private static double offsetLeft = 9.0;

    /**
     * Offset from the right distance sensor to the center line of the bot
     */
    private static double offsetRight = 9.0;

    /**
     * Reference point used to convert to field-centric positions
     */
    private static Pose2DGobilda botRef = new Pose2DGobilda(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    /**
     * Multiplier used to invert Y values if the Y-axis is inverted
     */
    private static int yMultiplier = 1;

    /**
     * Initialize the field tracker for this bot
     *
     * @param base DraculaBase
     * @param xOffset Offset from the rear distance sensor to the center line of the bot
     * @param leftOffset Offset from the left distance sensor to the center line of the bot
     * @param rightOffset Offset from the right distance sensor to the center line of the bot
     * @param revY Reverse Y direction? false => Y increases when moving left.
     */
    public static void initialize( DraculaBase base, double xOffset, double leftOffset, double rightOffset, boolean revY ) {
        driveBase   = base;
        offsetX = xOffset;
        offsetLeft  = leftOffset;
        offsetRight = rightOffset;
        yMultiplier = 1;
        if( revY ) {
            yMultiplier *= -1;
        }
    }

    /**
     * Drive to the specified field-centric position
     *
     * @param speed Speed to travel
     * @param pos   Field-centric position to travel to
     */
    public static void driveTo( double speed, Pose2DGobilda pos ) {
        driveBase.driveTo( speed, fieldToBot(pos) );
    }

    /**
     * Find the position of the bot based on starting position of the bot
     *
     * @param sensor Which left/right sensor to use to locate the bot
     */
    public static void findPosition(DraculaBase.SensorDir sensor ) {
        if( sensor == DraculaBase.SensorDir.LEFT) {
            findPositionLeft();
        } else if (sensor == DraculaBase.SensorDir.RIGHT) {
            findPositionRight();
        }
    }

    public static Pose2DGobilda getBotRef() {
        return botRef;
    }

    /**
     * Set the bot reference location based on the left distance sensor
     */
    public static void findPositionLeft() {
        double currY = driveBase.distanceToWall(DraculaBase.SensorDir.LEFT) + offsetLeft;
        botRef = new Pose2DGobilda(DistanceUnit.INCH, offsetX, currY, AngleUnit.DEGREES, 0.0);
    }

    /**
     * Set the bot reference location based on the right distance sensor
     */
    public static void findPositionRight() {
        double currY = 144 - ( driveBase.distanceToWall(DraculaBase.SensorDir.RIGHT) + offsetRight);
        botRef = new Pose2DGobilda(DistanceUnit.INCH, offsetX, currY, AngleUnit.DEGREES, 0.0);
    }

    /**
     * Translate a field-centric position to a bot-centric position
     *
     * @param pos Field-centric position to translate
     * @return Translated bot-centric position
     */
    public static Pose2DGobilda fieldToBot( Pose2DGobilda pos ) {
        double cvtX = pos.getX(DistanceUnit.INCH) - botRef.getX(DistanceUnit.INCH);
        double cvtY = (pos.getY(DistanceUnit.INCH) - botRef.getY(DistanceUnit.INCH));

        return new Pose2DGobilda(
                DistanceUnit.INCH, cvtX, cvtY * yMultiplier,
                AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES)
                );
    }

    /**
     * Translate a bot-centric position to a field-centric position
     *
     * @param pos Bot-centric position to translate
     * @return Translated field-centric position
     */
    public static Pose2DGobilda botToField( Pose2DGobilda pos ) {
        double cvtX = pos.getX(DistanceUnit.INCH) - botRef.getX(DistanceUnit.INCH);
        double cvtY = (pos.getY(DistanceUnit.INCH) - botRef.getY(DistanceUnit.INCH));
        return new Pose2DGobilda(
            DistanceUnit.INCH, cvtX, cvtY,
                AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES)
            );
    }
}
