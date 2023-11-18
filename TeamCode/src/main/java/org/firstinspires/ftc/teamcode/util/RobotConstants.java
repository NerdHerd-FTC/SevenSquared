package org.firstinspires.ftc.teamcode.util;

public class RobotConstants {
    public static double WRIST_LOWER_LIMIT = 0.0;
    public static double WRIST_UPPER_LIMIT = 1.0;

    public static double CLAW_RIGHT_CLOSED = 0.0;
    public static double CLAW_RIGHT_OPEN = 1.0;

    public static double CLAW_LEFT_CLOSED = 0.6;
    public static double CLAW_LEFT_OPEN = 0.0;

    // tune...
    public static double ARM_LOWER_LIMIT = 0.0;
    public static double ARM_UPPER_LIMIT = 1000;

    public static double ARM_SPEED = 0.8;

    // tune...
    public static double ARM_HOME = 0.0;
    public static double ARM_SCORE = 1000;
    public static double ARM_GROUND = 1000;

    // tune...
    public static double JOINT_LOWER_LIMIT = 0.0;
    public static double JOINT_UPPER_LIMIT = 1000;

    public static double JOINT_SPEED = 1.0;

    // tune...
    public static double JOINT_HOME = 0.0;
    public static double JOINT_SCORE = 1000;
    public static double JOINT_GROUND = 0.0;

    // AUTONOMOUS
    //11 or 7.5
    private static double ROBOT_RADIUS_INCHES = 8; // Half the distance between left and right wheels
    private static double DEGREES_TO_INCHES = Math.PI * 2 * ROBOT_RADIUS_INCHES / 360;

    // Pulled from "encoder resolution formula": https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static double TICKS_PER_REV = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    // Pulled from strafer kit - converts mm. to in.
    private static double WHEEL_DIAMETER_INCH = 96/25.4;
    public static double DRIVE_TICKS_PER_INCH = (TICKS_PER_REV) / (WHEEL_DIAMETER_INCH * Math.PI);

    public static double DRIVE_TICKS_PER_DEGREE = DRIVE_TICKS_PER_INCH * DEGREES_TO_INCHES;

}