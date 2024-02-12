package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    /*
    public static double WRIST_LOWER_LIMIT = 0.0;
    public static double WRIST_UPPER_LIMIT = 1.0;
    public static double WRIST_GROUND = 0.55; // TUNE THIS FOR MACRO
     */

    public static double CLAW_RIGHT_CLOSED =0.52;
    public static double CLAW_RIGHT_OPEN = 0.25;

    public static double CLAW_LEFT_OPEN = 0.4;
    public static double CLAW_LEFT_CLOSED =0.66;

    // tune...
    public static double ARM_LOWER_LIMIT = 0.0;
    public static double ARM_UPPER_LIMIT = 1000;

    // tune...
    public static Integer ARM_HOME = 0;
    public static Integer ARM_BACKWARDS_SCORE = 1275;
    public static Integer ARM_GROUND = 1200; // TUNE THIS FOR MACRO
    public static Integer ARM_FORWARDS_SCORE = 730;
    public static Integer ARM_FORWARDS_LOW_SCORE = 800;
    public static Integer ARM_AIRPLANE = 2160;
    public static Integer ARM_DROP_1 = 800;
    public static Integer ARM_DROP_2 = 780;


    // for auto mainly
    public static Integer ARM_PIXEL_DEPTH_1 = 1040;
    public static Integer ARM_PIXEL_DEPTH_2 = 1050;
    public static Integer ARM_PIXEL_DEPTH_3 = 1100;
    public static Integer ARM_PIXEL_DEPTH_4 = 1150;
    public static Integer ARM_PIXEL_DEPTH_5 = ARM_GROUND;

    public static double JOINT_SPEED = 1.0;

    // tune...
    public static Integer JOINT_HOME = 0;
    public static Integer JOINT_BACKWARDS_SCORE = -2650;
    public static Integer JOINT_FORWARDS_SCORE = 0;
    public static Integer JOINT_GROUND = 0;
    public static Integer JOINT_AIRPLANE = -1430;

    public static double jointP = 0.002, jointI = 0.0, jointD = 0.000018, joint_norm_F = 0.0015;
    public static double armP = 0.0032, armI = 0.0, armD = 0.00007, armF = 0.0037;

    // Drive
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_TURN = 0.5;

    // CONTROLLERS
    public static double DEAD_BAND = 0.1;

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

    public static double joint_ticks_per_degree = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28 * 10.0/3) / 360.0;
    public static double arm_ticks_per_degree = ((((1+(46.0/17))) * (1+(46.0/11))) * 28 * 5)/360.0;

}