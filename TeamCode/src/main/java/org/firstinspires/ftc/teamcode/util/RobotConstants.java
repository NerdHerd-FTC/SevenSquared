package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class RobotConstants {
    public static double WRIST_LOWER_LIMIT = 0.0;
    public static double WRIST_UPPER_LIMIT = 1.0;
    public static double WRIST_GROUND = 0.55; // TUNE THIS FOR MACRO

    public static double CLAW_RIGHT_CLOSED = 0.0;
    public static double CLAW_RIGHT_OPEN = 1.0;

    public static double CLAW_LEFT_CLOSED = 0.6;
    public static double CLAW_LEFT_OPEN = 0.0;

    // tune...
    public static double ARM_LOWER_LIMIT = 0.0;
    public static double ARM_UPPER_LIMIT = 1000;

    public static double ARM_SPEED = 0.8;

    // tune...
    public static Integer ARM_HOME = 0;
    public static Integer ARM_SCORE = 0; // TUNE THIS FOR MACRO
    public static Integer ARM_GROUND = 1000; // TUNE THIS FOR MACRO

    // tune...
    public static int JOINT_LOWER_LIMIT = 0;
    public static int JOINT_UPPER_LIMIT = 1000;

    public static double JOINT_SPEED = 1.0;

    // tune...
    public static Integer JOINT_HOME = 0;
    public static Integer JOINT_SCORE = 1000;
    public static Integer JOINT_GROUND = 0;

    //public static double jointP = 0.00009, jointI = 0.0, jointD = 0.0, joint_norm_F = 0.000047, joint_extra_F = 0.0;
    public static double armP = 0.0035, armI = 0.0, armD = 0.0003, armF = 0.002;

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