package org.firstinspires.ftc.teamcode.util;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

@Config
public class AutoUtil {
    public LinearOpMode opMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, joint;
    public Servo ClawServoLeft, ClawServoRight;
    public ColorSensor topColorSensor, bottomColorSensor;
    public Telemetry telemetry;
    public static boolean debug = true;
    private ElapsedTime callGap = new ElapsedTime();
    private ElapsedTime lockTimeout = new ElapsedTime();
    private int callsToColor = 0;

    public static int armUpperLimit = 1070;
    public static int armLowerLimit = 1020;

    public Integer bottomRed;
    public Integer bottomGreen;
    public Integer bottomBlue;

    public Integer topRed;
    public Integer topGreen;
    public Integer topBlue;

    public static int whiteRedThresholdBottom = 200;
    public static int whiteRedThresholdTop = 350;

    public ElapsedTime pixelLock = new ElapsedTime();

    private PIDController armPID = new PIDController(armP, armI, armD);
    private PIDController jointPID = new PIDController(jointP, jointI, jointD);

    public enum RobotState {
        IDLE,
        FOLLOWING_TRAJECTORY,
        PIXEL_STACK,
        PIXEL_DROPOFF,
        ERROR
    }

    public enum ARM_DEMANDS {
        IDLE,
        MOVE_UP,
        MOVE_DOWN,
        HOLD
    }

    public RobotState currentState = RobotState.IDLE;
    public ARM_DEMANDS armDemands = ARM_DEMANDS.IDLE;

    // Constructor
    public AutoUtil(LinearOpMode opMode, DcMotor arm, DcMotor joint, Servo ClawServoLeft, Servo ClawServoRight, ColorSensor topColorSensor, ColorSensor bottomColorSensor, Telemetry telemetry) {
        this.opMode = opMode;
        this.arm = arm;
        this.joint = joint;
        this.ClawServoLeft = ClawServoLeft;
        this.ClawServoRight = ClawServoRight;
        this.topColorSensor = topColorSensor;
        this.bottomColorSensor = bottomColorSensor;
        this.telemetry = telemetry;
    }

    private boolean isWhite(ColorSensor sensor, int threshold, String name) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        if (Objects.equals(name, "TOP")) {
            topBlue = blue;
            topRed = red;
            topGreen = green;
        } else if (Objects.equals(name, "BOTTOM")) {
            bottomBlue = blue;
            bottomRed = red;
            bottomGreen = green;
        }

        return red > threshold && green > threshold && blue > threshold;
    }

    // Add something to adjust the robot when it is stuck - strafe left/move back

    public double lockOntoPixel() {
        if (callGap.milliseconds() > 300) {
            callGap.reset();
            // Check if top and bottom sensors detect white
            boolean topDetectsWhite = isWhite(topColorSensor, whiteRedThresholdTop, "TOP");
            boolean bottomDetectsWhite = isWhite(bottomColorSensor, whiteRedThresholdBottom, "BOTTOM");
            if (topDetectsWhite && bottomDetectsWhite) {
                // If both sensors detect white
                armDemands = ARM_DEMANDS.MOVE_UP;
            } else if (!topDetectsWhite && bottomDetectsWhite) {
                // If only the bottom sensor detects white
                armDemands = ARM_DEMANDS.HOLD;
            } else if (!(topDetectsWhite && bottomDetectsWhite)) {
                // Assuming this condition is meant to be if neither sensor detects white
                armDemands = ARM_DEMANDS.MOVE_DOWN;
            } else if (topDetectsWhite && !bottomDetectsWhite) {
                telemetry.addData("Scuffed Readings", "Top but not bottom...");
            }

            // Log the current state to telemetry for debugging
            telemetry.addData("Arm Demands", armDemands.toString());

            if (armDemands == ARM_DEMANDS.MOVE_UP) {
                pixelLock.reset();
                double target = arm.getCurrentPosition() - 5;

                if (target > armUpperLimit) {
                    target = armUpperLimit;
                } else if (target < armLowerLimit) {
                    target = armLowerLimit;
                }

                asyncMoveArm(target);
            } else if (armDemands == ARM_DEMANDS.MOVE_DOWN) {
                pixelLock.reset();
                double target = arm.getCurrentPosition() + 5;

                if (target > armUpperLimit) {
                    target = armUpperLimit;
                } else if (target < armLowerLimit) {
                    target = armLowerLimit;
                }
                asyncMoveArm(target);
            } else if (armDemands == ARM_DEMANDS.HOLD) {
                asyncMoveArm(arm.getCurrentPosition());
                moveRightFinger(CLAW_RIGHT_CLOSED);
            }

            callsToColor += 1;
        }

        telemetry.addData("Arm Position", arm.getCurrentPosition());

        telemetry.addData("Top Red", topRed);
        telemetry.addData("Top Green", topGreen);
        telemetry.addData("Top Blue", topBlue);

        telemetry.addLine("\n");

        telemetry.addData("Bottom Red", bottomRed);
        telemetry.addData("Bottom Green", bottomGreen);
        telemetry.addData("Bottom Blue", bottomBlue);

        telemetry.addData("Pixel Lock", pixelLock.milliseconds());
        telemetry.addData("Calls to Color", callsToColor);

        return pixelLock.milliseconds();
    }

    public boolean pixelLockVerification() {
        boolean bottomDetects = isWhite(bottomColorSensor, 1000, "Bottom");

        return bottomDetects;
    }

    public void syncMoveArm(double target) {
        double error = target - arm.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(error) > 10) {
            // calculate angles of joint & arm (in degrees) to account for torque
            double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
            double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
            double arm_angle = 270 - relative_arm_angle - joint_angle;

            double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

            error = target - arm.getCurrentPosition();

            double arm_out = armPID.calculate(arm.getCurrentPosition(), target);

            double arm_power = arm_ff + arm_out;

            arm.setPower(arm_power);

            opMode.telemetry.addData("Arm Position", arm.getCurrentPosition());
            //opMode.telemetry.addData("Arm Error", error);
            sleep(100);
        }
    }

    public void syncMoveJoint(double target) {
        double error = target -joint.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        while (opMode.opModeIsActive() && Math.abs(error) > 10) {
            error = target - joint.getCurrentPosition();

            double joint_out = jointPID.calculate(joint.getCurrentPosition(), target);

            double joint_power = joint_ff + joint_out;

            joint.setPower(joint_power);

            //opMode.telemetry.addData("Joint Error", error);
            sleep(100);
        }
    }

    public double asyncMoveJoint(double target) {
        double error = target -joint.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        double joint_out = jointPID.calculate(joint.getCurrentPosition(), target);

        double joint_power = joint_ff + joint_out;

        joint.setPower(joint_power);

        //opMode.telemetry.addData("Joint Error", error);

        return error;
    }

    public double asyncMoveArm(double target) {
        double error = target - arm.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        double arm_out = armPID.calculate(arm.getCurrentPosition(), target);

        double arm_power = arm_ff + arm_out;

        arm.setPower(arm_power);

        //opMode.telemetry.addData("Arm Error", error);

        return error;
    }

    public void pixelPickup(Integer pixelDepth) {
        currentState = AutoUtil.RobotState.PIXEL_STACK;
        moveRightFinger(CLAW_RIGHT_OPEN);
        if (pixelDepth == 1) {
            syncMoveArm(ARM_PIXEL_DEPTH_1);
        } else if (pixelDepth == 2) {
            syncMoveArm(ARM_PIXEL_DEPTH_2);
        } else if (pixelDepth == 3) {
            syncMoveArm(ARM_PIXEL_DEPTH_3);
        } else if (pixelDepth == 4) {
            syncMoveArm(ARM_PIXEL_DEPTH_4);
        } else if (pixelDepth == 5) {
            syncMoveArm(ARM_PIXEL_DEPTH_5);
        } else {
            syncMoveArm(ARM_PIXEL_DEPTH_1);
        }
        moveRightFinger(CLAW_RIGHT_CLOSED);
        currentState = AutoUtil.RobotState.IDLE;
    }

    public void pixelDropoff() {
        currentState = AutoUtil.RobotState.PIXEL_DROPOFF;
        syncMoveArm(ARM_FORWARDS_SCORE);
        sleep(500);
        moveLeftFinger(CLAW_LEFT_OPEN);
        moveRightFinger(CLAW_RIGHT_OPEN);
        sleep(500);
        syncMoveArm(ARM_FORWARDS_SCORE - 100);
        sleep(500);
        syncMoveArm(ARM_HOME);
        moveLeftFinger(CLAW_LEFT_CLOSED);
        moveRightFinger(CLAW_RIGHT_CLOSED);
        currentState = AutoUtil.RobotState.IDLE;
    }

    public void moveLeftFinger(double target) {
        ClawServoLeft.setPosition(target);
    }

    public void moveRightFinger(double target) {
        ClawServoRight.setPosition(target);
    }

    public void holdArm() {
        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        arm.setPower(arm_ff);
    }

    public void holdJoint() {
        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        joint.setPower(joint_ff);
    }

    public void killArm() {
        arm.setPower(0);
    }

    public void killJoint() {
        joint.setPower(0);
    }
}
