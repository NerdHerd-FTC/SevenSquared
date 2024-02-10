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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AutoUtil {
    public LinearOpMode opMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, joint;
    public Servo ClawServoLeft, ClawServoRight;
    public ColorSensor topColorSensor, bottomColorSensor;
    public static double insane_joint = -2654;
    public static double insane_arm = 1273;
    public Telemetry telemetry;
    public static boolean debug = true;

    public static int whiteThreshold = 1000;

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
    public AutoUtil(LinearOpMode opMode, DcMotor arm, DcMotor joint, Servo ClawServoLeft, Servo ClawServoRight, Telemetry telemetry) {
        this.opMode = opMode;
        this.arm = arm;
        this.joint = joint;
        this.ClawServoLeft = ClawServoLeft;
        this.ClawServoRight = ClawServoRight;
        this.telemetry = telemetry;
    }

    private boolean isWhite(ColorSensor sensor, int threshold) {
        // get rgba and normalize
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        telemetry.addData("Color Sensor", sensor.toString());
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        return red > threshold && green > threshold && blue > threshold &&
                Math.abs(red - green) < threshold * 0.2 &&
                Math.abs(green - blue) < threshold * 0.2 &&
                Math.abs(blue - red) < threshold * 0.2;
    }

    public void updateStateBasedOnColor() {
        // Check if top and bottom sensors detect white
        boolean topDetectsWhite = topColorSensor.alpha() > whiteThreshold;
        boolean bottomDetectsWhite = bottomColorSensor.alpha() > whiteThreshold;

        if (topDetectsWhite && bottomDetectsWhite) {
            // If both sensors detect white
            armDemands = ARM_DEMANDS.MOVE_UP;
        } else if (!topDetectsWhite && bottomDetectsWhite) {
            // If only the bottom sensor detects white
            armDemands = ARM_DEMANDS.HOLD;
        } else if (topDetectsWhite) {
            // Assuming this condition is meant to be if only the top sensor detects white
            armDemands = ARM_DEMANDS.MOVE_DOWN;
        } else {
            telemetry.addData("Scuffed", "Scuffed");
            // If none of the conditions above are met, you can either set it to IDLE or keep the current state
            // currentState = RobotState.IDLE;
        }

        // Log the current state to telemetry for debugging
        telemetry.addData("Current State", currentState.toString());
    }

    public double lockOntoPixel() {
        updateStateBasedOnColor();
        if (armDemands == ARM_DEMANDS.MOVE_UP) {
            asyncMoveArm(arm.getCurrentPosition() - 2);
            pixelLock.reset();
        } else if (armDemands == ARM_DEMANDS.MOVE_DOWN) {
            asyncMoveArm(arm.getCurrentPosition() + 2);
            pixelLock.reset();
        } else if (armDemands == ARM_DEMANDS.HOLD) {
            asyncMoveArm(arm.getCurrentPosition());
        }

        telemetry.addData("Pixel Lock", pixelLock.milliseconds());
        return pixelLock.milliseconds();
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

            opMode.telemetry.addData("Arm Error", error);
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

            opMode.telemetry.addData("Joint Error", error);
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

        opMode.telemetry.addData("Joint Error", error);

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

        opMode.telemetry.addData("Arm Error", error);

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

    public void insanePixelPickup() {
        currentState = RobotState.PIXEL_STACK;
        moveRightFinger(CLAW_RIGHT_OPEN);
        while (opMode.opModeIsActive() && asyncMoveArm(insane_arm) < 10 && asyncMoveJoint(insane_joint) < 10) {
            opMode.telemetry.addData("Arm Error", asyncMoveArm(insane_arm));
            opMode.telemetry.addData("Joint Error", asyncMoveJoint(insane_joint));
        }

        moveRightFinger(CLAW_RIGHT_CLOSED);
        currentState = RobotState.IDLE;
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
