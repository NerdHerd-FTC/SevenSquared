package org.firstinspires.ftc.teamcode.util;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

@Config
public class AutoUtil {
    public LinearOpMode opMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, joint;
    public Servo ClawServoLeft, ClawServoRight;
    public static double insane_joint = -2654;
    public static double insane_arm = 1273;

    private PIDController armPID = new PIDController(armP, armI, armD);
    private PIDController jointPID = new PIDController(jointP, jointI, jointD);

    public enum RobotState {
        IDLE,
        FOLLOWING_TRAJECTORY,
        PIXEL_STACK,
        PIXEL_DROPOFF,
        ERROR
    }

    public RobotState currentState = RobotState.IDLE;

    // Constructor
    public AutoUtil(LinearOpMode opMode, DcMotor arm, DcMotor joint, Servo ClawServoLeft, Servo ClawServoRight) {
        this.opMode = opMode;
        this.arm = arm;
        this.joint = joint;
        this.ClawServoLeft = ClawServoLeft;
        this.ClawServoRight = ClawServoRight;
    }

    public void motorTelemetry(DcMotor motor, String name) {
        opMode.telemetry.addLine("--- " + name + " ---");
        opMode.telemetry.addData(name + " Power", motor.getPower());
        opMode.telemetry.addData(name + " Position", motor.getCurrentPosition());
        opMode.telemetry.addData(name + " Target Position", motor.getTargetPosition());
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

            motorTelemetry(arm, "Arm");
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.addData("Power", arm_power);
            opMode.telemetry.update();
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

            motorTelemetry(joint, "Joint");
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.addData("Power", joint_power);
            opMode.telemetry.update();
            sleep(100);
        }
    }

    public double  asyncMoveJoint(double target) {
        double error = target -joint.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        error = target - joint.getCurrentPosition();

        double joint_out = jointPID.calculate(joint.getCurrentPosition(), target);

        double joint_power = joint_ff + joint_out;

        joint.setPower(joint_power);

        motorTelemetry(joint, "Joint");
        opMode.telemetry.addData("Error", error);
        opMode.telemetry.addData("Power", joint_power);
        opMode.telemetry.update();

        return error;
    }

    public double asyncMoveArm(double target) {
        double error = target - arm.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        error = target - arm.getCurrentPosition();

        double arm_out = armPID.calculate(arm.getCurrentPosition(), target);

        double arm_power = arm_ff + arm_out;

        arm.setPower(arm_power);

        motorTelemetry(arm, "Arm");
        opMode.telemetry.addData("Error", error);
        opMode.telemetry.addData("Power", arm_power);
        opMode.telemetry.update();

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
            opMode.telemetry.update();
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

}
