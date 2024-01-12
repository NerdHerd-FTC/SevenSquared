package org.firstinspires.ftc.teamcode.util;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

public class AutoUtil {
    public LinearOpMode opMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, joint;
    public Servo leftClaw, rightClaw;

    private PIDController armPID = new PIDController(armP, armI, armD);
    private PIDController jointPID = new PIDController(jointP, jointI, jointD);

    public enum RobotState {
        IDLE,
        MOVING_FORWARD,
        STRAFING_LEFT,
        STRAFING_RIGHT,
        TURNING,
        ARTICULATING_ARM,
        ARTICULATING_JOINT,
        ARTICULATING_CLAW,
        ERROR
    }

    public RobotState currentState = RobotState.IDLE;


    // Constructor
    public AutoUtil(LinearOpMode opMode, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, DcMotor arm, DcMotor joint, Servo leftClaw, Servo rightClaw) {
        this.opMode = opMode;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.arm = arm;
        this.joint = joint;
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
    }

    public void waitForDrive() {
        while (opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            motorTelemetry(frontLeft, "frontLeft");
            motorTelemetry(frontRight, "frontRight");
            motorTelemetry(backLeft, "backLeft");
            motorTelemetry(backRight, "backRight");
            opMode.idle();
        }
    }

    public void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveForward(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.MOVING_FORWARD;
            int move = (int) (inches * DRIVE_TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForDrive();

            stopDriveMotors();
            currentState = RobotState.IDLE;
        }
    }

    public void strafeLeft(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.STRAFING_LEFT;
            int move = (int) (inches * DRIVE_TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() - move);

            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForDrive();

            stopDriveMotors();
            currentState = RobotState.IDLE;
        }
    }

    public void strafeRight(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.STRAFING_RIGHT;
            int move = (int) (inches * DRIVE_TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForDrive();

            stopDriveMotors();
            currentState = RobotState.IDLE;
        }
    }

    // runs from -180 to 180
    public void turn(double targetAngle) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.TURNING;

            int turnTicks = (int) (targetAngle * DRIVE_TICKS_PER_DEGREE);

            // For a left turn, the left motors should move backward and the right motors forward
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - turnTicks);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + turnTicks);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - turnTicks);
            backRight.setTargetPosition(backRight.getCurrentPosition() + turnTicks);

            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the power for turning, this can be adjusted as necessary
            final double TURN_POWER = 0.5;
            frontLeft.setPower(-TURN_POWER);
            frontRight.setPower(TURN_POWER);
            backLeft.setPower(-TURN_POWER);
            backRight.setPower(TURN_POWER);

            waitForDrive();

            stopDriveMotors();
            currentState = RobotState.IDLE;
        }
    }

    public void setDriveMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void motorTelemetry(DcMotor motor, String name) {
        opMode.telemetry.addLine("--- " + name + " ---");
        opMode.telemetry.addData(name + " Power", motor.getPower());
        opMode.telemetry.addData(name + " Position", motor.getCurrentPosition());
        opMode.telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    public void moveArm(double target) {
        PIDController armPID = new PIDController(armP, armI, armD);
        double error = target - arm.getCurrentPosition();

        while (opMode.opModeIsActive() && Math.abs(error) > 10) {
            // calculate angles of joint & arm (in degrees) to account for torque
            double joint_angle = 193;
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
            sleep(5);
        }

    }

    public void moveLeftFinger(double target) {
        leftClaw.setPosition(target);
    }

}
