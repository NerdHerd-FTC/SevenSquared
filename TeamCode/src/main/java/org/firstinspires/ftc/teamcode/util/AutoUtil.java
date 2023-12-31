package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;

public class AutoUtil {
    public LinearOpMode opMode;
    public DcMotor frontLeft, frontRight, backLeft, backRight, arm, joint;

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
    public AutoUtil(LinearOpMode opMode, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, DcMotor arm, DcMotor joint) {
        this.opMode = opMode;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.arm = arm;
        this.joint = joint;
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

}
