package org.firstinspires.ftc.teamcode.Archive;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.BlueCubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;


@Config
@Autonomous(name="DR Dropoff - Blue")
@Disabled
public class PixelDropoffBlueDR extends LinearOpMode {
    public static int CENTER_Y = 20, CENTER_ROT = 180, CENTER_X = 18;
    public static int LEFT_Y = 17, LEFT_X = 3;
    public static int RIGHT_Y = 10, RIGHT_X = 4;

    public static int CORNER_OFFSET = 5;

    // Define motors
    private DcMotor frontLeft, frontRight, backLeft, backRight, arm;
    private Servo ClawServoLeft;

    // Constants
    //11 or 7.5
    private static final double ROBOT_RADIUS_INCHES = 8; // Half the distance between left and right wheels
    private static final double DEGREES_TO_INCHES = Math.PI * 2 * ROBOT_RADIUS_INCHES / 360;

    // Pulled from "encoder resolution formula": https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double TICKS_PER_REV = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    // Pulled from strafer kit - converts mm. to in.
    private static final double WHEEL_DIAMETER_INCH = 96/25.4;
    private static final double TICKS_PER_INCH = (TICKS_PER_REV) / (WHEEL_DIAMETER_INCH * Math.PI);

    private static final double TICKS_PER_DEGREE = TICKS_PER_INCH * DEGREES_TO_INCHES;
    BlueCubeDetectionPipeline blueCubeDetectionPipeline = new BlueCubeDetectionPipeline(telemetry);

    private PIDController armPID = new PIDController(armP, armI, armD);

    boolean running = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors from hardware map
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams;

        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        // Technically this is the default, however specifying it is clearer
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(imuParams);

        // VisionPortal
        VisionPortal visionPortal;

        // Create a new VisionPortal Builder object.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "leftCamera"))
                .addProcessor(blueCubeDetectionPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        setClawServoLeft(ClawServoLeft, CLAW_LEFT_CLOSED);

            BlueCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

            if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
                moveForward(33);
                moveForward(-30);
                strafeLeft(30);
                moveForward(CENTER_Y);
                turn(CENTER_ROT);
                moveForward(CENTER_X);
            } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
                moveForward(24);
                turn(180);
                moveForward(9);
                moveForward(-9);
                strafeLeft(26);
                moveForward(30);
                strafeRight(LEFT_Y);
                moveForward(LEFT_X);
            } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
                moveForward(24);
                sleep(1500);
                turn(-180);
                moveForward(7);
                moveForward(-7);
                turn(360);
                moveForward(30);
                strafeRight(RIGHT_Y);
                moveForward(RIGHT_X);

        }
            stopMotors();
            setArmPower(ARM_FORWARDS_SCORE);
            stopArticulation();
            setClawServoLeft(ClawServoLeft, CLAW_LEFT_OPEN);
            sleep(50);
            setArmPower(0);
            stopArticulation();
            setClawServoLeft(ClawServoLeft, CLAW_LEFT_CLOSED);
            moveForward(-5);
            if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
                strafeLeft(CENTER_Y + CORNER_OFFSET);
            } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
                strafeLeft(LEFT_Y + CORNER_OFFSET);
            } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
                strafeLeft(24+RIGHT_Y +CORNER_OFFSET);
            }
    }

    public BlueCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return blueCubeDetectionPipeline.getDetection();
    }

    public void moveForward(double inches) {
        if (!running) {
            running = true;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            running = false;
        }
    }

    public void strafeLeft(double inches) {
        if (!running) {
            running = true;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() - move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            running = false;
        }
    }

    public void strafeRight(double inches) {
        if (!running) {
            running = true;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            running = false;
        }
    }

    // runs from -180 to 180
    private void turn(double targetAngle) {
        if (!running) {
            running = true;

            int turnTicks = (int) (targetAngle * TICKS_PER_DEGREE);

            // For a left turn, the left motors should move backward and the right motors forward
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - turnTicks);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + turnTicks);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - turnTicks);
            backRight.setTargetPosition(backRight.getCurrentPosition() + turnTicks);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the power for turning, this can be adjusted as necessary
            final double TURN_POWER = 0.3;
            frontLeft.setPower(-TURN_POWER);
            frontRight.setPower(TURN_POWER);
            backLeft.setPower(-TURN_POWER);
            backRight.setPower(TURN_POWER);

            waitForMotors();

            stopMotors();
            running = false;
        }
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void waitForMotors() {
        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            motorTelemetry(frontLeft, "frontLeft");
            motorTelemetry(frontRight, "frontRight");
            motorTelemetry(backLeft, "backLeft");
            motorTelemetry(backRight, "backRight");
            idle();
        }
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    public void setArmPower(double target) {
        double power = 0;
        double mult = 0.7;

        running = true;

        while (running && opModeIsActive()) {
            // calculate angles of joint & arm (in degrees) to account for torque
            double joint_angle = 0 / joint_ticks_per_degree + 193;
            double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 29;
            double arm_angle = 270 - relative_arm_angle - joint_angle;

            double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

            double arm_out = armPID.calculate(arm.getCurrentPosition(), target);
            power = arm_out + arm_ff;

            double error = target - arm.getCurrentPosition();

            // deadband
            if (Math.abs(power) < 0.05) {
                power = 0;
            } else if (power > 1.0) {
                power = 1.0;
            } else if (power < -1.0) {
                power = -1.0;
            }

            if (Math.abs(error) < 40 ) {
                power = 0;
                arm.setPower(0);
                running = false;
            }

            telemetry.addData("Error", error);
            telemetry.addData("Running", running);

            arm.setPower(power);
            telemetry.update();
        }
    }

    private void stopArticulation() {
        arm.setPower(0);
    }

    private void setClawServoLeft(Servo ClawServoLeft, double position) {
        ClawServoLeft.setPosition(position);
    }

}