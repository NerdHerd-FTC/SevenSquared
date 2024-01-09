package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Old Dropoff - Red")
public class PixelDropoffRed extends LinearOpMode {

    // Define motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

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
    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    boolean running = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors from hardware map
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-12, -60, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .forward(33)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .forward(-30)
                .build();

        Trajectory center3 = drive.trajectoryBuilder(center2.end())
                .strafeRight(40)
                .build();

        // VisionPortal
        VisionPortal visionPortal;

        // Create a new VisionPortal Builder object.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "leftCamera"))
                .addProcessor(redCubeDetectionPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

            RedCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

            if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
                drive.followTrajectory(center1);
                drive.followTrajectory(center2);
                drive.followTrajectory(center3);
            } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
                moveForward(27);
                turn(180);
                moveForward(8);
                moveForward(-49);
            } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
                moveForward(27);
                turn(-180);
                moveForward(8);
                moveForward(-8);
                strafeRight(26);
                moveForward(40);

        }
    }

    public RedCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return redCubeDetectionPipeline.getDetection();
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
            final double TURN_POWER = 0.5;
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

}