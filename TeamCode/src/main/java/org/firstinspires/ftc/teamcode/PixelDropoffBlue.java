package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Dropoff - Blue")
public class PixelDropoffBlue extends LinearOpMode {

    // Define motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constants

    // Pulled from "encoder resolution formula": https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    private static final double TICKS_PER_REV = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);

    // Pulled from strafer kit - converts mm. to in.
    private static final double WHEEL_DIAMETER_INCH = 96/25.4;
    private static final double TICKS_PER_INCH = (TICKS_PER_REV) / (WHEEL_DIAMETER_INCH * Math.PI);

    BlueCubeDetectionPipeline blueCubeDetectionPipeline = new BlueCubeDetectionPipeline(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors from hardware map
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

        waitForStart();

        while (opModeIsActive()) {
            BlueCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

            if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
                moveForward(48);
            } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
                moveForward(24);
                turn(-90, imu);
                strafeLeft(12);
            } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
                moveForward(24);
                turn(90, imu);
                strafeRight(12);
            }
        }
    }

    public BlueCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return blueCubeDetectionPipeline.getDetection();
    }

    // may need to make mods if there aren't enough encoder cables
    public void moveForward(double inches) {
        int move = (int)(inches * TICKS_PER_INCH);

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
    }

    public void strafeLeft(double inches) {
        int move = (int)(inches * TICKS_PER_INCH);

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
    }

    public void strafeRight(double inches) {
        int move = (int)(inches * TICKS_PER_INCH);

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
    }

    // runs from -180 to 180
    private void turn(double targetAngle, IMU imu) {
        final double kP = 0.01;

        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double angleDifference = targetAngle - currentAngle;

        // adjust threshold as needed
        while (angleDifference < 1) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            angleDifference = targetAngle - currentAngle;

            if (angleDifference > 180) {
                angleDifference -= 360;
            } else if (angleDifference < -180) {
                angleDifference += 360;
            }

            double turnPower = (angleDifference * kP);
            turnPower = Math.tanh(turnPower);

            frontLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backLeft.setPower(turnPower);
            backRight.setPower(-turnPower);
            sleep(50);
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
            // Optionally add telemetry updates here
            idle();
        }
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}