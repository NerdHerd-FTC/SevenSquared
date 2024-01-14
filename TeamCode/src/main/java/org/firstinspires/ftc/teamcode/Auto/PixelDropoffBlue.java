package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Old Dropoff - Blue")
public class PixelDropoffBlue extends LinearOpMode {
    public DcMotor arm;
    public Servo ClawServoLeft;

    BlueCubeDetectionPipeline blueCubeDetectionPipeline = new BlueCubeDetectionPipeline(telemetry);

    boolean running = false;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 270 degrees
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, 28), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(270))
                .splineTo(new Vector2d(57, 28), Math.toRadians(0))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28, 30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(23, 48), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(57, 36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(1, 30), Math.toRadians(180))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(20)
                .build();

        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .splineToSplineHeading(new Pose2d(58, 17), Math.toRadians(0))
                .build();

        Trajectory cornerCenter = drive.trajectoryBuilder(center.end())
                .strafeLeft(28)
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left1.end())
                .splineToConstantHeading(new Vector2d(50, 60), Math.toRadians(0))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right3.end())
                .splineToConstantHeading(new Vector2d(53, 60), Math.toRadians(0))
                .build();

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

        BlueCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectory(center);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_OPEN);
            drive.followTrajectory(cornerCenter);
        } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectory(left1);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_OPEN);
            drive.followTrajectory(cornerLeft);
        } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(right3);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_OPEN);
            drive.followTrajectory(cornerRight);
        }
    }

    public BlueCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return blueCubeDetectionPipeline.getDetection();
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    private void moveArm(double target) {
        PIDController armPID = new PIDController(armP, armI, armD);
        double error = target - arm.getCurrentPosition();

        while (opModeIsActive() && Math.abs(error) > 10) {
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
            telemetry.addData("Error", error);
            telemetry.addData("Power", arm_power);
            telemetry.update();
            sleep(100);
        }

    }

    private void moveLeftFinger(double target) {
        ClawServoLeft.setPosition(target);
    }


}