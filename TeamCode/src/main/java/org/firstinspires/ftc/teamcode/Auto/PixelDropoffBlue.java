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

@Autonomous(name="Old Dropoff - Blue")
public class PixelDropoffBlue extends LinearOpMode {
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

    boolean running = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, 34), Math.toRadians(90))
                .splineTo(new Vector2d(12, 52), Math.toRadians(90))
                .splineTo(new Vector2d(49, 36), Math.toRadians(0))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28, 35), Math.toRadians(180))
                .build();



        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(5.5, 34), Math.toRadians(175))
                .back(7)
                 // separate trajectory
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .splineToSplineHeading(new Pose2d(47, 36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory corner = drive.trajectoryBuilder(startPose)
                .strafeLeft(20)
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
        } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            drive.followTrajectory(corner);
        } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(corner);
        }
        drive.followTrajectory(corner);
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

}