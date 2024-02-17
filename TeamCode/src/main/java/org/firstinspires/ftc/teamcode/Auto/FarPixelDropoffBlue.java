package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_GROUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AutoUtil;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="Far Dropoff - Blue")
public class FarPixelDropoffBlue extends LinearOpMode {
    DcMotor arm, joint;

    public Servo ClawServoLeft;
    public Servo ClawServoRight;

    BlueCubeDetectionPipeline blueCubeDetectionPipeline = new BlueCubeDetectionPipeline(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        joint = hardwareMap.get(DcMotor.class, "joint");

        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint.setDirection(DcMotor.Direction.REVERSE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        ClawServoRight.setDirection(Servo.Direction.FORWARD);

        ColorSensor topColor = hardwareMap.get(ColorSensor.class, "topColor");
        ColorSensor bottomColor = hardwareMap.get(ColorSensor.class, "bottomColor");

        AutoUtil autoUtil = new AutoUtil(this, arm, joint, ClawServoLeft, ClawServoRight, topColor, bottomColor, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Bot starts on the far side of the blue side of the field
        Pose2d startPose = new Pose2d(-34, 62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // Push toward spike
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36, 28), Math.toRadians(270))
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(20)
                .build();

        // Go to pixel pickup
        Trajectory center3 = drive.trajectoryBuilder(center2.end())
                .strafeRight(5)
                .splineToSplineHeading(new Pose2d(-60, 10, Math.toRadians(180)), Math.toRadians(180))
                .build();

        // move towards backdrop
        Trajectory center4 = drive.trajectoryBuilder(center3.end())
                .back(90)
                .build();

        // arrive at backdrop
        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .lineToSplineHeading(new Pose2d(57, 33, Math.toRadians(0)))
                .build();

        // move to left corner
        Trajectory cornerCenter = drive.trajectoryBuilder(center5.end())
                .splineToConstantHeading(new Vector2d(50, 10), Math.toRadians(0))
                .build();

        // push to spike
        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .forward(20)
                .splineTo(new Vector2d(-47, 30), Math.toRadians(180))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(15)
                .build();

        // move to pixel stack
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .strafeLeft(15)
                .splineToLinearHeading(new Pose2d(-60, 10, Math.toRadians(180)), Math.toRadians(180))
                .build();

        // move towards backdrop
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .back(90)
                .build();

        // arrive at backdrop
        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .lineToSplineHeading(new Pose2d(58, 17, Math.toRadians(0)))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right5.end())
                // new Pose2d(57, -30, Math.toRadians(0));
                .splineToConstantHeading(new Vector2d(50, 10), Math.toRadians(0))
                .build();

        // push to spike
        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .forward(20)
                .splineTo(new Vector2d(-20, 30), Math.toRadians(0))
                .build();

        // moving to pixel stack
        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .back(15)
                .build();

        // arrive at pixel stack
        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .strafeLeft(15)
                .splineToLinearHeading(new Pose2d(-60, 10, Math.toRadians(180)), Math.toRadians(180))
                .build();

        // move towards backdrop
        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .back(90)
                .build();

        // arrive at backdrop
        Trajectory left5 = drive.trajectoryBuilder(left4.end())
                .lineToSplineHeading(new Pose2d(57, 36, Math.toRadians(0)))
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left5.end())
                .splineToConstantHeading(new Vector2d(50, 10), Math.toRadians(0))
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

        autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);

        if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectory(center1);
            drive.followTrajectory(center2);
            drive.followTrajectory(center3);

            autoUtil.pixelPickup(1);

            drive.followTrajectory(center4);
            while(opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }
            drive.followTrajectory(center5);

            autoUtil.pixelDropoff();

            drive.followTrajectory(cornerCenter);
        } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            drive.followTrajectory(left3);

            autoUtil.pixelPickup(1);

            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(left4);
            while(opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }
            drive.followTrajectory(left5);

            autoUtil.pixelDropoff();

            drive.followTrajectory(cornerLeft);
        } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(right3);

            autoUtil.pixelPickup(1);

            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(right4);
            while(opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }
            drive.followTrajectory(right5);

            autoUtil.pixelDropoff();

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

}