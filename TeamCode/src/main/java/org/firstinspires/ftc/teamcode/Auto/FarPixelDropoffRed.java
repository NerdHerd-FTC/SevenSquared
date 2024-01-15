package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_GROUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name="Far Dropoff - Red")
public class FarPixelDropoffRed extends LinearOpMode {
    DcMotor arm, joint;

    public Servo ClawServoLeft;
    public Servo ClawServoRight;

    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
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

        AutoUtil autoUtil = new AutoUtil(this, arm, joint, ClawServoLeft, ClawServoRight);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Bot starts on the far side of the red side of the field
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        // Push toward spike
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .forward(34)
                .back(19)
                .build();

        // Move to pixel stack
        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .splineToLinearHeading(new Pose2d(-53, -36, Math.toRadians(180)), Math.toRadians(180))
                .build();

        // Spline under the edge truss to drop off at backdrop
        Trajectory center3 = drive.trajectoryBuilder(center2.end())
                .splineToConstantHeading(new Vector2d(-18, -59.25), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(55, -31.5, Math.toRadians(0)), Math.toRadians(0))
                .build();

        // move to left corner
        Trajectory cornerCenter = drive.trajectoryBuilder(center3.end())
                .strafeLeft(28)
                .build();

        // push to spike
        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-46, -30), Math.toRadians(180))
                .build();

        // moving to pixel stack
        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .back(14)
                .build();

        // moving to pixel stack
        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .strafeRight(15)
                .splineToLinearHeading(new Pose2d(-55, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        // spline through the middle truss to drop off at backdrop
        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .lineToSplineHeading(new Pose2d(0, -10, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(59, -25), Math.toRadians(0))
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left4.end())
                // new Pose2d(57, -30, Math.toRadians(0));
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(0))
                .build();

        // RIGHT goes through the edge truss to drop off at backdrop
        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .forward(11)
                .splineTo(new Vector2d(-26, -38), Math.toRadians(0))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(13)
                .build();

        // go to pixel stack
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .lineToSplineHeading(new Pose2d(-57, -34, Math.toRadians(180)))
                .build();

        // spline through middle truss to get to backdrop
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .strafeRight(6)
                .splineTo(new Vector2d(34, -10), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(59, -42, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right4.end())
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(0))
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

        waitForStart();

        RedCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(center1);
            drive.followTrajectory(center2);

            autoUtil.pixelPickup(1);

            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(center3);
            while (opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }

            autoUtil.pixelDropoff();

            drive.followTrajectory(cornerCenter);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            drive.followTrajectory(left3);

            autoUtil.pixelPickup(1);

            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(left4);
            while (opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }

            autoUtil.pixelDropoff();

            drive.followTrajectory(cornerLeft);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(right3);

            autoUtil.pixelPickup(1);

            autoUtil.currentState = AutoUtil.RobotState.FOLLOWING_TRAJECTORY;
            drive.followTrajectory(right4);
            while (opModeIsActive() && drive.isBusy()) {
                autoUtil.asyncMoveArm(ARM_HOME);
            }

            autoUtil.pixelDropoff();

            drive.followTrajectory(cornerRight);
        }
    }

    public RedCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return redCubeDetectionPipeline.getDetection();
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }
}