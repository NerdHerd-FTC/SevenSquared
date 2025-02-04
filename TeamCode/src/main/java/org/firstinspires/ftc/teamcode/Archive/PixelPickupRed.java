package org.firstinspires.ftc.teamcode.Archive;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.RedCubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous(name="PixelPickup - Red")
public class PixelPickupRed extends LinearOpMode {
    DcMotor arm;

    public Servo ClawServoLeft;

    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    public static double armPower = 0.0;

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

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        // probably should be ~61 but keep this for consistency with other paths
        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));

        Vector2d centerEnd = new Vector2d(55, -31.5);
        Pose2d leftEnd = new Pose2d(59, -25, Math.toRadians(0));
        Pose2d rightEnd = new Pose2d(59, -42, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -29), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -52), Math.toRadians(90))
                .splineTo(centerEnd, Math.toRadians(0))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(2, -30, Math.toRadians(180)), Math.toRadians(180)) // Move backward to (0, -30)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(28, -30, Math.toRadians(180))) // Intermediate to allow for turning
                .splineToSplineHeading(leftEnd, Math.toRadians(0)) // Move backward to (49, -36)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(22, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(22, -60), Math.toRadians(90))
                .splineToSplineHeading(rightEnd, Math.toRadians(0))
                .build();

        Trajectory cornerCenter = drive.trajectoryBuilder(center.end())
                .strafeRight(28)
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left2.end())
                // new Pose2d(57, -30, Math.toRadians(0));
                .splineToConstantHeading(new Vector2d(50, -60), Math.toRadians(0))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right1.end())
                .splineToConstantHeading(new Vector2d(50, -60), Math.toRadians(0))
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

        moveLeftFinger(CLAW_LEFT_CLOSED);

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectory(center);
            moveArm(ARM_FORWARDS_LOW_SCORE);
            moveLeftFinger(CLAW_LEFT_OPEN);
            sleep(500);
            moveArm(ARM_FORWARDS_LOW_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            drive.followTrajectory(cornerCenter);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_OPEN);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            drive.followTrajectory(cornerLeft);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectory(right1);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_OPEN);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_CLOSED);
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
