package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_GROUND;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name="Far Dropoff - Red")
public class FarPixelDropoffRed extends LinearOpMode {
    DcMotor arm;

    public Servo ClawServoLeft;
    public Servo ClawServoRight;

    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    public static double armPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        ClawServoRight.setDirection(Servo.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Bot starts on the far side of the red side of the field
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        // Push toward spike
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .forward(36)
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
                .splineTo(new Vector2d(-47, -34), Math.toRadians(180))
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
                .splineTo(new Vector2d(-26, -35), Math.toRadians(0))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(9)
                .build();

        // go to pixel stack
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .lineToSplineHeading(new Pose2d(-50, -34, Math.toRadians(180)))
                .build();

        // spline through edge truss to get to backdrop
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .splineToConstantHeading(new Vector2d(-18, -59.25), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(53, -37, Math.toRadians(0)), Math.toRadians(0))
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

        moveLeftFinger(CLAW_LEFT_OPEN);

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectory(center1);
            drive.followTrajectory(center2);
            moveRightFinger(CLAW_RIGHT_OPEN);
            moveArm(ARM_GROUND); // PICKUP
            moveRightFinger(CLAW_RIGHT_CLOSED);
            sleep(500);
            moveArm(ARM_HOME);
            drive.followTrajectory(center3);
            moveArm(ARM_FORWARDS_LOW_SCORE);
            sleep(500);
            moveRightFinger(CLAW_RIGHT_OPEN);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            moveArm(ARM_FORWARDS_LOW_SCORE - 100);
            sleep(100);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_OPEN);
            moveRightFinger(CLAW_RIGHT_CLOSED);
            drive.followTrajectory(cornerCenter);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            moveArm(ARM_FORWARDS_SCORE);
            sleep(500);
            moveLeftFinger(CLAW_LEFT_CLOSED);
            sleep(500);
            moveArm(ARM_FORWARDS_SCORE - 100);
            sleep(500);
            moveArm(ARM_HOME);
            moveLeftFinger(CLAW_LEFT_OPEN);
            drive.followTrajectory(cornerLeft);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectory(right1);
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

    private void moveRightFinger(double target) {
        ClawServoRight.setPosition(target);
    }

}