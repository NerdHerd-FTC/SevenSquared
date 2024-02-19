package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_norm_F;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Autonomous(name="NEW Dropoff - Blue")
public class NEWPixelDropoffBlue extends LinearOpMode {
    public DcMotor arm, joint;
    public Servo ClawServoLeft;

    public ElapsedTime runCycle = new ElapsedTime();

    public PIDController armPID = new PIDController(armP, armI, armD);

    BlueCubeDetectionPipeline blueCubeDetectionPipeline = new BlueCubeDetectionPipeline(telemetry);

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
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 270 degrees
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, 28), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(270))
                .splineTo(new Vector2d(49, 28), Math.toRadians(0))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(23, 48), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(49, 36, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(1, 30), Math.toRadians(180))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(20)
                .build();

        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .splineToSplineHeading(new Pose2d(49, 17), Math.toRadians(0))
                .build();

        Trajectory cornerCenter = drive.trajectoryBuilder(center.end())
                .strafeLeft(28)
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left1.end())
                .splineToConstantHeading(new Vector2d(50, 60), Math.toRadians(0))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right3.end())
                .lineToConstantHeading(new Vector2d(42, 5))
                .build();

        TrajectorySequence cornerRightRotate = drive.trajectorySequenceBuilder(cornerRight.end())
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence cornerLeftRotate = drive.trajectorySequenceBuilder(cornerLeft.end())
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence cornerCenterRotate = drive.trajectorySequenceBuilder(cornerCenter.end())
                .turn(Math.toRadians(180))
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

        while (opModeInInit()) {
            if (gamepad2.left_bumper) {
                moveLeftFinger(CLAW_LEFT_OPEN);
            } else if (gamepad2.right_bumper) {
                moveLeftFinger(CLAW_LEFT_CLOSED);
            }
        }

        waitForStart();

        moveLeftFinger(CLAW_LEFT_CLOSED);

        BlueCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        if (decision == BlueCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectory(center);
            moveArm(ARM_FORWARDS_LOW_SCORE);
            moveLeftFinger(CLAW_LEFT_OPEN);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 500 ) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
            }
            moveArm(ARM_FORWARDS_LOW_SCORE - 100);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 250) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE - 100);
            }
            moveArm(ARM_HOME);
            killArm();
            moveLeftFinger(CLAW_LEFT_CLOSED);
            drive.followTrajectory(cornerCenter);
            drive.followTrajectorySequence(cornerCenterRotate);
        } else if (decision == BlueCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectory(left1);
            moveArm(ARM_FORWARDS_LOW_SCORE);
            moveLeftFinger(CLAW_LEFT_OPEN);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 500 ) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
            }
            moveArm(ARM_FORWARDS_LOW_SCORE - 100);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 500 ) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE - 100);
            }
            moveArm(ARM_HOME);
            killArm();
            moveLeftFinger(CLAW_LEFT_CLOSED);
            drive.followTrajectory(cornerLeft);
            drive.followTrajectorySequence(cornerLeftRotate);
        } else if (decision == BlueCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(right3);
            moveArm(ARM_FORWARDS_LOW_SCORE);
            moveLeftFinger(CLAW_LEFT_OPEN);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 500 ) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
            }
            moveArm(ARM_FORWARDS_LOW_SCORE - 100);
            runCycle.reset();
            while (opModeIsActive() && runCycle.milliseconds() < 500 ) {
                asyncMoveArm(ARM_FORWARDS_LOW_SCORE - 100);
            }
            moveArm(ARM_HOME);
            killArm();
            moveLeftFinger(CLAW_LEFT_CLOSED);
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

    public double asyncMoveArm(double target) {
        double error = target - arm.getCurrentPosition();

        double joint_angle = 0 / joint_ticks_per_degree + 193;
        double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        double arm_out = armPID.calculate(arm.getCurrentPosition(), target);

        double arm_power = arm_ff + arm_out;

        arm.setPower(arm_power);

        return error;
    }

    public void killArm() {
        arm.setPower(0);
    }

    public void syncMoveJoint(double target) {
        PIDController jointPID = new PIDController(jointP, jointI, jointD);
        double error = target -joint.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        while (opModeIsActive() && Math.abs(error) > 10) {
            error = target - joint.getCurrentPosition();

            double joint_out = jointPID.calculate(joint.getCurrentPosition(), target);

            double joint_power = joint_ff + joint_out;

            joint.setPower(joint_power);

            //opMode.telemetry.addData("Joint Error", error);
            sleep(100);
        }
    }

    public double asyncMoveJoint(double target) {
        PIDController jointPID = new PIDController(jointP, jointI, jointD);
        double error = target - joint.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;

        double joint_out = jointPID.calculate(joint.getCurrentPosition(), target);

        double joint_power = joint_ff + joint_out;

        joint.setPower(joint_power);

        //opMode.telemetry.addData("Joint Error", error);

        return error;
    }


}