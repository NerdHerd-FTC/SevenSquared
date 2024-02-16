package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.util.AutoUtil;

@Config
@Autonomous(name="NEW Dropoff - Red")
public class NEWPixelDropoffRed extends LinearOpMode {
    DcMotor arm, joint;

    public Servo ClawServoLeft, ClawServoRight;

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

    public static double armPower = 0.0;

    boolean running = false;

    private PIDController armPID = new PIDController(armP, armI, armD);

    private ElapsedTime waitForClaw = new ElapsedTime();

    private enum centerState {
        CENTER_PUSH,
        ARM_TO_SCORE,
        RELEASE,
        ARM_TO_HOME,
        MOVE_TO_CORNER,
        DONE
    }

    private enum leftState {
        LEFT_PUSH,
        LEFT_BACKDROP,
        ARM_TO_SCORE,
        RELEASE,
        ARM_TO_HOME,
        MOVE_TO_CORNER,
        DONE
    }

    private enum rightState {
        RIGHT_PUSH,
        ARM_TO_SCORE,
        RELEASE,
        ARM_TO_HOME,
        MOVE_TO_CORNER,
        DONE
    }

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

        Vector2d centerEnd = new Vector2d(51, -31.5);
        Pose2d leftEnd = new Pose2d(51, -25, Math.toRadians(0));
        Pose2d rightEnd = new Pose2d(51, -42, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory center = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -29), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -52), Math.toRadians(90))
                .splineTo(centerEnd, Math.toRadians(0))
                .build();

        // Push to spike
        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(2, -30, Math.toRadians(180)), Math.toRadians(180)) // Move backward to (0, -30)
                .build();

        // Bring to backdrop
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
                .strafeRight(37)
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right1.end())
                .strafeRight(20)
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

        while (opModeInInit()) {
            if (gamepad2.left_bumper) {
                moveLeftFinger(CLAW_LEFT_OPEN);
            } else if (gamepad2.right_bumper) {
                moveLeftFinger(CLAW_LEFT_CLOSED);
            }
        }

        leftState leftCurrentState = leftState.LEFT_PUSH;
        rightState rightCurrentState = rightState.RIGHT_PUSH;
        centerState centerCurrentState = centerState.CENTER_PUSH;

        moveLeftFinger(CLAW_LEFT_CLOSED);

        RedCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        visionPortal.setProcessorEnabled(redCubeDetectionPipeline, false);

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectoryAsync(center);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectoryAsync(left1);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectoryAsync(right1);
        }

        // exits after completing center_push - debug
        while (opModeIsActive() && (leftCurrentState != leftState.DONE && rightCurrentState != rightState.DONE && centerCurrentState != centerState.DONE)) {
            telemetry.addData("Center Current State", centerCurrentState);
            telemetry.addData("Left Current State", leftCurrentState);
            telemetry.addData("Right Current State", rightCurrentState);

            if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
                switch (centerCurrentState) {
                    case CENTER_PUSH:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.ARM_TO_SCORE;
                        }
                        break;

                    case ARM_TO_SCORE:
                        double error = asyncMoveArm(ARM_FORWARDS_LOW_SCORE);

                        if (Math.abs(error) < 10) {
                            waitForClaw.reset();
                            moveLeftFinger(CLAW_LEFT_OPEN);
                            centerCurrentState = centerState.RELEASE;
                        }
                        break;

                    case RELEASE:
                        moveLeftFinger(CLAW_LEFT_OPEN);

                        if (waitForClaw.milliseconds() > 500) {
                            centerCurrentState = centerState.ARM_TO_HOME;
                        }
                        break;

                    case ARM_TO_HOME:
                        error = asyncMoveArm(ARM_HOME);

                        if (Math.abs(error) < 10) {
                            drive.followTrajectoryAsync(cornerCenter);
                            centerCurrentState = centerState.MOVE_TO_CORNER;
                            killArm();
                        }
                        break;

                    case MOVE_TO_CORNER:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.DONE;
                        }
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
                switch (leftCurrentState) {
                    case LEFT_PUSH:
                        drive.update();

                        if (!drive.isBusy()) {
                            drive.followTrajectoryAsync(left2);
                            leftCurrentState = leftState.LEFT_BACKDROP;
                        }
                        break;
                    case LEFT_BACKDROP:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.ARM_TO_SCORE;
                        }
                        break;
                    case ARM_TO_SCORE:
                        double error = asyncMoveArm(ARM_FORWARDS_LOW_SCORE);

                        if (Math.abs(error) < 10) {
                            waitForClaw.reset();
                            moveLeftFinger(CLAW_LEFT_OPEN);
                            leftCurrentState = leftState.RELEASE;
                        }
                        break;
                    case RELEASE:
                        moveLeftFinger(CLAW_LEFT_OPEN);
                        if (waitForClaw.milliseconds() > 500) {
                            leftCurrentState = leftState.ARM_TO_HOME;
                        }
                        break;
                    case ARM_TO_HOME:
                        error = asyncMoveArm(ARM_HOME);

                        if (Math.abs(error) < 10) {
                            drive.followTrajectoryAsync(cornerLeft);
                            leftCurrentState = leftState.MOVE_TO_CORNER;
                            killArm();
                        }
                        break;
                    case MOVE_TO_CORNER:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.DONE;
                        }
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
                switch (rightCurrentState) {
                    case RIGHT_PUSH:
                        drive.update();

                        if (!drive.isBusy()) {
                            rightCurrentState =  rightState.ARM_TO_SCORE;
                        }
                        break;
                    case ARM_TO_SCORE:
                        double error = asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
                        if (Math.abs(error) < 10) {
                            waitForClaw.reset();
                            moveLeftFinger(CLAW_LEFT_OPEN);
                            rightCurrentState = rightState.RELEASE;
                        }
                        break;
                    case RELEASE:
                        moveLeftFinger(CLAW_LEFT_OPEN);

                        if (waitForClaw.milliseconds() > 500) {
                            rightCurrentState = rightState.ARM_TO_HOME;
                        }
                        break;
                    case ARM_TO_HOME:
                        error = asyncMoveArm(ARM_HOME);

                        if (Math.abs(error) < 10) {
                            drive.followTrajectoryAsync(cornerRight);
                            rightCurrentState = rightState.MOVE_TO_CORNER;
                            killArm();
                        }
                        break;
                    case MOVE_TO_CORNER:
                        drive.update();

                        if (!drive.isBusy()) {
                            rightCurrentState = rightState.DONE;
                        }
                        break;
                }
            }
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

    public double asyncMoveArm(double target) {
        double error = target - arm.getCurrentPosition();

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = arm.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        double arm_out = armPID.calculate(arm.getCurrentPosition(), target);

        double arm_power = arm_ff + arm_out;

        arm.setPower(arm_power);

        //opMode.telemetry.addData("Arm Error", error);

        return error;
    }

    private void moveLeftFinger(double target) {
        ClawServoLeft.setPosition(target);
    }

    private void killArm() {
        arm.setPower(0);
    }
}