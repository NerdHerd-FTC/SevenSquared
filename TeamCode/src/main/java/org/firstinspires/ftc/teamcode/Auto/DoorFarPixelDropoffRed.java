package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_DROP_2;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_FORWARDS_LOW_SCORE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_HOME;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.ARM_PIXEL_DEPTH_1;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.JOINT_AVOID_CUBE;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.JOINT_HOME;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Door Far Dropoff - Red")
public class DoorFarPixelDropoffRed extends LinearOpMode {
    DcMotor arm, joint;

    public static double x_end = -47.5;
    public static double y_end = -28;

    public static double jointError = 0;
    public static double armError = 0;

    public Servo ClawServoLeft;
    public Servo ClawServoRight;

    private ElapsedTime waitForClaw = new ElapsedTime();

    private ElapsedTime waitForArm = new ElapsedTime();

    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    private AprilTagProcessor aprilTag;

    static final double CAMERA_X_OFFSET = 0.0; // replace with your camera's x offset
    static final double CAMERA_Y_OFFSET = 0.0; // replace with your camera's y offset

    public static int calibration_step = 0;

    // States for the state machine
    // Center
    private enum centerState {
        CENTER1,
        CENTER2,
        CENTER2_1,
        CENTER2_2,
        CENTER3,
        PICK_UP,
        CALIBRATE_PICKUP,
        GRAB,
        HOME,
        CENTER4,
        CENTER5,
        CENTER6,
        CENTER7,
        FIRST_DROP_OFF,
        RELEASE,
        HOME2,
        MOVE_TO_CORNER,
        ROTATE,
        DONE
    }

    // Left
    private enum leftState {
        LEFT1,
        LEFT1_2,
        LEFT2,
        LEFT2_2,
        PICK_UP,
        CALIBRATE_PICKUP,
        GRAB,
        HOME,
        LEFT3,
        LEFT4,
        FIRST_DROP_OFF,
        RELEASE,
        HOME2,
        MOVE_TO_CORNER,
        ROTATE,
        DONE
    }

    // Right
    private enum rightState {
        RIGHT1,
        RIGHT2,
        RIGHT2_2,
        RIGHT2_3,
        PICK_UP,
        CALIBRATE_PICKUP,
        GRAB,
        HOME,
        RIGHT4,
        RIGHT5,
        RIGHT6,
        FIRST_DROP_OFF,
        RELEASE,
        HOME2,
        MOVE_TO_CORNER,
        ROTATE,
        DONE
    }

    private enum armIssue {
        stuckOnPixel,
        stuckOnWall,
        noDetection
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // how to add functionality to determine if the movement has been completed successfully for the async functions? additionally, how to implement a better state machine design
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

        // Bot starts on the far side of the red side of the field
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        // Push toward spike
        Trajectory center1 = drive.trajectoryBuilder(startPose)
                .forward(42.5)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(33)
                .build();

        TrajectorySequence center2_turn = drive.trajectorySequenceBuilder(center2.end())
                .turn(Math.toRadians(90))
                .build();

        Trajectory center2_strafeToX = drive.trajectoryBuilder(center2_turn.end())
                .lineToConstantHeading(new Vector2d(-34, -31))
                .build();

        // Spline to pixel stack
        Trajectory center3 = drive.trajectoryBuilder(center2_strafeToX.end())
                .lineToConstantHeading(new Vector2d(x_end, y_end))
                .build();

        // Raise arm up before executing 3_ series of movements - one issue may be time for later autos
        // Calibration numbers need to be tuned
        Trajectory center3_1 = drive.trajectoryBuilder(center3.end())
                .forward(-1)
                .build();

        Trajectory center3_2 = drive.trajectoryBuilder(center3_1.end())
                .strafeLeft(1)
                .build();

        Trajectory center3_3 = drive.trajectoryBuilder(center3_2.end())
                .forward(1)
                .build();

        // moving to backdrop
        Trajectory center4 = drive.trajectoryBuilder(center3.end())
                .lineToConstantHeading(new Vector2d(-55, -29.95))
                .build();

        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .strafeRight(36)
                .build();

        Trajectory center6 = drive.trajectoryBuilder(center5.end())
                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                .build();

        Trajectory center7 = drive.trajectoryBuilder(center6.end())
                .splineToConstantHeading(new Vector2d(67, -30.5), Math.toRadians(0))
                .build();

        // move to left corner
        Trajectory cornerCenter = drive.trajectoryBuilder(center7.end())
                .strafeTo(new Vector2d(64, 0))
                .build();

        TrajectorySequence rotateCenter = drive.trajectorySequenceBuilder(cornerCenter.end())
                .turn(Math.toRadians(180)) // Turns 45 degrees counter-clockwise
                .build();

        // push to spike
        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(180)), Math.toRadians(180))
                .build();

        // moving to pixel stack
        Trajectory left1_2 = drive.trajectoryBuilder(left1.end())
                .back(11)
                .build();

        // moving to pixel stack
        Trajectory left2 = drive.trajectoryBuilder(left1_2.end())
                .lineToConstantHeading(new Vector2d(-34, -5.95))
                .build();

        Trajectory left2_2 = drive.trajectoryBuilder(left2.end())
                .lineToConstantHeading(new Vector2d(-55, -5.95))
                .build();

        Trajectory left3 = drive.trajectoryBuilder(left2_2.end())
                .strafeRight(12)
                .build();

        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(67, -17), Math.toRadians(0))
                .build();

        Trajectory cornerLeft = drive.trajectoryBuilder(left4.end())
                // new Pose2d(57, -30, Math.toRadians(0));
                .strafeTo(new Vector2d(64, 0))
                .build();

        TrajectorySequence rotateLeft = drive.trajectorySequenceBuilder(cornerLeft.end())
                .turn(Math.toRadians(180))
                .build();

        // RIGHT goes through the edge truss to drop off at backdrop
        Trajectory right1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-20.261, -30, Math.toRadians(45)), Math.toRadians(45))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .strafeTo(new Vector2d(-38, -48))
                .build();

        TrajectorySequence right2_2 = drive.trajectorySequenceBuilder(right2.end())
                .turn(Math.toRadians(135))
                .build();

        Trajectory right2_3 = drive.trajectoryBuilder(right2_2.end())
                .lineToConstantHeading(new Vector2d(-38, -29.95))
                .build();

        // PIKCUP FROM STACK
        Trajectory right3 = drive.trajectoryBuilder(right2_3.end())
                .lineToConstantHeading(new Vector2d(-55, -29.95))
                .build();

        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .strafeRight(36)
                .build();

        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .lineToSplineHeading(new Pose2d(30, 6, Math.toRadians(0)))
                .build();

        Trajectory right6 = drive.trajectoryBuilder(right5.end())
                .splineToConstantHeading(new Vector2d(67, -40), Math.toRadians(0)) // TUNE THIS ENDPOINT
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right6.end())
                .strafeTo(new Vector2d(64, 0))
                .build();

        TrajectorySequence rotateRight = drive.trajectorySequenceBuilder(cornerRight.end())
                .turn(Math.toRadians(180))
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251f)
                .build();

        // VisionPortal
        VisionPortal visionPortal;

        // Create a new VisionPortal Builder object.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "leftCamera"))
                .addProcessor(redCubeDetectionPipeline)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);

        while (opModeInInit()) {
            if (gamepad2.left_bumper) {
                autoUtil.moveLeftFinger(CLAW_LEFT_OPEN);
            } else if (gamepad2.right_bumper) {
                autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);
            }
        }

        leftState leftCurrentState = leftState.LEFT1;
        rightState rightCurrentState = rightState.RIGHT1;
        centerState centerCurrentState = centerState.CENTER1;

        RedCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);

        visionPortal.setProcessorEnabled(redCubeDetectionPipeline, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectoryAsync(center1);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectoryAsync(left1);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectoryAsync(right1);
        }

        while (opModeIsActive() && (leftCurrentState != leftState.DONE && rightCurrentState != rightState.DONE && centerCurrentState != centerState.DONE)) {
            // aprilTagRelocalization(drive, CAMERA_Y_OFFSET, CAMERA_X_OFFSET);
            // telemetryAprilTag();
            telemetry.addData("Center Current State", centerCurrentState);
            telemetry.addData("Left Current State", leftCurrentState);
            telemetry.addData("Right Current State", rightCurrentState);
            telemetry.addData("Clock", waitForClaw.milliseconds());

            if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
                switch (centerCurrentState) {
                    case CENTER1:
                        // Move forward to spike mark
                        drive.update();

                        autoUtil.asyncMoveJoint(JOINT_AVOID_CUBE);

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.CENTER2;
                            drive.followTrajectoryAsync(center2);
                        }
                        break;
                    case CENTER2:
                        // Move backwards
                        drive.update();
                        autoUtil.asyncMoveJoint(JOINT_AVOID_CUBE);

                        if (!drive.isBusy()) {
                            autoUtil.asyncMoveJoint(0);
                            centerCurrentState = centerState.CENTER2_1;
                            drive.followTrajectorySequenceAsync(center2_turn);
                        }
                        break;
                    case CENTER2_1:
                        drive.update();

                        autoUtil.asyncMoveJoint(0);

                        if (!drive.isBusy()) {
                            autoUtil.killJoint();
                            centerCurrentState = centerState.CENTER2_2;
                            drive.followTrajectoryAsync(center2_strafeToX);
                        }
                        break;
                    case CENTER2_2:
                        drive.update();
                        armError = autoUtil.asyncMoveArm(ARM_PIXEL_DEPTH_1);

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.CENTER3;
                            drive.followTrajectoryAsync(center3);
                        }
                        break;
                    case CENTER3:
                        // Move to the pixel stack
                        drive.update();

                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        armError = autoUtil.asyncMoveArm(ARM_PIXEL_DEPTH_1);

                        if (!drive.isBusy()) {
                            autoUtil.pixelLock.reset();
                            centerCurrentState = centerState.PICK_UP;
                        }
                        break;
                    case PICK_UP:
                        // Move the arm to the pixel stack height
                        double timeLock = autoUtil.lockOntoPixel();

                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        // If the arm is at the pixel stack height, move to the next state
                        if (timeLock > 750) {
                            autoUtil.moveRightFinger(CLAW_RIGHT_CLOSED);

                            centerCurrentState = centerState.GRAB;

                            waitForClaw.reset();
                        } else if (autoUtil.armDemands == AutoUtil.ARM_DEMANDS.STUCK) {
                            if (drive.isBusy()) {
                                drive.update();
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.MOVE_BACK) {
                                drive.followTrajectoryAsync(center3_1);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.MOVE_FORWARD) {
                                drive.followTrajectoryAsync(center3_2);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.STRAFE_RIGHT) {
                                drive.followTrajectoryAsync(center3_3);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            }
                        }
                        break;
                    case CALIBRATE_PICKUP:
                        drive.update();

                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        armError = autoUtil.asyncMoveArm(1000);

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.PICK_UP;
                            armError = autoUtil.asyncMoveArm(ARM_PIXEL_DEPTH_1);
                        }

                        break;
                    case GRAB:
                        autoUtil.moveRightFinger(CLAW_RIGHT_CLOSED);

                        // Hold the position of the arm and joint and close the claw
                        autoUtil.holdArm();

                        // If the claw has been closed for 1000 milliseconds, move to the next state
                        if (waitForClaw.milliseconds() > 1000) {
                            waitForClaw.reset();

                            if (!autoUtil.pixelLockVerification()) {
                                autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);
                                autoUtil.pixelLock.reset();
                                centerCurrentState = centerState.PICK_UP;
                            } else {
                                centerCurrentState = centerState.HOME;
                            }
                        }
                        break;
                    case HOME:
                        // Move the arm and joint to the home position
                        armError = autoUtil.asyncMoveArm(ARM_HOME);

                        // If the arm and joint are at the home position, move to the next state
                        if (Math.abs(armError) <= 10) {
                            centerCurrentState = centerState.CENTER4;
                            drive.followTrajectory(center4);
                            autoUtil.killArm();
                        }
                        break;
                    case CENTER4:
                        // Move to the second pixel stack
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.CENTER5;
                            drive.followTrajectory(center5);
                        }
                        break;
                    case CENTER5:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.CENTER6;
                            drive.followTrajectory(center6);
                        }
                        break;

                    case CENTER6:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.CENTER7;
                            drive.followTrajectory(center7);
                        }
                        break;
                    case CENTER7:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.FIRST_DROP_OFF;
                        }
                        break;
                    case FIRST_DROP_OFF:
                        double error = autoUtil.asyncMoveArm(ARM_FORWARDS_LOW_SCORE);

                        if (Math.abs(error) < 10) {
                            if (waitForArm.milliseconds() > 1000) {
                                waitForClaw.reset();
                                autoUtil.moveLeftFinger(CLAW_LEFT_OPEN);
                                centerCurrentState = centerState.RELEASE;
                            }
                        } else {
                            waitForArm.reset();
                        }
                        break;
                    case RELEASE:
                        autoUtil.asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
                        autoUtil.moveLeftFinger(CLAW_LEFT_OPEN);
                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        if (waitForClaw.milliseconds() > 1000) {
                            centerCurrentState = centerState.HOME2;
                        }
                        break;
                    case HOME2:
                        error = autoUtil.asyncMoveArm(ARM_HOME);

                        if (Math.abs(error) < 10) {
                            drive.followTrajectoryAsync(cornerCenter);
                            centerCurrentState = centerState.MOVE_TO_CORNER;
                            autoUtil.killArm();
                        }
                        break;
                    case MOVE_TO_CORNER:
                        drive.update();

                        if (!drive.isBusy()) {
                            centerCurrentState = centerState.ROTATE;
                            drive.followTrajectorySequence(rotateCenter);
                        }
                        break;
                    case ROTATE:
                        drive.update();
                        armError = autoUtil.asyncMoveArm(ARM_HOME);
                        jointError = autoUtil.asyncMoveJoint(JOINT_HOME);

                        if (!drive.isBusy()) {
                            autoUtil.killArm();
                            autoUtil.killJoint();
                            centerCurrentState = centerState.DONE;
                        }
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
                switch (leftCurrentState) {
                    case LEFT1:
                        drive.update();

                        autoUtil.asyncMoveJoint(JOINT_AVOID_CUBE);

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.LEFT1_2;
                            drive.followTrajectoryAsync(left1_2);
                        }
                        break;

                    case LEFT1_2:
                        drive.update();
                        autoUtil.asyncMoveJoint(JOINT_AVOID_CUBE);

                        if (!drive.isBusy()) {
                            autoUtil.asyncMoveJoint(0);
                            leftCurrentState = leftState.LEFT2;
                            drive.followTrajectoryAsync(left2);
                        }
                        break;

                    case LEFT2:
                        drive.update();
                        autoUtil.asyncMoveJoint(0);

                        if (!drive.isBusy()) {
                            autoUtil.killJoint();
                            leftCurrentState = leftState.LEFT2_2;
                            drive.followTrajectoryAsync(left2_2);
                        }
                        break;

                    case LEFT2_2:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.PICK_UP;
                        }
                        break;

                    case PICK_UP:
                        // Move the arm to the pixel stack height
                        double timeLock = autoUtil.lockOntoPixel();

                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        // If the arm is at the pixel stack height, move to the next state
                        if (timeLock > 750) {
                            autoUtil.moveRightFinger(CLAW_RIGHT_CLOSED);

                            leftCurrentState = leftState.GRAB;

                            waitForClaw.reset();
                        } else if (autoUtil.armDemands == AutoUtil.ARM_DEMANDS.STUCK) {
                            if (drive.isBusy()) {
                                drive.update();
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.MOVE_BACK) {
                                drive.followTrajectoryAsync(center3_1);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.MOVE_FORWARD) {
                                drive.followTrajectoryAsync(center3_2);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            } else if (autoUtil.pixelDemands == AutoUtil.PIXEL_DEMANDS.STRAFE_RIGHT) {
                                drive.followTrajectoryAsync(center3_3);
                                autoUtil.pixelDemands = AutoUtil.PIXEL_DEMANDS.IDLE;
                            }
                        }
                        break;
                    case GRAB:
                        autoUtil.moveRightFinger(CLAW_RIGHT_CLOSED);

                        // Hold the position of the arm and joint and close the claw
                        autoUtil.holdArm();

                        // If the claw has been closed for 1000 milliseconds, move to the next state
                        if (waitForClaw.milliseconds() > 1000) {
                            waitForClaw.reset();

                            if (!autoUtil.pixelLockVerification()) {
                                autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);
                                autoUtil.pixelLock.reset();
                                leftCurrentState = leftState.PICK_UP;
                            } else {
                                leftCurrentState = leftState.HOME;
                            }
                        }
                        break;
                    case HOME:
                        // Move the arm and joint to the home position
                        armError = autoUtil.asyncMoveArm(ARM_HOME);

                        // If the arm and joint are at the home position, move to the next state
                        if (Math.abs(armError) <= 10) {
                            leftCurrentState = leftState.LEFT3;
                            drive.followTrajectory(left3);
                            autoUtil.killArm();
                        }
                        break;
                    case LEFT3:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.LEFT4;
                            drive.followTrajectory(left4);
                        }
                        break;
                    case LEFT4:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.FIRST_DROP_OFF;
                        }
                        break;
                    case FIRST_DROP_OFF:
                        double error = autoUtil.asyncMoveArm(ARM_FORWARDS_LOW_SCORE);

                        if (Math.abs(error) < 10) {
                            if (waitForArm.milliseconds() > 1000) {
                                waitForClaw.reset();
                                autoUtil.moveLeftFinger(CLAW_LEFT_OPEN);
                                leftCurrentState = leftState.RELEASE;
                            }
                        } else {
                            waitForArm.reset();
                        }
                        break;
                    case RELEASE:
                        autoUtil.asyncMoveArm(ARM_FORWARDS_LOW_SCORE);
                        autoUtil.moveLeftFinger(CLAW_LEFT_OPEN);
                        autoUtil.moveRightFinger(CLAW_RIGHT_OPEN);

                        if (waitForClaw.milliseconds() > 1000) {
                            leftCurrentState = leftState.HOME2;
                        }
                        break;
                    case HOME2:
                        error = autoUtil.asyncMoveArm(ARM_HOME);

                        if (Math.abs(error) < 10) {
                            drive.followTrajectoryAsync(cornerLeft);
                            leftCurrentState = leftState.MOVE_TO_CORNER;
                            autoUtil.killArm();
                        }
                        break;
                    case MOVE_TO_CORNER:
                        drive.update();

                        if (!drive.isBusy()) {
                            leftCurrentState = leftState.ROTATE;
                            drive.followTrajectorySequence(rotateLeft);
                        }
                        break;
                    case ROTATE:
                        drive.update();
                        armError = autoUtil.asyncMoveArm(ARM_HOME);
                        jointError = autoUtil.asyncMoveJoint(JOINT_HOME);

                        if (!drive.isBusy()) {
                            autoUtil.killArm();
                            autoUtil.killJoint();
                            leftCurrentState = leftState.DONE;
                        }
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
                switch (rightCurrentState) {

                }
            }

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.update();
        }
    }
    private void aprilTagRelocalization(SampleMecanumDrive drive, double camera_y_offset, double camera_x_offset) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Grab first AprilTag to use as pose relocalizer
        if (!currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);

            Pose2d currentPose = new Pose2d(detection.ftcPose.x - camera_y_offset, detection.ftcPose.y - camera_x_offset, detection.ftcPose.z);

            drive.setPoseEstimate(currentPose);
        }
    }

    public RedCubeDetectionPipeline.Detection getDecisionFromEOCV() {
        return redCubeDetectionPipeline.getDetection();
    }

    private void motorTelemetry (DcMotor motor, String name){
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    /**
     * Update the robot's pose based on the detected AprilTags.
     * @param drive The SampleMecanumDrive object for pose updates.
     */
    private void updateRobotPoseFromAprilTags (SampleMecanumDrive drive){
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            // Assuming the first detection is the most reliable one
            AprilTagDetection detection = detections.get(0);

            // Convert the AprilTag pose to robot pose considering the camera offsets
            Pose2d robotPose = new Pose2d(
                    detection.ftcPose.x - CAMERA_X_OFFSET,
                    detection.ftcPose.y - CAMERA_Y_OFFSET,
                    Math.toRadians(detection.ftcPose.yaw)
            );

            // Update Roadrunner's pose estimate
            drive.setPoseEstimate(robotPose);

            // Telemetry for debugging
            telemetry.addData("AprilTag ID", detection.id);
            telemetry.addData("Robot Pose", robotPose);
            telemetry.update();
        }
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag () {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void continueFollowing(SampleMecanumDrive drive) {
        drive.update();
    }
}