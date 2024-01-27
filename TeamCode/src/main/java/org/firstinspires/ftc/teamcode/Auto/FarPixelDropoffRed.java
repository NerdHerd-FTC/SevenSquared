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
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Far Dropoff - Red")
public class FarPixelDropoffRed extends LinearOpMode {
    DcMotor arm, joint;

    public static double insane_joint = -2654;
    public static double insane_arm = 1273;

    public Servo ClawServoLeft;
    public Servo ClawServoRight;

    RedCubeDetectionPipeline redCubeDetectionPipeline = new RedCubeDetectionPipeline(telemetry);

    private AprilTagProcessor aprilTag;

    static final double CAMERA_X_OFFSET = 0.0; // replace with your camera's x offset
    static final double CAMERA_Y_OFFSET = 0.0; // replace with your camera's y offset

    // States for the state machine
    // Center
    private enum centerState {
        CENTER1,
        CENTER2,
        CENTER3,
        PICK_UP,
        CENTER4,
        FIRST_DROP_OFF,
        CENTER5,
        SECOND_DROP_OFF,
        MOVE_TO_CORNER,
        FOLLOWING,
        DONE
    }

    // Left
    private enum leftState {
        LEFT1,
        LEFT2,
        LEFT3,
        PICK_UP,
        LEFT4,
        FIRST_DROP_OFF,
        LEFT5,
        SECOND_DROP_OFF,
        MOVE_TO_CORNER,
        DONE
    }

    // Right
    private enum rightState {
        RIGHT1,
        RIGHT2,
        RIGHT3,
        PICK_UP,
        RIGHT4,
        FIRST_DROP_OFF,
        RIGHT5,
        SECOND_DROP_OFF,
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
                .forward(43)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(33)
                .build();

        // Move to pixel stack
        Trajectory center3 = drive.trajectoryBuilder(center2.end())
                .splineToLinearHeading(new Pose2d(-37, -30, Math.toRadians(0)), Math.toRadians(90))
                .build();

        // Spline under the edge truss to drop off at backdrop
        Trajectory center4 = drive.trajectoryBuilder(center3.end())
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
                .forward(26)
                .splineTo(new Vector2d(-20, -26.5), Math.toRadians(0))
                .build();


        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                .back(15)
                .build();

        // go to pixel stack
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .lineToSplineHeading(new Pose2d(-48.5, -32, Math.toRadians(180)))
                .build();

        // spline through middle truss to get to backdrop
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .strafeRight(6)
                .splineTo(new Vector2d(41, -10), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(74, -41, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory cornerRight = drive.trajectoryBuilder(right4.end())
                .splineToConstantHeading(new Vector2d(65, -10), Math.toRadians(0))
                .build();

        /*
        <Calibration
                size="640 480"
        focalLength="622.001f, 622.001f"
        principalPoint="319.803f, 241.251f"
        distortionCoefficients="0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0"
                />

         */
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

        waitForStart();

        leftState leftCurrentState = leftState.LEFT1;
        rightState rightCurrentState = rightState.RIGHT1;
        centerState centerCurrentState = centerState.CENTER1;

        RedCubeDetectionPipeline.Detection decision = getDecisionFromEOCV();

        if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
            drive.followTrajectoryAsync(center1);
        } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
            drive.followTrajectoryAsync(left1);
        } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
            drive.followTrajectoryAsync(right1);
        }

        autoUtil.moveLeftFinger(CLAW_LEFT_CLOSED);

        visionPortal.setProcessorEnabled(redCubeDetectionPipeline, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        while (opModeIsActive()) {
            aprilTagRelocalization(drive, CAMERA_Y_OFFSET, CAMERA_X_OFFSET);
            telemetryAprilTag();
            telemetry.addData("Center Current State", centerCurrentState);
            telemetry.addData("Left Current State", leftCurrentState);
            telemetry.addData("Right Current State", rightCurrentState);

            if (decision == RedCubeDetectionPipeline.Detection.CENTER) {
                switch (centerCurrentState) {
                    case CENTER1:
                        drive.update();

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            centerCurrentState = centerState.CENTER2;
                            drive.followTrajectoryAsync(center2);
                        }
                        break;
                    case CENTER2:
                        drive.update();

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            centerCurrentState = centerState.CENTER3;
                            drive.followTrajectoryAsync(center3);
                        }
                        break;
                    case CENTER3:
                        drive.update();

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            centerCurrentState = centerState.PICK_UP;
                        }
                        break;
                    case PICK_UP:
                        // synchronous
                        autoUtil.insanePixelPickup();
                        centerCurrentState = centerState.CENTER4;
                        drive.followTrajectory(center4);
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.LEFT) {
                switch (leftCurrentState) {
                    case LEFT1:
                        drive.followTrajectoryAsync(left1);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.LEFT2;
                        }
                        break;
                    case LEFT2:
                        drive.followTrajectoryAsync(left2);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.LEFT3;
                        }
                        break;
                    case LEFT3:
                        drive.followTrajectoryAsync(left3);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.PICK_UP;
                        }
                        break;
                    case PICK_UP:
                        autoUtil.pixelPickup(1);
                        leftCurrentState = leftState.LEFT4;
                        break;
                    case LEFT4:
                        drive.followTrajectoryAsync(left4);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.FIRST_DROP_OFF;
                        }
                        break;
                    case FIRST_DROP_OFF:
                        autoUtil.pixelDropoff();
                        leftCurrentState = leftState.LEFT5;
                        break;
                    case LEFT5:
                        drive.followTrajectoryAsync(cornerLeft);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.SECOND_DROP_OFF;
                        }
                        break;
                    case SECOND_DROP_OFF:
                        autoUtil.pixelDropoff();
                        leftCurrentState = leftState.MOVE_TO_CORNER;
                        break;
                    case MOVE_TO_CORNER:
                        drive.followTrajectoryAsync(cornerLeft);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            leftCurrentState = leftState.DONE;
                        }
                        break;
                }
            } else if (decision == RedCubeDetectionPipeline.Detection.RIGHT) {
                switch (rightCurrentState) {
                    case RIGHT1:
                        drive.followTrajectoryAsync(right1);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.RIGHT2;
                        }
                        break;
                    case RIGHT2:
                        drive.followTrajectoryAsync(right2);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.RIGHT3;
                        }
                        break;
                    case RIGHT3:
                        drive.followTrajectoryAsync(right3);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.PICK_UP;
                        }
                        break;
                    case PICK_UP:
                        autoUtil.pixelPickup(1);
                        rightCurrentState = rightState.RIGHT4;
                        break;
                    case RIGHT4:
                        drive.followTrajectoryAsync(right4);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.FIRST_DROP_OFF;
                        }
                        break;
                    case FIRST_DROP_OFF:
                        autoUtil.pixelDropoff();
                        rightCurrentState = rightState.RIGHT5;
                        break;
                    case RIGHT5:
                        drive.followTrajectoryAsync(cornerRight);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.SECOND_DROP_OFF;
                        }
                        break;
                    case SECOND_DROP_OFF:
                        autoUtil.pixelDropoff();
                        rightCurrentState = rightState.MOVE_TO_CORNER;
                        break;
                    case MOVE_TO_CORNER:
                        drive.followTrajectoryAsync(cornerRight);

                        if (drive.isBusy()) {
                            autoUtil.asyncMoveArm(ARM_HOME);
                        } else {
                            rightCurrentState = rightState.DONE;
                        }
                        break;
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