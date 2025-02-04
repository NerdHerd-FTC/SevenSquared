package org.firstinspires.ftc.teamcode.Archive;
import android.util.Size;
import java.util.List;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.BlueCubeDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous(name="Chase April Tag")
public class ChaseAprilTag extends LinearOpMode {
    // Define motors
    private DcMotor frontLeft, frontRight, backLeft, backRight, joint, arm;
    private Servo ClawServoLeft;
    private AprilTagDetection lastDetectedTag = null;

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
    int tagID = 1;

    AprilTagProcessor aprilTag;

    private int frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget, jointTarget, armTarget;

    // Create the vision portal by using a builder.
    VisionPortal.Builder builder = new VisionPortal.Builder();

    public enum RobotState {
        IDLE,
        MOVING_FORWARD,
        STRAFING_LEFT,
        STRAFING_RIGHT,
        TURNING,
        ARTICULATING_ARM,
        ARTICULATING_JOINT,
        ARTICULATING_CLAW,
        ERROR
    }
    private RobotState currentState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors from hardware map
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        joint = hardwareMap.dcMotor.get("joint");
        arm = hardwareMap.dcMotor.get("arm");

        joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint.setDirection(DcMotor.Direction.REVERSE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams;

        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        // Technically this is the default, however specifying it is clearer
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(imuParams);

        // VisionPortal
        VisionPortal visionPortal;

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create a new VisionPortal Builder object.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "leftCamera"))
                .addProcessor(aprilTag)
                .addProcessor(blueCubeDetectionPipeline)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        visionPortal.setProcessorEnabled(aprilTag, true);

        advanceByAprilTag(aprilTag, 0.5, 8);
        stopMotors();
    }

    public void moveForward(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.MOVING_FORWARD;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    public void strafeLeft(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.STRAFING_LEFT;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() - move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    public void strafeRight(double inches) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.STRAFING_RIGHT;
            int move = (int) (inches * TICKS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);

            waitForMotors();

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    // runs from -180 to 180
    private void turn(double targetAngle) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.TURNING;

            int turnTicks = (int) (targetAngle * TICKS_PER_DEGREE);

            // For a left turn, the left motors should move backward and the right motors forward
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - turnTicks);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + turnTicks);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() - turnTicks);
            backRight.setTargetPosition(backRight.getCurrentPosition() + turnTicks);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the power for turning, this can be adjusted as necessary
            final double TURN_POWER = 0.5;
            frontLeft.setPower(-TURN_POWER);
            frontRight.setPower(TURN_POWER);
            backLeft.setPower(-TURN_POWER);
            backRight.setPower(TURN_POWER);

            waitForMotors();

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void waitForMotors() {
        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Current State", currentState);
            motorTelemetry(frontLeft, "frontLeft");
            motorTelemetry(frontRight, "frontRight");
            motorTelemetry(backLeft, "backLeft");
            motorTelemetry(backRight, "backRight");

            aprilTagTelemetry(tagID);
            telemetry.update();
            idle();
        }
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            telemetry.addData(name + " Target Position", motor.getTargetPosition());
            telemetry.addData(name + "Error", motor.getTargetPosition() - motor.getCurrentPosition());
        }
        telemetry.update();
    }

    private void strafeByAprilTag(AprilTagProcessor aprilTag, int tagID, double power, double offset) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.STRAFING_RIGHT;

            double range = 0.0;
            if (aprilTag.getDetections().size() == 0 && lastDetectedTag.id == tagID) {
                // we lost sight of the tag, but we know where it is
                AprilTagDetection tag = lastDetectedTag;
                range = tag.ftcPose.range - offset;
            } else if (aprilTag.getDetections().size() != 0) {
                for (AprilTagDetection tag : aprilTag.getDetections()) {
                    if (tag.id == tagID) {
                        lastDetectedTag = tag;
                    }
                }

                if (lastDetectedTag.id != tagID) {
                    // we can't see the tag
                    return;
                }

                // inches
                range = lastDetectedTag.ftcPose.range - offset;
            }  else {
                // wait until we can see the tag
                while (opModeIsActive() && aprilTag.getDetections().size() == 0) {
                    telemetry.addLine("Tag not found....");
                }
            }

            while (opModeIsActive() && Math.abs(range) > 0.5) {
                int move = (int) (range * TICKS_PER_INCH);

                frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
                frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
                backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
                backRight.setTargetPosition(backRight.getCurrentPosition() + move);

                setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);

                waitForMotors();

                // refresh
                AprilTagDetection tag = getTagData(aprilTag, tagID);
                range = tag.ftcPose.range;
            }

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    private void advanceByAprilTag(AprilTagProcessor aprilTag, double power, double offset) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.MOVING_FORWARD;

            if (aprilTag.getDetections().size() > 0) {
                double range = 0.0;
                int tags_found = aprilTag.getDetections().size();
                for (AprilTagDetection tag : aprilTag.getDetections()) {
                    if (tag.id == tagID) {
                        range += tag.ftcPose.range - offset;
                    }
                }
                double average_range = range / tags_found;

                moveForward(average_range);
            } else {
                // wait until we can see the tag
                // clock
                ElapsedTime timeout = new ElapsedTime();

                while (opModeIsActive() && aprilTag.getDetections().size() == 0 && timeout.seconds() < 10) {
                    telemetry.addLine("Tag not found....");
                    moveForward(1);
                    moveForward(-1);
                }

                if(aprilTag.getDetections().size() > 0) {
                    double range = 0.0;
                    int tags_found = aprilTag.getDetections().size();
                    for (AprilTagDetection tag : aprilTag.getDetections()) {
                        if (tag.id == tagID) {
                            range += tag.ftcPose.range - offset;
                        }
                    }
                    double average_range = range / tags_found;

                    moveForward(average_range);
                } else {
                    moveForward(32);
                }
            }

            stopMotors();
            currentState = RobotState.IDLE;
        }
    }

    private AprilTagDetection getTagData(AprilTagProcessor aprilTag, double tagID) {
        if (aprilTag.getDetections().size() == 0) {
            // do something here - add later
            return null;
        }

        // get tag data
        for (AprilTagDetection tag : aprilTag.getDetections()) {
            if (tag.id == tagID) {
                return tag;
            }
        }
        return null;
    }

    private void aprilTagTelemetry(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                // Add "key" information to telemetry
                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");
            }
        }
    }

    // ARM AND JOINT MOTOR METHODS
    private void runJoint(DcMotor jointMotor, double targetPosition, double power) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.ARTICULATING_JOINT;
            jointMotor.setTargetPosition((int) targetPosition);
            jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            jointMotor.setPower(power);

            waitForArticulation();

            jointMotor.setPower(0);
            currentState = RobotState.IDLE;
        }
    }

    private void runArm(DcMotor armMotor, double targetPosition, double power) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.ARTICULATING_ARM;
            armMotor.setTargetPosition((int) targetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(power);

            waitForArticulation();

            armMotor.setPower(0);
            currentState = RobotState.IDLE;
        }
    }

    private void stopArticulation() {
        joint.setPower(0);
        arm.setPower(0);
    }

    private void waitForArticulation() {
        while (opModeIsActive() && (joint.isBusy() || arm.isBusy())) {
            telemetry.addData("Joint Busy", joint.isBusy());
            telemetry.addData("Arm Busy", arm.isBusy());
            telemetry.update();

            idle();
        }
    }

    private void setClawServoLeft(Servo ClawServoLeft, double position) {
        if (currentState == RobotState.IDLE) {
            currentState = RobotState.ARTICULATING_CLAW;
            ClawServoLeft.setPosition(position);
            currentState = RobotState.IDLE;
        }
    }
}