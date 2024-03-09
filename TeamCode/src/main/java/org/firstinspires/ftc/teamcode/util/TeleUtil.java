package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointP;

import android.graphics.Color;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
public class TeleUtil {
    public LinearOpMode opMode;
    public DcMotor motorFL, motorFR, motorBL, motorBR, arm, joint;
    public Servo ClawServoRight, ClawServoLeft;
    public CRServo DroneServo, DroneCover;
    public ColorSensor leftBottomColor, rightBottomColor;
    private ElapsedTime DroneActive = new ElapsedTime();
    public static boolean autoPickup = true;
    SampleMecanumDrive drive;

    // Color Thresholds - yellow, green, purple


    private enum DroneCoverState {
        closing,
        open,
        opening,
        closed
    }

    private DroneCoverState droneState = DroneCoverState.closed;

    public static double joint_error = 0;
    public static double arm_error = 0;

    public static boolean debug = true;

    private boolean fl_closed = true;
    private boolean fr_closed = true;
    private boolean turnSlow = false;
    private boolean moveSlow = false;
    private TouchSensor touchSensor;
    private boolean touchSensorPressed = false;
    private double touchSensorValue;
    private ElapsedTime CSR = new ElapsedTime();
    private ElapsedTime CSL = new ElapsedTime();
    private ElapsedTime FSC = new ElapsedTime();
    private ElapsedTime ARM_TIME_TO_HOLD = new ElapsedTime();
    private ElapsedTime dpadDebounce = new ElapsedTime();
    private ElapsedTime leftFingerLastActive = new ElapsedTime();
    private ElapsedTime rightFingerLastActive = new ElapsedTime();

    private double driveSlowMult = 0.25;
    private int targetPixelLayer = 1;

    private PIDController armPID = new PIDController(armP, armI, armD);
    private PIDController jointPID = new PIDController(jointP, jointI, jointD);

    public static double joint_hold = 0;
    public static double arm_hold = 0;

    private double lastArmPosition = 0;

    public static double armSetPoint = 0;
    public static double jointSetPoint = 0;

    // drive slow
    // pid arm - kill power
    //

    private enum ArmState {
        DRIVER_CONTROL(null),
        HOMING(ARM_HOME),
        GROUNDING(ARM_GROUND),
        FORWARDS_SCORING(ARM_FORWARDS_SCORE - 1),
        FORWARDS_REACHED(ARM_FORWARDS_SCORE),
        PLANE_LAUNCHING(ARM_AIRPLANE),
        WAITING_TO_HOLD(null),
        HOLDING(null);

        private Integer target;

        ArmState(Integer target) {
            this.target = target;
        }

        public Integer getTarget() {
            return target;
        }
    }

    private enum JointState {
        DRIVER_CONTROL(null),
        GROUNDING(0),
        BACKWARDS_SCORING(JOINT_BACKWARDS_SCORE - 1),
        BACKWARDS_REACHED(JOINT_BACKWARDS_SCORE),
        PLANE_LAUNCHING(JOINT_AIRPLANE),
        FORWARDS_SCORING(null),
        HOLDING(null);

        private Integer target;

        JointState(Integer target) {
            this.target = target;
        }

        public int getTarget() {
            return target;
        }
    }

    private enum leftFingerAuto {
        IDLE,
        AWAITING_SIGNAL,
        CLOSING
    }

    private enum rightFingerAuto {
        IDLE,
        AWAITING_SIGNAL,
        CLOSING
    }

    private leftFingerAuto leftFingerAutoState = leftFingerAuto.IDLE;
    private rightFingerAuto rightFingerAutoState = rightFingerAuto.IDLE;

    // Pixel Lock Variables
    private static ElapsedTime pixelLockLoops = new ElapsedTime();
    public static int pixelLockRedThreshold = 1000;
    public static int pixelLockGreenThreshold = 1000;
    public static int pixelLockBlueThreshold = 1000;

    public static int pixelLockDetectionRed = 185;


    private ArmState armState = ArmState.DRIVER_CONTROL;
    private JointState jointState = JointState.DRIVER_CONTROL;

    private double driveMult = 1.0;

    // Constructor
    public TeleUtil(LinearOpMode opMode, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, DcMotor arm, DcMotor joint, Servo ClawServoLeft, Servo ClawServoRight, CRServo DroneServo, CRServo DroneCover, ColorSensor bottomLeft, ColorSensor bottomRight) {
        this.opMode = opMode;
        this.motorFL = frontLeft;
        this.motorFR = frontRight;
        this.motorBL = backLeft;
        this.motorBR = backRight;
        this.arm = arm;
        this.joint = joint;
        this.ClawServoRight = ClawServoRight;
        this.ClawServoLeft = ClawServoLeft;
        this.DroneServo = DroneServo;
        this.DroneCover = DroneCover;
        this.drive = new SampleMecanumDrive(opMode.hardwareMap);
        this.leftBottomColor = bottomLeft;
        this.rightBottomColor = bottomRight;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    /*public void fieldOrientedDrive(SampleMecanumDrive drive, Gamepad gamepad, boolean exponential_drive, boolean slowdown) {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading


        // Toggle turn slow
        if (gamepad.right_stick_button && gamepad.right_trigger > 0.5) {
            turnSlow = !turnSlow;
        }

        // Exponential Drive
        double exponent = 2.0;

        double y_raw = -gamepad.left_stick_y;
        double x_raw = -gamepad.left_stick_x;
        double rx = -gamepad.right_stick_x;

        Vector2d input = new Vector2d(Math.signum(y_raw) * Math.pow(Math.abs(y_raw), exponent),
                Math.signum(x_raw) * Math.pow(Math.abs(x_raw), exponent))
                .rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        double rot_ex = 5.0;

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        Math.signum(rx) * Math.pow(Math.abs(rx), rot_ex)
                )
        );

        opMode.telemetry.addData("Heading", poseEstimate.getHeading() * 180 / Math.PI);
    }
     */

    public void robotOrientedDrive(Gamepad gamepad, boolean exponential_drive, boolean slowdown, boolean turnSlow) {
        double y_raw = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x_raw = gamepad.left_stick_x;
        double rx_raw = gamepad.right_stick_x;
        // notes on robot oriented drive controls : b= right, y = forward, x= left, a= back
        // = gamepad.b;
        // = gamepad.y;
        // = gamepad.x;
        // = gamepad.a;

        // Toggle move slow
        if (gamepad.a && FSC.milliseconds() > 500) {
            moveSlow = !moveSlow;
            FSC.reset();
        }

        // Deadband to address controller drift
        double deadband = DEAD_BAND;
        if (Math.abs(y_raw) < deadband) {
            y_raw = 0;
        }
        if (Math.abs(x_raw) < deadband) {
            x_raw = 0;
        }
        if (Math.abs(rx_raw) < deadband) {
            rx_raw = 0;
        }

        // Exponential Drive
        double exponent = 2.0;
        double y = exponential_drive ? Math.signum(y_raw) * Math.pow(Math.abs(y_raw), exponent) : y_raw;
        double x = exponential_drive ? Math.signum(x_raw) * Math.pow(Math.abs(x_raw), exponent) : x_raw;
        double rx = exponential_drive ? Math.signum(rx_raw) * Math.pow(Math.abs(rx_raw), exponent) : rx_raw;

        if (moveSlow) {
            y *= 0.35;
            x *= 0.35;
            rx *= 0.25;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFL.setPower(frontLeftPower);
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);

        if (debug) {
            motorTelemetry(motorFL, "FL");
            motorTelemetry(motorBL, "BL");
            motorTelemetry(motorFR, "FR");
            motorTelemetry(motorBR, "BR");
        }
    }

    public void dpadSupportedRobotDrive(Gamepad gamepad, boolean exponential_drive, boolean slowdown, boolean turnSlow, Pose2d pose) {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        double botHeading = poseEstimate.getHeading();

        double y_raw = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x_raw = gamepad.left_stick_x;
        double rx_raw = -gamepad.right_stick_x;

        // Toggle move slow
        if (gamepad.a && FSC.milliseconds() > 500) {
            moveSlow = !moveSlow;
            FSC.reset();
        }

        // Deadband to address controller drift
        double deadband = DEAD_BAND;
        if (Math.abs(y_raw) < deadband) {
            y_raw = 0;
        }
        if (Math.abs(x_raw) < deadband) {
            x_raw = 0;
        }
        if (Math.abs(rx_raw) < deadband) {
            rx_raw = 0;
        }

        // Exponential Drive
        double exponent = 2.0;
        double y = exponential_drive ? Math.signum(y_raw) * Math.pow(Math.abs(y_raw), exponent) : y_raw;
        double x = exponential_drive ? Math.signum(x_raw) * Math.pow(Math.abs(x_raw), exponent) : x_raw;
        double rx = exponential_drive ? Math.signum(rx_raw) * Math.pow(Math.abs(rx_raw), exponent) : rx_raw;

        // Strafe with dpad
        if (gamepad.dpad_right) {
            // Rotate the movement direction counter to the bot's rotation
            x = -1 * Math.cos(-botHeading);
            y = -1 * Math.sin(-botHeading);

            x = x * 1.1;  // Counteract imperfect strafing
        } else if (gamepad.dpad_left) {
            // Rotate the movement direction counter to the bot's rotation
            x = Math.cos(-botHeading);
            y = Math.sin(-botHeading);

            x = x * 1.1;  // Counteract imperfect strafing
        } else if (gamepad.dpad_up) {
            // Rotate the movement direction counter to the bot's rotation
            x = Math.sin(botHeading);
            y = Math.cos(botHeading);

            y = y * 1.1;  // Counteract imperfect strafing
        } else if (gamepad.dpad_down) {
            // Rotate the movement direction counter to the bot's rotation
            x = -1 * Math.sin(botHeading);
            y = -1 * Math.cos(botHeading);

            y = y * 1.1;  // Counteract imperfect strafing
        }

        if (moveSlow) {
            y *= 0.25;
            x *= 0.25;
            rx *= 0.25;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        y,
                        x,
                        rx
                )
        );

        drive.update();

        if (debug) {
            motorTelemetry(motorFL, "FL");
            motorTelemetry(motorBL, "BL");
            motorTelemetry(motorFR, "FR");
            motorTelemetry(motorBR, "BR");
        }
    }

    public void newDpadSupportedDrive(Gamepad gamepad, boolean exponential_drive, boolean slowdown, boolean turnSlow, Pose2d pose) {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        double botHeading = poseEstimate.getHeading();

        // Joystick inputs
        double y_raw = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x_raw = -gamepad.left_stick_x;
        double rx_raw = -gamepad.right_stick_x;

        // Initial movement values from joystick input, apply deadband if necessary
        double forward = Math.abs(y_raw) > DEAD_BAND ? y_raw : 0.0;
        double strafe = Math.abs(x_raw) > DEAD_BAND ? x_raw : 0.0;
        double rx = Math.abs(rx_raw) > DEAD_BAND ? rx_raw : 0.0;

        // Override with dpad input if any dpad button is pressed
        boolean dpadActive = gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right;
        if (dpadActive) {
            if (gamepad.dpad_up) {
                forward = 1.0;
            } else if (gamepad.dpad_down) {
                forward = -1.0;
            }

            if (gamepad.dpad_left) {
                strafe = -1.0;
            } else if (gamepad.dpad_right) {
                strafe = 1.0;
            }
        }

        // Rotate movement vector by robot's heading for dpad control, keeping joystick control as is
        if (dpadActive) {
            double tempForward = forward * Math.cos(botHeading) - strafe * Math.sin(botHeading);
            double tempStrafe = forward * Math.sin(botHeading) + strafe * Math.cos(botHeading);

            forward = tempForward;
            strafe = tempStrafe;
        }

        // Apply exponential scaling if desired
        if (exponential_drive) {
            forward = Math.signum(forward) * Math.pow(Math.abs(forward), 2.0);
            strafe = Math.signum(strafe) * Math.pow(Math.abs(strafe), 2.0);
            rx = Math.signum(rx) * Math.pow(Math.abs(rx), 2.0);
        }

        // Apply movement slow mode if activated
        if (moveSlow) {
            forward *= driveSlowMult;
            strafe *= driveSlowMult;
            rx *= driveSlowMult;
        }

        // Now apply the forward, strafe, and rx values to drive the robot
        drive.setWeightedDrivePower(new Pose2d(strafe, forward, rx));
        drive.update();

        // Debugging telemetry, if enabled
        if (debug) {
            motorTelemetry(motorFL, "FL");
            motorTelemetry(motorBL, "BL");
            motorTelemetry(motorFR, "FR");
            motorTelemetry(motorBR, "BR");
        }
    }

    public void motorTelemetry(DcMotor motor, String name) {
        opMode.telemetry.addLine("--- " + name + " ---");
        opMode.telemetry.addData(name + " Power", motor.getPower());
        opMode.telemetry.addData(name + " Position", motor.getCurrentPosition());
        opMode.telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    public void servoTelemetry(Servo servo, String name) {
        opMode.telemetry.addData(name + " Position", servo.getPosition());
    }

    private boolean pixelInFront(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Detect white
        if (red > 100 || green > 190 || blue > 160) {
            return true;
        } else {
            return false;
        }
    }

    private boolean pixelLocked(ColorSensor sensor) {
        int red = sensor.red();

        if (red > pixelLockDetectionRed) {
            return true;
        } else {
            return false;
        }
    }

    // SERVO METHODS
    public void setClawServoLeft(Gamepad gamepad, double closed_position, double open_position) {
        double position;
        if (fl_closed) {
            position = closed_position;
        } else {
            position = open_position;
        }

        if (gamepad.left_bumper && CSL.seconds() > 0.5) {
            if (fl_closed) {
                position = open_position;
                fl_closed = false;
            } else {
                position = closed_position;
                fl_closed = true;
            }
            CSL.reset();
            leftFingerAutoState = leftFingerAuto.IDLE;
        } else if (CSL.seconds() > 1 && leftFingerLastActive.milliseconds() > 500 && !fl_closed && autoPickup && Math.abs(ARM_GROUND - arm.getCurrentPosition()) <= 50) {
            leftFingerAutoState = leftFingerAuto.AWAITING_SIGNAL;
            if (pixelInFront(leftBottomColor)) {
                if (!fl_closed) {
                    position = closed_position;
                    fl_closed = true;
                    leftFingerLastActive.reset();
                }
                leftFingerAutoState = leftFingerAuto.CLOSING;
            }
        } else {
            leftFingerAutoState = leftFingerAuto.IDLE;
        }

        ClawServoLeft.setPosition(position);
    }

    public void GettouchSensorValue(TouchSensor touchSensor) {
        touchSensorPressed = touchSensor.isPressed();
        touchSensorValue = touchSensor.getValue();
    }

    public void setClawServoRight(Gamepad gamepad, double closed_position, double open_position) {
        double position;
        if (fr_closed) {
            position = closed_position;
        } else {
            position = open_position;
        }

        if (gamepad.right_bumper && CSR.seconds() > 0.5) {
            if (fr_closed) {
                position = open_position;
                fr_closed = false;
            } else {
                position = closed_position;
                fr_closed = true;
            }
            CSR.reset();
        } else if (CSR.seconds() > 1 && rightFingerLastActive.milliseconds() > 500 && !fr_closed && autoPickup && Math.abs(ARM_GROUND - arm.getCurrentPosition()) <= 50) {
            rightFingerAutoState = rightFingerAuto.AWAITING_SIGNAL;
            if (pixelInFront(rightBottomColor)) {
                if (!fr_closed) {
                    position = closed_position;
                    fr_closed = true;
                    rightFingerLastActive.reset();
                }
                rightFingerAutoState = rightFingerAuto.CLOSING;
            }
        } else {
            rightFingerAutoState = rightFingerAuto.IDLE;
        }

        ClawServoRight.setPosition(position);
    }

    public void activateDroneLauncher(Gamepad gamepad, ElapsedTime matchTime) {
        double power = 0;

        if (droneState == DroneCoverState.opening && DroneActive.milliseconds() > 1000) {
            droneState = DroneCoverState.open;
            DroneCover.setPower(0);
        }

        if (droneState == DroneCoverState.closing && DroneActive.milliseconds() > 1000) {
            droneState = DroneCoverState.closed;
            DroneCover.setPower(0);
        }

        if (gamepad.dpad_up && matchTime.seconds() > 90) {
            if (droneState == DroneCoverState.open) {
                power = 1.0;
            } else if (droneState != DroneCoverState.opening) {
                droneState = DroneCoverState.opening;
                DroneActive.reset();
            } else if (droneState == DroneCoverState.opening) {
                DroneCover.setPower(1.0);
            }
        } else if (gamepad.dpad_down) {
            if (droneState != DroneCoverState.closing) {
                droneState = DroneCoverState.closing;
                DroneActive.reset();
            } else if (droneState == DroneCoverState.closing) {
                DroneCover.setPower(-1.0);
            }
        }

        DroneServo.setPower(power);
    }

    // ARM AND JOINT MOTOR METHODS
    public double setJointPower(Gamepad gamepad) {
        jointPID.setPID(jointP, jointI, jointD);
        double power = 0;

        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;

        if (joint_angle > 360) {
            joint_angle -= 360;
        } else if (joint_angle < 0) {
            joint_angle += 360;
        }

        double joint_ff = Math.cos(Math.toRadians(joint_angle)) * joint_norm_F;
        double input = -gamepad.left_stick_y;

        if (gamepad.b) {
            double joint_out = jointPID.calculate(joint.getCurrentPosition(), JOINT_GROUND);
            joint_error = JOINT_GROUND - joint.getCurrentPosition();
            power = joint_out + joint_ff;
            joint_hold = JOINT_GROUND;
            jointState = JointState.GROUNDING;
        } else if (gamepad.a) {
            double joint_out = jointPID.calculate(joint.getCurrentPosition(), 0);
            joint_error = -joint.getCurrentPosition();
            power = joint_out + joint_ff;
            joint_hold = 0;
            jointState = JointState.GROUNDING;
        } else if (gamepad.x && gamepad.right_trigger > 0.75) {
            joint.setPower(0);
            joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            opMode.telemetry.addLine("Joint reset");
        } else if (gamepad.left_trigger > 0.5) {
            double joint_out = jointPID.calculate(joint.getCurrentPosition(), JOINT_AIRPLANE);
            power = joint_out + joint_ff;
            joint_hold = JOINT_AIRPLANE;
            jointState = JointState.PLANE_LAUNCHING;
        } else if (Math.abs(gamepad.left_stick_y) > 0.1) {
            power = input + joint_ff;
            joint_hold = joint.getCurrentPosition();
            jointState = JointState.DRIVER_CONTROL;
        } else if (gamepad.right_trigger > 0.5) {
            double joint_out = jointPID.calculate(joint.getCurrentPosition(), jointSetPoint);
            power = joint_out + joint_ff;
            joint_hold = jointSetPoint;
            jointState = JointState.FORWARDS_SCORING;
        } else if (joint.getCurrentPosition() > -10) {
            jointState = JointState.GROUNDING;
            power = 0.0;
            joint_hold = 0;
        } else {
            if (gamepad.y) {
                jointSetPoint = joint.getCurrentPosition();
            }

            if (jointState != JointState.HOLDING) {
                joint_hold = joint.getCurrentPosition();
            }
            double joint_out = jointPID.calculate(joint.getCurrentPosition(), joint_hold);
            power = joint_out + joint_ff;
            jointState = JointState.HOLDING;

            if (joint.getCurrentPosition() > -20) {
                power = 0;
            }
        }

        if (power > 1.0) {
            power = 1.0;
        } else if (power < -1.0) {
            power = -1.0;
        }

        return power;
    }

    public double setArmPower(Gamepad gamepad) {
        armPID.setPID(armP, armI, armD);
        double power = 0;

        double currentArmPosition = arm.getCurrentPosition();
        double armVelocity = currentArmPosition - lastArmPosition; // Positive if increasing, negative if decreasing

        // Calculate angles and feedforward
        double joint_angle = joint.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = currentArmPosition / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;
        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        double input = -gamepad.right_stick_y;
        if (gamepad.b) {
            // ground!
            double arm_out = armPID.calculate(arm.getCurrentPosition(), ARM_GROUND);
            power = arm_out + arm_ff;
            arm_hold = ARM_GROUND;
            armState = ArmState.GROUNDING;
        } else if (gamepad.a) {
            double arm_out = armPID.calculate(arm.getCurrentPosition(), 0);
            power = arm_out + arm_ff;
            arm_hold = 0;
            armState = ArmState.HOMING;
        } else if (gamepad.right_trigger > 0.5) {
            double arm_out = armPID.calculate(arm.getCurrentPosition(), armSetPoint);
            power = arm_out + arm_ff;
            armState = ArmState.FORWARDS_SCORING;
        } else if (gamepad.left_trigger > 0.5) {
            // airplane!
            double arm_out = armPID.calculate(arm.getCurrentPosition(), ARM_AIRPLANE);
            power = arm_out + arm_ff;
            arm_hold = ARM_FORWARDS_SCORE;
            armState = ArmState.PLANE_LAUNCHING;
        } else if (Math.abs(gamepad.right_stick_y) > 0.05) {
            double distanceToZero = Math.abs(currentArmPosition);
            if (currentArmPosition < 0) {
                distanceToZero = 0.0;
            }
            double maxThreshold = 100;
            double scalingFactor = 1.0;

            // Apply dynamic scaling based on arm's movement direction and position
            if (distanceToZero <= maxThreshold && armVelocity < 0) { // Apply only when moving towards zero
                scalingFactor = Math.max(0.1, distanceToZero / maxThreshold);
            }

            power = input * scalingFactor;

            if (gamepad.right_trigger > 0.5) {
                power *= 0.3;
            }

            arm_hold = currentArmPosition;
            armState = ArmState.DRIVER_CONTROL;
        } else if (gamepad.x && gamepad.right_trigger > 0.75) {
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            opMode.telemetry.addLine("Arm reset");
        } else {
            if (gamepad.y) {
                armSetPoint = arm.getCurrentPosition();
            }
            // Transition to holding state with smooth reduction in power
            if (armState != ArmState.HOLDING && armState != ArmState.WAITING_TO_HOLD) {
                ARM_TIME_TO_HOLD.reset();
                armState = ArmState.WAITING_TO_HOLD;
            }

            // Wait for 500ms before setting hold - allowing some time for the arm to settle
            if (armState == ArmState.WAITING_TO_HOLD && ARM_TIME_TO_HOLD.milliseconds() > 1000) {
                arm_hold = arm.getCurrentPosition();
                armState = ArmState.HOLDING;
            } else {
                power = arm_ff;
            }

            if (armState == ArmState.HOLDING) {
                // Use PIDF to hold the arm position
                double arm_out = armPID.calculate(arm.getCurrentPosition(), arm_hold);
                power = arm_out + arm_ff;
            }

            if (Math.abs(arm.getCurrentPosition()) < 20) {
                power = 0;
            }
        }

        power = Math.max(-1.0, Math.min(1.0, power));

        lastArmPosition = currentArmPosition;

        return power;
    }

    // still in progress
    public void turnToHeading(double targetHeading, double currentHeading, double max_power, double sleepTime) {
        double error = targetHeading - currentHeading;

        // PID
        double p = error * 0.01;
        double i = 0;
        double d = 0;

        double power = p + i + d;

        if (power > max_power) {
            power = max_power;
        } else if (power < -max_power) {
            power = -max_power;
        }
    }

    // TELEMETRY METHODS
    public void checkGamepadParameters(Gamepad gamepad, String position) {
        if (debug) {
            opMode.telemetry.addLine("--- " + position + " ---");

            if (gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0) {
                opMode.telemetry.addData("Left Stick X", gamepad.left_stick_x);
                opMode.telemetry.addData("Left Stick Y", gamepad.left_stick_y);
            }

            if (gamepad.right_stick_x != 0 || gamepad.right_stick_y != 0) {
                opMode.telemetry.addData("Right Stick X", gamepad.right_stick_x);
                opMode.telemetry.addData("Right Stick Y", gamepad.right_stick_y);

            }

            /*
            if (touchSensorPressed || joint.getCurrentPosition() - 10 > 0 || arm.getCurrentPosition() - 10 > 0){
                opMode.telemetry.addData("touchSensorPressed", touchSensorPressed);
                DcMotor.RunMode JstopAndResetEncoder = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                joint.setMode(JstopAndResetEncoder);
                DcMotor.RunMode AstopAndResetEncoder = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
                arm.setMode(AstopAndResetEncoder);
            }
             */

            if (gamepad.left_trigger != 0) {
                opMode.telemetry.addData("Left Trigger", gamepad.left_trigger);
            }

            if (gamepad.right_trigger != 0) {
                opMode.telemetry.addData("Right Trigger", gamepad.right_trigger);
            }
            if (gamepad.left_stick_button) {
                opMode.telemetry.addLine("LeftStick Button Pressed");
            }
            if (gamepad.right_stick_button) {
                opMode.telemetry.addLine("RightStick Button Pressed");
            }

            if (gamepad.dpad_up) {
                opMode.telemetry.addLine("DPad Up Pressed");
            }
            if (gamepad.dpad_down) {
                opMode.telemetry.addLine("DPad Down Pressed");
            }
            if (gamepad.dpad_left) {
                opMode.telemetry.addLine("DPad Left Pressed");
            }
            if (gamepad.dpad_right) {
                opMode.telemetry.addLine("DPad Right Pressed");
            }

            if (gamepad.a) {
                opMode.telemetry.addLine("A pressed");
            }
            if (gamepad.b) {
                opMode.telemetry.addLine("B pressed");
            }
            if (gamepad.x) {
                opMode.telemetry.addLine("X pressed");
            }
            if (gamepad.y) {
                opMode.telemetry.addLine("Y pressed");
            }

            if (gamepad.left_bumper) {
                opMode.telemetry.addLine("Left bumper clicked");
            }
            if (gamepad.right_bumper) {
                opMode.telemetry.addLine("Right bumper clicked");
            }

            opMode.telemetry.addLine("\n");
            opMode.telemetry.addData("Joint Hold", joint_hold);
            opMode.telemetry.addData("Joint Current", joint.getCurrentPosition());
            opMode.telemetry.addData("Joint Power", joint.getPower());

            opMode.telemetry.addLine("\n");
            opMode.telemetry.addData("Arm Hold", arm_hold);
            opMode.telemetry.addData("Arm Position", arm.getCurrentPosition());
            opMode.telemetry.addData("Arm Power", arm.getPower());
        }
    }

    public void logGamepad(Telemetry telemetry, Gamepad gamepad, String position) {
        telemetry.addLine("--- " + position + " ---");
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) {
                continue;
            }

            try {
                telemetry.addData(position + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }
        }
    }
}