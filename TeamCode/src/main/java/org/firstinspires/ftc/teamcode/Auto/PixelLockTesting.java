// Code Created By Derrick, Owen, Shash
package org.firstinspires.ftc.teamcode.Auto;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.TeleUtil;

import java.util.Objects;

@Config
@TeleOp(name = "Pixel Lock!")
public class PixelLockTesting extends LinearOpMode {
    private ElapsedTime matchTime = new ElapsedTime();
    public ColorSensor topColorSensor, bottomColorSensor;

    private ElapsedTime callGap = new ElapsedTime();
    private int callsToColor = 0;

    public static int whiteRedThresholdBottom = 130;
    public static int whiteRedThresholdTop = 350;
    public ElapsedTime pixelLock = new ElapsedTime();

    public DcMotor jointMotor, armMotor;

    private PIDController armPID = new PIDController(armP, armI, armD);
    private PIDController jointPID = new PIDController(jointP, jointI, jointD);

    public Integer bottomRed;
    public Integer bottomGreen;
    public Integer bottomBlue;

    public Integer topRed;
    public Integer topGreen;
    public Integer topBlue;

    public Servo ClawServoRight;

    public AutoUtil.ARM_DEMANDS armDemands = AutoUtil.ARM_DEMANDS.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Get motors
        jointMotor = hardwareMap.dcMotor.get("joint");
        armMotor = hardwareMap.dcMotor.get("arm");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setDirection(DcMotor.Direction.REVERSE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive control variables
        boolean exponential_drive = true;

        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBL = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFR = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBR = hardwareMap.dcMotor.get("backRight");

        // Unlock full speed of drive motors
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Left motors should move in reverse
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Right motors should move forward
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set drive motors to brake when power is set to 0
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get servos
        ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");

        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.FORWARD);
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);

        topColorSensor = hardwareMap.get(ColorSensor.class, "topColor");
        bottomColorSensor = hardwareMap.get(ColorSensor.class, "bottomColor");

        // TeleUtil instance
        TeleUtil teleUtil = new TeleUtil(this, motorFL, motorFR, motorBL, motorBR, armMotor, jointMotor, ClawServoLeft, ClawServoRight, DroneServo);

        waitForStart();

        if (isStopRequested()) return;

        ClawServoRight.setPosition(CLAW_RIGHT_OPEN);

        matchTime.reset();

        while (opModeIsActive()) {
            // Pixel lock
            lockOntoPixel();

            teleUtil.motorTelemetry(armMotor, "Arm");

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());

            telemetry.update();
        }
    }

    private boolean isWhite(ColorSensor sensor, int threshold, String name) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        if (Objects.equals(name, "TOP")) {
            topBlue = blue;
            topRed = red;
            topGreen = green;
        } else if (Objects.equals(name, "BOTTOM")) {
            bottomBlue = blue;
            bottomRed = red;
            bottomGreen = green;
        }

        return red > threshold && green > threshold && blue > threshold;
    }

    public double lockOntoPixel() {
        if (callGap.milliseconds() > 300) {
            callGap.reset();
            // Check if top and bottom sensors detect white
            boolean topDetectsWhite = isWhite(topColorSensor, whiteRedThresholdTop, "TOP");
            boolean bottomDetectsWhite = isWhite(bottomColorSensor, whiteRedThresholdBottom, "BOTTOM");
            if (topDetectsWhite && bottomDetectsWhite) {
                // If both sensors detect white
                armDemands = AutoUtil.ARM_DEMANDS.MOVE_UP;
            } else if (!topDetectsWhite && bottomDetectsWhite) {
                // If only the bottom sensor detects white
                armDemands = AutoUtil.ARM_DEMANDS.HOLD;
            } else if (!(topDetectsWhite && bottomDetectsWhite)) {
                // Assuming this condition is meant to be if neither sensor detects white
                armDemands = AutoUtil.ARM_DEMANDS.MOVE_DOWN;
            } else if (topDetectsWhite && !bottomDetectsWhite) {
                telemetry.addData("Scuffed Readings", "Top but not bottom...");
            }

            if (armDemands == AutoUtil.ARM_DEMANDS.MOVE_UP) {
                pixelLock.reset();
                asyncMoveArm(armMotor.getCurrentPosition() - 2);
            } else if (armDemands == AutoUtil.ARM_DEMANDS.MOVE_DOWN) {
                pixelLock.reset();
                asyncMoveArm(armMotor.getCurrentPosition() + 2);
            } else {
                asyncMoveArm(armMotor.getCurrentPosition());
                moveRightFinger(CLAW_RIGHT_CLOSED);
            }

            callsToColor += 1;
        }

        telemetry.addData("Top Red", topRed);
        telemetry.addData("Top Green", topGreen);
        telemetry.addData("Top Blue", topBlue);

        telemetry.addLine("\n");

        telemetry.addData("Bottom Red", bottomRed);
        telemetry.addData("Bottom Green", bottomGreen);
        telemetry.addData("Bottom Blue", bottomBlue);

        telemetry.addData("Pixel Lock", pixelLock.milliseconds());
        telemetry.addData("Calls to Color", callsToColor);

        return pixelLock.milliseconds();
    }

    public double asyncMoveArm(double target) {
        double error = target - armMotor.getCurrentPosition();

        double joint_angle = jointMotor.getCurrentPosition() / joint_ticks_per_degree + 193;
        double relative_arm_angle = armMotor.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
        double arm_angle = 270 - relative_arm_angle - joint_angle;

        double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

        double arm_out = armPID.calculate(armMotor.getCurrentPosition(), target);

        double arm_power = arm_ff + arm_out;

        armMotor.setPower(arm_power);

        //telemetry.addData("Arm Error", error);
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());

        return error;
    }

    public void moveRightFinger(double target) {
        ClawServoRight.setPosition(target);
    }
}