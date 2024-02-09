package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TeleUtil;

@Config
@TeleOp(name = "Modern Robot Drive")
public class NewRobotDrive extends LinearOpMode {
    private ElapsedTime matchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Get motors
        DcMotor jointMotor = hardwareMap.dcMotor.get("joint");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

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

        // Get servos
        Servo ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");

        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.FORWARD);
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // TeleUtil instance
        TeleUtil teleUtil = new TeleUtil(this, motorFL, motorFR, motorBL, motorBR, armMotor, jointMotor, ClawServoLeft, ClawServoRight, DroneServo);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        matchTime.reset();

        while (opModeIsActive()) {
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            // Drive
            teleUtil.dpadSupportedRobotDrive(gamepad1, true, false, false, myPose);

            // Articulation
            jointMotor.setPower(teleUtil.setJointPower(gamepad2));
            armMotor.setPower(teleUtil.setArmPower(gamepad2));

            teleUtil.setClawServoRight(gamepad2, RobotConstants.CLAW_RIGHT_CLOSED , RobotConstants.CLAW_RIGHT_OPEN);
            teleUtil.setClawServoLeft(gamepad2, RobotConstants.CLAW_LEFT_CLOSED, RobotConstants.CLAW_LEFT_OPEN);

            teleUtil.activateDroneLauncher(gamepad2, matchTime);

            // Servo Telemetry
            teleUtil.servoTelemetry(ClawServoLeft, "Left Claw");
            teleUtil.servoTelemetry(ClawServoRight, " Right Claw");

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();
        }
    }
}