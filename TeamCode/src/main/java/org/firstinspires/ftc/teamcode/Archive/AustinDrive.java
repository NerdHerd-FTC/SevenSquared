// Code Created By Derrick, Owen, Shassh
/*
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.TeleUtil;

@Config
@TeleOp(name = "Unrestricted Austin Drive")
public class AustinDrive extends LinearOpMode {
    private ElapsedTime matchTime = new ElapsedTime();
    private TeleUtil teleUtil;

    @Override
    public void runOpMode() throws InterruptedException {
        // REMOVE THESE BEFORE COMPETITION
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
        boolean slowdown = false;
        boolean debug = false;

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

        // Right motors should move in reverse
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Left motors should move forward
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Get servos
        Servo ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");
        CRServo WristServo = hardwareMap.get(CRServo.class, "FRSR");

        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.REVERSE);
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);

        TeleUtil teleUtil = new TeleUtil(this, motorFL, motorFR, motorBL, motorBR, armMotor, jointMotor, ClawServoLeft, ClawServoRight, DroneServo, WristServo);

        waitForStart();

        if (isStopRequested()) return;

        matchTime.reset();

        while (opModeIsActive()) {
            // Get yaw
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Reset yaw to 0 7
            if (gamepad1.x) {
                imu.resetYaw();
                telemetry.addLine("IMU reset!");
            }

            // Gyro Telemetry
            teleUtil.gyroTelemetry(yaw);
            telemetry.addLine("\n");

            // Drive
            teleUtil.fieldOrientedDrive(yaw, gamepad1, true, false, false);

            // Articulation
            jointMotor.setPower(teleUtil.setJointPower(gamepad2));
            armMotor.setPower(teleUtil.setArmPower(gamepad2));

            teleUtil.setClawServoRight(gamepad2, 0,1);
            teleUtil.setClawServoLeft(gamepad2,0.6,0);

            teleUtil.setWristServoPower(gamepad2);

            teleUtil.activateDroneLauncher(gamepad2, matchTime);

            telemetry.addLine("\n");

            // Gamepad Telemetry
            teleUtil.logGamepad(telemetry, gamepad1, "Driver");
            teleUtil.checkGamepadParameters(gamepad1, "Driver");
            teleUtil.checkGamepadParameters(gamepad2, "Operator");
            telemetry.addLine("\n");

            // Motor Telemetry
            teleUtil.motorTelemetry(jointMotor, "Joint");
            telemetry.addLine("\n");
            teleUtil.motorTelemetry(armMotor, "Arm");
            telemetry.addLine("\n");

            // Servo Telemetry
            teleUtil.servoTelemetry(WristServo, ClawServoRight, ClawServoLeft);

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());
            telemetry.update();
            sleep(50);
        }
    }
}

 */