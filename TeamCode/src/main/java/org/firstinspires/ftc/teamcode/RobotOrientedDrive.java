// Code Created By Derrick, Owen, Shash
package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_LEFT_OPEN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.CLAW_RIGHT_OPEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TeleUtil;

@Config
@TeleOp(name = "Robot Drive")
public class RobotOrientedDrive extends LinearOpMode {
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
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set drive motors to brake when power is set to 0
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Get servos
        Servo ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");
        CRServo DroneCover = hardwareMap.get(CRServo.class, "DC");

        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.FORWARD);
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);

        ColorSensor topColor = hardwareMap.get(ColorSensor.class, "topColor");
        ColorSensor bottomColor = hardwareMap.get(ColorSensor.class, "bottomColor");
        ColorSensor floorColor = hardwareMap.get(ColorSensor.class, "colorV3");

        topColor.enableLed(false);
        bottomColor.enableLed(false);

        // TeleUtil instance
        TeleUtil teleUtil = new TeleUtil(this, motorFL, motorFR, motorBL, motorBR, armMotor, jointMotor, ClawServoLeft, ClawServoRight, DroneServo, DroneCover);

        waitForStart();

        if (isStopRequested()) return;

        matchTime.reset();

        while (opModeIsActive()) {
            // Drive
            teleUtil.robotOrientedDrive(gamepad1, true, false, false);

            // Articulation
            jointMotor.setPower(teleUtil.setJointPower(gamepad2));
            armMotor.setPower(teleUtil.setArmPower(gamepad2));

            teleUtil.setClawServoRight(gamepad2, CLAW_RIGHT_CLOSED, CLAW_RIGHT_OPEN);
            teleUtil.setClawServoLeft(gamepad2, CLAW_LEFT_OPEN, CLAW_LEFT_CLOSED);

            teleUtil.activateDroneLauncher(gamepad2, matchTime);

            // telemetry.addLine("\n");

            // Gamepad Telemetry
            // teleUtil.checkGamepadParameters(gamepad1, "Driver");
            // teleUtil.checkGamepadParameters(gamepad2, "Operator");
            telemetry.addLine("\n");

            // Motor Telemetry
            teleUtil.motorTelemetry(jointMotor, "Joint");
            telemetry.addLine("\n");
            teleUtil.motorTelemetry(armMotor, "Arm");
            telemetry.addLine("\n");

            // Servo Telemetry
            teleUtil.servoTelemetry(ClawServoLeft, "Left Claw");
            teleUtil.servoTelemetry(ClawServoRight, " Right Claw");

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());

            if (gamepad1.dpad_right) {
                showMotorPower(motorFL, "FL");
                showMotorPower(motorFR, "FR");
                showMotorPower(motorBL, "BL");
                showMotorPower(motorBR, "BR");
            }

            if (gamepad2.dpad_right) {
                getColors(topColor, "top");
                getColors(bottomColor, "bottom");
                getColors(floorColor, "floor");

            }
            telemetry.update();
        }
    }

    public void getColors(ColorSensor sensor, String name) {
        telemetry.addData("Color Sensor", name);
        telemetry.addData("Red", sensor.red());
        telemetry.addData("Green", sensor.green());
        telemetry.addData("Blue", sensor.blue());
    }

    public void showMotorPower(DcMotor motor, String name) {
        telemetry.addData(name + " Power", motor.getPower());
    }
}