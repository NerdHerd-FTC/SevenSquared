// Code Created By Derrick, Owen, Shash
package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive Only")
@Disabled
public class DriveTesting extends LinearOpMode {
    ElapsedTime matchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive control variables
        boolean exponential_drive = true;
        boolean slowdown = false;

        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

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

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams;

        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        // Technically this is the default, however specifying it is clearer
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(imuParams);

        waitForStart();

        imu.resetYaw();

        if (isStopRequested()) return;

        matchTime.reset();

        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Drive
            fieldOrientedDrive(heading, gamepad1, exponential_drive, slowdown, motorFL, motorBL, motorFR, motorBR);

            // Reset yaw to 0 7
            if (gamepad1.x) {
                imu.resetYaw();
                telemetry.addLine("IMU reset");
            }

            // Gyro Telemetry
            gyroTelemetry(heading);
            telemetry.addLine("\n");

            // Gamepad Telemetry
            checkGamepadParameters(gamepad1, "Driver");
            telemetry.addLine("\n");

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());
            telemetry.update();
            sleep(50);
        }
    }

    private void fieldOrientedDrive(double rawHeading, Gamepad gamepad, boolean exponential_drive, boolean slowdown, DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR) {
        double y_raw = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x_raw = gamepad1.left_stick_x;
        double rx_raw = gamepad1.right_stick_x;

        // Deadband to address controller drift
        double deadband = 0.1;
        if (Math.abs(y_raw) < deadband) {
            y_raw = 0;
        }
        if (Math.abs(x_raw) < deadband) {
            x_raw = 0;
        }
        if (Math.abs(rx_raw) < deadband) {
            rx_raw = 0;
        }

        // Toggle exponential drive
        if (gamepad1.y) {
            exponential_drive = !exponential_drive;
        }

        // Toggle slowdown
        if (gamepad1.a) {
            slowdown = !slowdown;
        }

        // Exponential Drive
        double exponent = 2.0;
        double y = exponential_drive ? Math.signum(y_raw) * Math.pow(Math.abs(y_raw), exponent) : y_raw;
        double x = exponential_drive ? Math.signum(x_raw) * Math.pow(Math.abs(x_raw), exponent) : x_raw;
        double rx = exponential_drive ? Math.signum(rx_raw) * Math.pow(Math.abs(rx_raw), exponent) : rx_raw;

        // Slowdown
        if (slowdown) {
            y *= 0.75;
            x *= 0.75;
            rx *= 0.75;
        }

        double botHeading;

        // restrict range to [0, 360]
        if (rawHeading <= 0) {
            botHeading = rawHeading + 2 * Math.PI;
        } else {
            botHeading = rawHeading;
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFL.setPower(frontLeftPower);
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);
    }

    // TELEMETRY METHODS
    private void checkGamepadParameters(Gamepad gamepad, String position) {
        telemetry.addLine("--- " + position + " ---");
        if (gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0) {
            telemetry.addData("Left Stick X", gamepad.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad.left_stick_y);
        }

        if (gamepad.right_stick_x != 0 || gamepad.right_stick_y != 0) {
            telemetry.addData("Right Stick X", gamepad.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad.right_stick_y);

        }

        if (gamepad.left_trigger != 0) {
            telemetry.addData("Left Trigger", gamepad.left_trigger);
        }

        if (gamepad.right_trigger != 0) {
            telemetry.addData("Right Trigger", gamepad.right_trigger);
        }

        if (gamepad.dpad_up) {
            telemetry.addLine("DPad Up Pressed");
        }
        if (gamepad.dpad_down) {
            telemetry.addLine("DPad Down Pressed");
        }
        if (gamepad.dpad_left) {
            telemetry.addLine("DPad Left Pressed");
        }
        if (gamepad.dpad_right) {
            telemetry.addLine("DPad Right Pressed");
        }
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    private void gyroTelemetry(double heading) {
        telemetry.addLine("--- Gyro ---");
        if (heading == Double.doubleToLongBits(-0.0)) {
            telemetry.addLine("!!! Heading: -0.0 !!!");
            telemetry.addLine("\n\n\n\n\n\n\n\n\n\n");
        }
        telemetry.addData("Heading", heading);
    }
}