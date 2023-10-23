package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//https://github.com/NerdHerd-FTC/CAMS-FTC/blob/develop_Jadon/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumControlDriverOriented.java
@TeleOp(name = "Mecanum Drive - Field Centric")
public class MecanumFieldCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean exponential_drive = true;
        boolean slowdown = false;

        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBL = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFR = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBR = hardwareMap.dcMotor.get("backRight");

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
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        // Technically this is the default, however specifying it is clearer
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(imuParams);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            // Reset yaw to 0
            if (gamepad1.x) {
                imu.resetYaw();
            }

            // Toggle exponential drive
            if (gamepad1.y) {
                exponential_drive = !exponential_drive;
            }

            // Toggle slowdown
            if (gamepad1.a) {
                //slowdown = !slowdown;
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

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
    }
}