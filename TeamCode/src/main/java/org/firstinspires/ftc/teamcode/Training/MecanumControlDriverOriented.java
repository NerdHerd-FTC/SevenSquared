/*
package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//https://github.com/NerdHerd-FTC/CAMS-FTC/blob/develop_Jadon/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumControlDriverOriented.java
@TeleOp(name = "Mecanum Control Driver Oriented - SIM")
public class MecanumControlDriverOriented extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
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
            double y = -____; // Y stick is reversed
            double x = ____ * 1.1; // Counteract imperfect strafing
            double theta = gamepad1.right_stick_x; //Rotate by theta

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
            //Set motor powers
            double FLPower = (y + x + theta) / denominator;
            double BLPower = (y - x + theta) / denominator;
            double FRPower = (y - x - theta) / denominator;
            double BRPower = (y + x - theta) / denominator;

            //Run motors using powers
            motorFL.setPower(_);
            motorBL.setPower(_);
            motorFR.setPower(_);
            motorBR.setPower(_);
        }
    }
}
\*
 */