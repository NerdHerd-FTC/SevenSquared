package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

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
        PIDController visionController = new PIDController(0.2,0,0,0.1);



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
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(imuParams);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            sleep(100);
            double y = -gamepad1.left_stick_y; // Y stick is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double theta = gamepad1.right_stick_x; //Rotate by theta
            if(gamepad1.left_stick_button){

            }
            else{
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
                //Set motor powers
                double FLPower = (y + x + theta) / denominator;
                double BLPower = (y - x + theta) / denominator;
                double FRPower = (y - x - theta) / denominator;
                double BRPower = (y + x - theta) / denominator;

                //Run motors using powers
                motorFL.setPower(FLPower);
                motorBL.setPower(BLPower);
                motorFR.setPower(FRPower);
                motorBR.setPower(BRPower);
            }


        }
    }
}
