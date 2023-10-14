package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IMU Test")
public class IMUTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        IMU imu = hard  wareMap.get(IMU.class, "imu");
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
            telemetry.addLine("IMU Data");
            telemetry.addData("IMU Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            telemetry.update();
            sleep(100);
        }
    }
}