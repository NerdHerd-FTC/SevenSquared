package org.firstinspires.ftc.teamcode.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp()
@Disabled
public class MotorTest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor=hardwareMap.dcMotor.get("arm");
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Arm", motor.getPower());
            telemetry.update();
        }

    }
}