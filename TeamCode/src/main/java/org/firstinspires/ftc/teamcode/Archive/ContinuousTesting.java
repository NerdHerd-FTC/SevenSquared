package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class ContinuousTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        CRServo servo = hardwareMap.get(CRServo.class, "FRSR");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                servo.setPower(gamepad1.right_stick_y);
            }
        }
    }
}
