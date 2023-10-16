package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Airplane Launcher")
public class Trebuchet extends LinearOpMode {

    private DcMotor motorT;

    @Override
    public void runOpMode() throws InterruptedException {

        motorT = hardwareMap.dcMotor.get("motorJoint");

        // Set motor directions

        motorT.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            double y = gamepad1.touchpad_finger_1_y;

            motorT.setPower(1);

            telemetry.addData("Arm Encoder", motorT.getCurrentPosition());
            telemetry.update();


            sleep(50);
        }
    }
}