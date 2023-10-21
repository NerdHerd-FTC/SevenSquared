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

@TeleOp(name = "Drone Launcher")
@Disabled
public class DroneLauncherTesting extends LinearOpMode {
    ElapsedTime CSR = new ElapsedTime();
    ElapsedTime CSL = new ElapsedTime();
    ElapsedTime matchTime = new ElapsedTime();

    // Servo info
    boolean fr_closed = false;
    boolean fl_closed = false;
    boolean csr_on = false;
    boolean csl_on = false;

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");

        waitForStart();

        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) return;

        matchTime.reset();

        while (opModeIsActive()) {

            activateDroneLauncher(DroneServo, gamepad2, 1);

            checkGamepadParameters(gamepad1, "Driver");
            checkGamepadParameters(gamepad2, "Operator");
            telemetry.addLine("\n");
            checkGamepadParameters(gamepad2, "Operator");

            telemetry.update();
            sleep(50);
        }
    }

    private void activateDroneLauncher(CRServo DroneServo, Gamepad gamepad, double launch_speed) {
        double power = 0;
        if(gamepad.a && matchTime.seconds() > 0)  {
            power = launch_speed;
        }

        DroneServo.setPower(power);
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

    private void ServoTelemetry(Servo wrist, Servo FrontRight, Servo FrontLeft) {
        telemetry.addLine("--- Servo ---");
        telemetry.addData("FrontRollerRight Closed", fr_closed);
        telemetry.addData("Front Right Location", FrontRight.getPosition());
        telemetry.addData("FrontRollerLeft Closed", fl_closed);
        telemetry.addData("Front Left Location", FrontLeft.getPosition());
        telemetry.addData("CSR On", csr_on);
        telemetry.addData("CSR Timer", CSR.seconds());
        telemetry.addData("CSL On", csl_on);
        telemetry.addData("CSL Timer", CSL.seconds());
        telemetry.addData("Wrist", wrist.getPosition());
    }
    //-621, 0
    // -600
}