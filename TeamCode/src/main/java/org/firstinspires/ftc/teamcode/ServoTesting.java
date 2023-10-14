package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Single Servo Testing")
public class ServoTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        Servo ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        Servo DroneServo = hardwareMap.get(Servo.class, "DS");
        Servo WristServo = hardwareMap.get(Servo.class, "WS");

        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.REVERSE);
        ClawServoLeft.setDirection(Servo.Direction.FORWARD);

        // List of servos to test
        List<Servo> servos = new ArrayList<Servo>();
        servos.add(ClawServoRight);
        servos.add(ClawServoLeft);
        servos.add(DroneServo);
        servos.add(WristServo);

        waitForStart();

        if (isStopRequested()) return;

        double location = 0;
        int position = 0;

        double CSR_pos = 0;
        double CSL_pos = 0;
        double DS_pos = 0;
        double WS_pos = 0;

        Servo currentServo;

        while (opModeIsActive()) {
            currentServo = servos.get(position);
            if (gamepad1.y) {
                location = location + 0.1;
            } else if (gamepad1.a) {
                location = location - 0.1;
            } else if (gamepad1.x) {
                location = location + 0.01;
            } else if (gamepad1.b) {
                location = location - 0.01;
            }

            if (gamepad1.dpad_up) {
                position = position + 1;
            } else if (gamepad1.dpad_down) {
                position = position - 1;
            }

            currentServo.setPosition(location);

            if (currentServo == ClawServoRight) {
                CSR_pos = location;
            } else if (currentServo == ClawServoLeft) {
                CSL_pos = location;
            } else if (currentServo == DroneServo) {
                DS_pos = location;
            } else if (currentServo == WristServo) {
                WS_pos = location;
            }

            telemetry.addData("Current Servo", currentServo);
            telemetry.addData("Current Location", location);
            telemetry.addLine("\n\n");

            telemetry.addData("CSR Last Position", CSR_pos);
            telemetry.addData("CSL Last Position", CSL_pos);
            telemetry.addData("DS Last Position", DS_pos);
            telemetry.addData("WS Last Position", WS_pos);

            telemetry.update();
        }
    }
}