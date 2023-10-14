package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Control Driver Oriented - SIM")
public class MecanumControlDriverOriented extends LinearOpMode {

    private DcMotor motorFL, motorBL, motorFR, motorBR, motorJoint;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        motorFL = hardwareMap.dcMotor.get("frontLeft");
        motorBL = hardwareMap.dcMotor.get("backLeft");
        motorFR = hardwareMap.dcMotor.get("frontRight");
        motorBR = hardwareMap.dcMotor.get("backRight");
        motorJoint = hardwareMap.dcMotor.get("motorJoint");

        // Set motor directions
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorJoint.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double theta = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(theta), 1);
            double FLPower = (y + x + theta) / denominator;
            double BLPower = (y - x + theta) / denominator;
            double FRPower = (y - x - theta) / denominator;
            double BRPower = (y + x - theta) / denominator;

            motorFL.setPower(FLPower);
            motorBL.setPower(BLPower);
            motorFR.setPower(FRPower);
            motorBR.setPower(BRPower);

            // Arm control
            double armPower = getArmPower(gamepad2);

            motorJoint.setPower(armPower);

            telemetry.addData("Arm Encoder", motorJoint.getCurrentPosition());
            telemetry.update();

           
            sleep(50);
        }
    }

    private double getArmPower(Gamepad gamepad) {
        double power = 0;
        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;

        if (rightTrigger >= 0.4) {
            power = 0.1;
        } else if (leftTrigger >= 0.4) {
            power = -0.1;
        }

        return power;
    }
}
