package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Custom PID")
@Disabled
public class CustomPID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("frontLeft");
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int holding_joint_location = 0;

        double Kp = 0.01; // 0.001 for joint, 0.01 for arm
        double Ki = 0;
        double Kd = 0;

        int DELTA_T = 35;
        double D_MULT = Kd / DELTA_T;

        int[] macroPositions = {0, 200, 400, 600};
        double MACRO_POWER = 0.6; //for quick adjustments
        double prevPos = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double error = holding_joint_location - jointMotor.getCurrentPosition();

            if (error > 10) {
                //Base PID
                double P1 = Kp * error;
                double D1 = Kd;

                double powerPDF1 = (P1);
                prevPos = jointMotor.getCurrentPosition();

                //Final RV4B Motor Powers
                jointMotor.setPower(Math.tanh(powerPDF1));
            }

            // Macros for better PID tuning
            if (gamepad1.a) {
                holding_joint_location = macroPositions[0];
            } else if (gamepad1.b) {
                holding_joint_location = macroPositions[1];
            } else if (gamepad1.x) {
                holding_joint_location = macroPositions[2];
            } else if (gamepad1.y) {
                holding_joint_location = macroPositions[3];
            }

            // Adjust Kp using gamepad bumpers
            if (gamepad1.right_bumper && Kp <= 100) {
                Kp += 1; // Increase Kp by 0.001
                sleep(250); // Delay to prevent rapid-fire changes
            } else if (gamepad1.left_bumper && Kp >= 1) {
                Kp -= 1; // Decrease Kp by 0.001
                sleep(250); // Delay to prevent rapid-fire changes
            }

            telemetry.addLine("----- JOINT MOTOR -----");
            telemetry.addData("joint power", jointMotor.getPower());
            telemetry.addData("joint position", jointMotor.getCurrentPosition());
            telemetry.addData("joint target", jointMotor.getTargetPosition());
            telemetry.addData("joint error", jointMotor.getTargetPosition() - jointMotor.getCurrentPosition());
            telemetry.addData("holding position", holding_joint_location);
            telemetry.addData("mode", jointMotor.getMode());

            telemetry.addLine("----- PID COEFFICIENTS -----");
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);

            telemetry.update();

            sleep(DELTA_T);
        }
    }
}
