package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PID Tuning")
@Disabled
public class PIDTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("frontLeft");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean hold_joint = false;

        int holding_joint_location = 0;

        double Kp = 0.01; // 0.001 for joint, 0.01 for arm
        double Ki = 0;
        double Kd = 0;

        double prevKp = Kp;

        int[] macroPositions = {0, 200, 400, 600};

        // Set PID coefficients for the motor
        //setMotorPIDF(jointMotor, Kp, Ki, Kd, 0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double joint_power = 0;

            // Deadband to address controller drift
            double deadband = 0.1;

            double raw_joint_power = -gamepad1.right_stick_y;

            if (Math.abs(raw_joint_power) < deadband) {
                holding_joint_location = hold_joint ? holding_joint_location : jointMotor.getCurrentPosition();

                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(1); // Set power to move to target position

                /*
                if (!hold_joint) {
                    jointMotor.setTargetPosition(holding_joint_location);
                    jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                 */
                hold_joint = true;
                joint_power = 1;
            } else {
                // Exponential for fine control
                double exponent = 2.0;

                hold_joint = false;
                jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                joint_power = Math.signum(raw_joint_power) * Math.pow(Math.abs(raw_joint_power), exponent) * 0.5;
            }

            // Macros for better PID tuning
            if (gamepad1.a) {
                holding_joint_location = macroPositions[0];
                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(1); // Set power to move to target position
                hold_joint = true;
            } else if (gamepad1.b) {
                holding_joint_location = macroPositions[1];
                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(1); // Set power to move to target position
                hold_joint = true;
            } else if (gamepad1.x) {
                holding_joint_location = macroPositions[2];
                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(1); // Set power to move to target position
                hold_joint = true;
            } else if (gamepad1.y) {
                holding_joint_location = macroPositions[3];
                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setPower(1); // Set power to move to target position
                hold_joint = true;
            }

            // Adjust Kp using gamepad bumpers
            if (gamepad1.right_bumper && Kp <= 1.0) {
                Kp += 0.01; // Increase Kp by 0.001
                sleep(250); // Delay to prevent rapid-fire changes
            } else if (gamepad1.left_bumper && Kp >= 0.001) {
                Kp -= 0.01; // Decrease Kp by 0.001
                sleep(250); // Delay to prevent rapid-fire changes
            }

            /*
            // Update PID coefficients if Kp has changed
            if (prevKp != Kp) {
                setMotorPIDF(jointMotor, Kp, Ki, Kd, 0);
                prevKp = Kp;
            }
             */

            jointMotor.setPower(joint_power);

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

            telemetry.addLine("----- Motor Coefficients -----");
            //telemetry.addData("PIDF Coefficients", jointMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

            telemetry.addData("holding position?", hold_joint);

            telemetry.update();

            sleep(50);
        }
    }

    private void setMotorPIDF(DcMotorEx motor, double Kp, double Ki, double Kd, double F) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }
}
