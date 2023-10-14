package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Testing")
public class Claw extends LinearOpMode {
    public Servo servo1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor armMotor = hardwareMap.dcMotor.get("backLeft");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int Clawjoint = 0;
        int Clawposition = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Deadband to address controller drift
            double deadband = 0.1;

            double raw_arm_power = gamepad1.left_stick_y;
            double raw_joint_power = gamepad1.right_stick_y;

            // Apply deadband
            if (Math.abs(raw_arm_power) < deadband) {
                raw_arm_power = 0;
            }
            if (Math.abs(raw_joint_power) < deadband) {
                raw_joint_power = 0;
            }

            // Exponential for fine control
            double exponent = 2.0;
            double arm_power = Math.signum(raw_arm_power) * Math.pow(Math.abs(raw_arm_power), exponent) * 0.5;
            double joint_power = Math.signum(raw_joint_power) * Math.pow(Math.abs(raw_joint_power), exponent) * 0.5;

            // CONNECT ENCODERS BEFORE ENABLING THIS
            /*
            if (arm_power == 0) {
                armMotor.setTargetPosition(armPosition);
            }

            if (joint_power == 0) {
                jointMotor.setTargetPosition(jointPosition);
            }
            */

            Clawposition = armMotor.getCurrentPosition();
            Clawjoint = jointMotor.getCurrentPosition();

            jointMotor.setPower(joint_power);
            armMotor.setPower(arm_power);

            telemetry.addData("jointMotor", jointMotor.getPower());
            telemetry.addData("armMotor", armMotor.getPower());

            telemetry.addData("jointMotor", Clawjoint);
            telemetry.addData("armMotor", Clawposition);

            telemetry.update();
        }
    }
}