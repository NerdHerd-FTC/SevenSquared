package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Testing")
@Disabled
public class ArmTesting extends LinearOpMode {
    public Servo servo1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("joint");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean hold_arm = false;
        boolean hold_joint = false;

        int jointPosition = 0;
        int armPosition = 0;

        double armError = 0;
        int jointError = 0;

        double armKp = 0.0001;
        double jointKp = 0.001;

        int holding_arm_location = 0;
        int holding_joint_location = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double arm_power = 0;
            double joint_power = 0;

            // Deadband to address controller drift
            double deadband = 0.1;

            double raw_arm_power = -gamepad1.left_stick_y;
            double raw_joint_power = -gamepad1.right_stick_y;

            // Apply deadband
            if (Math.abs(raw_arm_power) < deadband) {
                raw_arm_power = 0;
                holding_arm_location = hold_arm ? holding_arm_location: armMotor.getCurrentPosition();
                hold_arm = true;
            } else {
                hold_arm = false;
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (Math.abs(raw_joint_power) < deadband) {
                raw_joint_power = 0;
                holding_joint_location = hold_joint ? holding_joint_location : jointMotor.getCurrentPosition();
                hold_joint = true;
            } else {
                hold_joint = false;
                jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Exponential for fine control
            double exponent = 2.0;
            if (!hold_arm) {
                arm_power = Math.signum(raw_arm_power) * Math.pow(Math.abs(raw_arm_power), exponent) * 0.25;
            }
            if (!hold_joint) {
                joint_power = Math.signum(raw_joint_power) * Math.pow(Math.abs(raw_joint_power), exponent) * 0.25;
            }

            if (gamepad1.y) {
                jointKp += 0.001;
            } else if (gamepad1.a) {
                jointKp -= 0.001;
            } else if (gamepad1.x) {
                armKp -= 0.0001;
            } else if (gamepad1.b) {
                armKp += 0.0001;
            }

            // CONNECT ENCODERS BEFORE ENABLING THIS
            if (hold_arm) {
                armMotor.setTargetPosition(holding_arm_location);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_power = hold_position_power(armPosition, holding_arm_location, true, armKp);
            }

            if (hold_joint) {
                jointMotor.setTargetPosition(holding_joint_location);
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointError = jointMotor.getTargetPosition() - jointMotor.getCurrentPosition();
                joint_power = hold_position_power(jointPosition, holding_joint_location, false, jointKp);
            }

            /*
            if (armMotor.getCurrentPosition() >= 680 && armMotor.getPower()) {

            }

             */

            jointMotor.setPower(joint_power);
            armMotor.setPower(arm_power * 0.5);

            telemetry.addLine("----- JOINT MOTOR -----");
            telemetry.addData("joint power", jointMotor.getPower());
            telemetry.addData("joint position", jointPosition);
            telemetry.addData("joint target", jointMotor.getTargetPosition());
            telemetry.addData("joint error", jointError);
            telemetry.addData("holding joint", hold_joint);

            telemetry.addLine("----- ARM MOTOR -----");
            telemetry.addData("arm power", armMotor.getPower());
            telemetry.addData("arm position", armPosition);
            telemetry.addData("arm target", armMotor.getTargetPosition());
            telemetry.addData("arm error", armError);
            telemetry.addData("holding arm", hold_arm);

            telemetry.update();

            sleep(50);
        }
    }

    private double hold_position_power(double current_position, double target_position, boolean isArm, double armP){
        if (!isArm) {
            double armError = target_position - current_position;
            return armError*armP;
        } else {
            double armError = target_position - current_position;
            return armError*armP;
        }
    }
}