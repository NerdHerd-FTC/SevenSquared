package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.armD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.armP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointI;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jointP;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_norm_F;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.joint_ticks_per_degree;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotConstants.*;

@Config
@TeleOp
public class PIDF_Arm_Joint extends LinearOpMode {
    private PIDController jointPID;
    private PIDController armPID;

    public static int jointTarget = 0, armTarget = 0;

    private final double joint_ticks_per_degree = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28 * 10.0/3) / 360.0;
    private final double arm_ticks_per_degree = ((((1+(46.0/17))) * (1+(46.0/11))) * 28 * 5)/360.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // REMOVE THESE BEFORE COMPETITION
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        jointPID = new PIDController(jointP, jointI, jointD);
        armPID = new PIDController(armP, armI, armD);

        // Get motors
        DcMotor jointMotor = hardwareMap.dcMotor.get("joint");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setDirection(DcMotor.Direction.REVERSE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // set PID values every loop to account for dashboard changes
            armPID.setPID(armP, armI, armD); // working values: 0.0035, 0.0, 0.0003 w/ f = 0.002
            jointPID.setPID(jointP, jointI, jointD);

            // calculate PID power
            double joint_out = jointPID.calculate(jointMotor.getCurrentPosition(), jointTarget);
            double arm_out = armPID.calculate(armMotor.getCurrentPosition(), armTarget);

            // calculate angles of joint & arm (in degrees) to account for torque
            double joint_angle = jointMotor.getCurrentPosition() / joint_ticks_per_degree + 193;
            double relative_arm_angle = armMotor.getCurrentPosition() / RobotConstants.arm_ticks_per_degree + 14.8;
            double arm_angle = 270 - relative_arm_angle - joint_angle;

            // account for negative angles
            if (arm_angle > 360) {
                arm_angle -= 360;
            } else if (joint_angle < 0) {
                joint_angle += 360;
            }

            if (joint_angle > 360) {
                joint_angle -= 360;
            } else if (joint_angle < 0) {
                joint_angle += 360;
            }

            // joint self-torque: 16.2917713642 lb. in.
            // arm self-torque: 12.9580463969 lb. in.

            // calculate effective torque arm length - if arm is on the same side as the joint, use arm length with joint, otherwise use arm length as cog since it's pretty close and find additional torque...
            double joint_ff = joint_norm_F * Math.cos(Math.toRadians(joint_angle));

            // calculate feedforward with torque weights and cosine
            double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

            // calculate total power
            double joint_power = joint_out + joint_ff;
            double arm_power = arm_out + arm_ff;

            // allow for manual override for tuning purposes
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                joint_power = gamepad1.left_stick_y * 0.7;
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                arm_power = gamepad1.right_stick_y * 0.7;
            }

            // set motor powers with tanh to normalize
            jointMotor.setPower(Math.tanh(joint_power));
            armMotor.setPower(Math.tanh(arm_power));

            telemetry.addData("Joint Position", jointMotor.getCurrentPosition());
            telemetry.addData("Joint Target", jointTarget);
            telemetry.addData("Joint Power", jointMotor.getPower());
            telemetry.addData("Joint FF", joint_ff);

            telemetry.addLine("\n");

            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armTarget);

            telemetry.addLine("\n");

            telemetry.addData("Joint Angle", joint_angle);
            telemetry.addData("Arm Angle", arm_angle);
            telemetry.addData("Relative Arm Angle", relative_arm_angle);

            telemetry.update();
        }
    }
}