// Code Created By Derrick, Owen, Shassh
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.TeleUtil;

@Config
@TeleOp
public class PIDF_Arm_Joint extends LinearOpMode {
    private PIDController jointPID;
    private PIDController armPID;

    public static double jointP = 0.00009, jointI = 0.0, jointD = 0.0, jointF = -0.000047;
    public static double armP = 0.0035, armI = 0.0, armD = 0.0003, armF = 0.002;

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
            armPID.setPID(armP, armI, armD);
            jointPID.setPID(jointP, jointI, jointD);

            double joint_out = jointPID.calculate(jointMotor.getCurrentPosition(), jointTarget);
            double arm_out = armPID.calculate(armMotor.getCurrentPosition(), armTarget);

            double joint_angle = jointMotor.getCurrentPosition() / joint_ticks_per_degree + 193;
            double relative_arm_angle = armMotor.getCurrentPosition() / arm_ticks_per_degree + 29;
            double arm_angle =270 - relative_arm_angle - joint_angle;

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
            double arm_part_of_joint = Math.cos(Math.toRadians(arm_angle - 29)) * 6.207;
            if (arm_part_of_joint < 0) {
                arm_part_of_joint = 0;
            }

            if (joint_angle > 180) {
                joint_angle = 180;
            }

            double joint_ff = (-Math.cos(Math.toRadians(joint_angle)) * 3.793 + arm_part_of_joint) * jointF;
            double arm_ff = Math.cos(Math.toRadians(arm_angle)) * armF;

            double joint_power = joint_out + joint_ff;
            double arm_power = arm_out + arm_ff;

            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                joint_power = gamepad1.left_stick_y * 0.7;
            }

            jointMotor.setPower(Math.tanh(joint_power));
            armMotor.setPower(Math.tanh(arm_power));

            telemetry.addData("Joint Position", jointMotor.getCurrentPosition());
            telemetry.addData("Joint Target", jointTarget);
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armTarget);

            telemetry.addData("Joint Angle", joint_angle);
            telemetry.addData("Arm Angle", arm_angle);
            telemetry.addData("Relative Arm Angle", relative_arm_angle);
            telemetry.addData("Joint Power", jointMotor.getPower());

            telemetry.update();
        }

    }
}