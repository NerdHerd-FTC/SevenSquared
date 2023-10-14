// Code Created By Derrick, Owen, Shash
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@TeleOp(name = "Joint Testing - Simple")
public class JointTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor armMotor = hardwareMap.dcMotor.get("backLeft");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jointMotor.setDirection(DcMotor.Direction.REVERSE);
        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        CRServo FrontRollerServoRight = hardwareMap.get(CRServo.class, "Servo1");
        CRServo FrontRollerServoLeft= hardwareMap.get(CRServo.class, "Servo2");
        CRServo ArmServoRight = hardwareMap.get(CRServo.class, "Servo3");
        CRServo ArmServoLeft = hardwareMap.get(CRServo.class, "Servo4");
        Servo DroneServo = hardwareMap.get(Servo.class, "Servo 5");
        Servo WristServo = hardwareMap.get(Servo.class, "Servo 6");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            jointMotor.setPower(setJointPower(jointMotor, gamepad1));
            armMotor.setPower(setArmPower(armMotor, gamepad1));

            checkGamepadParameters(gamepad1, "Driver");
            telemetry.addLine("\n");
            //checkGamepadParameters(gamepad2, "Operator");

            motorTelemetry(jointMotor, "Joint");
            telemetry.addLine("\n");
            motorTelemetry(armMotor, "Arm");

            telemetry.update();
            sleep(50);
        }
    }

    private double setJointPower(DcMotor jointMotor, Gamepad gamepad) {
        double power;
        double mult = 0.5;
        double input = -gamepad1.left_stick_y;

        if (jointMotor.getCurrentPosition() <= 0 && input < 0) {
            power = 0;
        } else {
            power = input*mult;
        }

        return power;
    }
    private double setRollerPowerRight(CRServo FrontRollerServoRight, Gamepad gamepad) {
        double power = 0;

        if(gamepad.right_trigger != 0 )  {
           power = 0.75;
        }

        return power;

    }
    private double setRollerPowerLeft(CRServo FrontRollerServoLeft, Gamepad gamepad) {
        double power = 0;

        if(gamepad.left_trigger != 0 )  {
            power = -0.75;
        }

        return power;

    }
    private double setArmServoPowerRight(CRServo ArmServoPowerRight, Gamepad gamepad) {
        double power = 0;

        if(!gamepad.left_bumper)  {
            power = 0.75;
        }
        else if(gamepad.left_bumper){
            power = -0.75;
        }

        return power;

    }
    private double setArmServoPowerLeft(CRServo ArmServoPowerLeft, Gamepad gamepad) {
        double power = 0;

        if(!gamepad.right_bumper)  {
            power = 0.75;
        }
        else if(gamepad.left_bumper){
            power = -0.75;
        }

        return power;

    }
    private double setDroneServoPower(Servo DroneServoPower, Gamepad gamepad) {
        double power = 0;

        if(gamepad.y)  {
            power = 1;
        }

        return power;

    }
    private double setWristServoPower(Servo WristServoPower, Gamepad gamepad){
        double power = 0;

        if(gamepad.dpad_up){
            power = 0.1;
        }
        if(gamepad.dpad_down){
            power = -0.1;
        }
        return power;
    }

    private double setArmPower(DcMotor jointMotor, Gamepad gamepad) {
        double power;
        double mult = 0.25;
        double input = -gamepad1.right_stick_y;

        if (jointMotor.getCurrentPosition() <= 0 && input < 0) {
            power = 0;
        } else {
            power = input*mult;
        }

        return power;
    }

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


            if(gamepad.left_trigger != 0){
                telemetry.addData("Left Trigger", gamepad.left_trigger);
            }
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

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }
    //-621, 0
    // -600
}