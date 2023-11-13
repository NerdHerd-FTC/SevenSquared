// Code Created By Derrick, Owen, Shash
package org.firstinspires.ftc.teamcode;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Austin Drive")
public class AustinDrive extends LinearOpMode {
    private ElapsedTime CSR = new ElapsedTime();
    private ElapsedTime CSL = new ElapsedTime();
    private ElapsedTime matchTime = new ElapsedTime();

    // Servo info
    boolean fr_closed = true;
    boolean fl_closed = true;

    boolean arm_macro = false;
    double arm_target;

    boolean joint_macro = false;
    double joint_target;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Drive control variables
        boolean exponential_drive = true;
        boolean slowdown = false;

        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBL = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFR = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBR = hardwareMap.dcMotor.get("backRight");

        // Unlock full speed of drive motors
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Right motors should move in reverse
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Left motors should move forward
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams;

        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        // Technically this is the default, however specifying it is clearer
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(imuParams);

        // Get servos
        Servo ClawServoRight = hardwareMap.get(Servo.class, "CSR");
        Servo ClawServoLeft = hardwareMap.get(Servo.class, "CSL");
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");
        Servo WristServo = hardwareMap.get(Servo.class, "WS");


        // Reverse if opposite directions are seen
        ClawServoRight.setDirection(Servo.Direction.REVERSE);
        ClawServoLeft.setDirection(Servo.Direction.REVERSE);
        DroneServo.setDirection(DcMotorSimple.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        matchTime.reset();
        CSR.reset();
        CSL.reset();
        WristServo.setPosition(0);


        while (opModeIsActive()) {
            // Get yaw
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Drive
            fieldOrientedDrive(yaw, gamepad1, exponential_drive, slowdown, motorFL, motorBL, motorFR, motorBR);

            // Articulation
            jointMotor.setPower(setJointPower(jointMotor, gamepad2));
            armMotor.setPower(setArmPower(armMotor, gamepad2));

            setClawServoRight(ClawServoRight, gamepad2, 0,1);
            setClawServoLeft(ClawServoLeft, gamepad2,0.6,0);

            setWristServoPower(WristServo, gamepad2);

            activateDroneLauncher(DroneServo, gamepad2);

            // Reset yaw to 0 7
            if (gamepad1.x) {
                imu.resetYaw();
                telemetry.addLine("IMU reset");
            }

            // Gyro Telemetry
            gyroTelemetry(yaw);
            telemetry.addLine("\n");

            // Gamepad Telemetry
            checkGamepadParameters(gamepad1, "Driver");
            checkGamepadParameters(gamepad2, "Operator");
            telemetry.addLine("\n");

            // Motor Telemetry
            motorTelemetry(jointMotor, "Joint");
            telemetry.addLine("\n");
            motorTelemetry(armMotor, "Arm");
            telemetry.addLine("\n");

            // Servo Telemetry
            servoTelemetry(WristServo, ClawServoRight, ClawServoLeft);

            // Timers
            telemetry.addData("Match Time", matchTime.seconds());
            telemetry.update();
            sleep(50);
        }
    }

    private void fieldOrientedDrive(double heading, Gamepad gamepad, boolean exponential_drive, boolean slowdown, DcMotor motorFL, DcMotor motorBL, DcMotor motorFR, DcMotor motorBR) {
        double y_raw = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x_raw = gamepad1.left_stick_x;
        double rx_raw = gamepad1.right_stick_x;

        // Deadband to address controller drift
        double deadband = 0.1;
        if (Math.abs(y_raw) < deadband) {
            y_raw = 0;
        }
        if (Math.abs(x_raw) < deadband) {
            x_raw = 0;
        }
        if (Math.abs(rx_raw) < deadband) {
            rx_raw = 0;
        }

        // Toggle exponential drive
        /*
        if (gamepad1.y) {
            exponential_drive = !exponential_drive;
        }
         */

        // Toggle slowdown
        /*if (gamepad1.a) {
            slowdown = !slowdown;
        }
         */

        // Exponential Drive
        double exponent = 2.0;
        double y = exponential_drive ? Math.signum(y_raw) * Math.pow(Math.abs(y_raw), exponent) : y_raw;
        double x = exponential_drive ? Math.signum(x_raw) * Math.pow(Math.abs(x_raw), exponent) : x_raw;
        double rx = exponential_drive ? Math.signum(rx_raw) * Math.pow(Math.abs(rx_raw), exponent) : rx_raw;

        // Slowdown
        if (slowdown) {
            y *= 0.75;
            x *= 0.75;
            rx *= 0.75;
        }

        // get heading from IMU
        double botHeading;

        // restrict range to [0, 360]
        if (heading <= 0) {
            botHeading = heading + 2 * Math.PI;
        } else {
            botHeading = heading;
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFL.setPower(frontLeftPower);
        motorBL.setPower(backLeftPower);
        motorFR.setPower(frontRightPower);
        motorBR.setPower(backRightPower);

        motorTelemetry(motorFL, "FL");
        motorTelemetry(motorBL, "BL");
        motorTelemetry(motorFR, "FR");
        motorTelemetry(motorBR, "BR");
    }



    // SERVO METHODS
    private void setClawServoLeft(Servo ClawServoLeft, Gamepad gamepad, double closed_position, double open_position) {
        double position;
        if (fl_closed) {
            position = closed_position;
        } else {
            position = open_position;
        }

        if(gamepad.left_bumper&& CSL.seconds() > 0.5)  {
            if (fl_closed){
                position = open_position;
                fl_closed = false;
            }
            else {
                position = closed_position;
                fl_closed = true;
            }
            CSL.reset();
        }

        ClawServoLeft.setPosition(position);
    }

    private void setClawServoRight(Servo ClawServoRight, Gamepad gamepad, double closed_position, double open_position) {
        double position;
        if (fr_closed) {
            position = closed_position;
        } else {
            position = open_position;
        }

        if(gamepad.right_bumper && CSR.seconds() > 0.5)  {
            if (fr_closed){
                position = open_position;
                fr_closed = false;
            }
            else {
                position = closed_position;
                fr_closed = true;
            }
            CSR.reset();
        }

        ClawServoRight.setPosition(position);
    }

    private void activateDroneLauncher(CRServo DroneServo, Gamepad gamepad) {
        double power = 0;
        if(gamepad.a && matchTime.seconds() > 0)  {
            power = 1;
        }

        DroneServo.setPower(power);
    }
    private void setWristServoPower(Servo WristServo, Gamepad gamepad){
        double position = WristServo.getPosition();

        if(gamepad.dpad_up){
            position += 0.05;
        }
        else if(gamepad.dpad_down){
            position -= 0.05;
        }

        WristServo.setPosition(position);
    }

    // ARM AND JOINT MOTOR METHODS
    private double setJointPower(DcMotor jointMotor, Gamepad gamepad) {
        double power = 0;
        double mult = 1;

        if (!joint_macro && gamepad.b) {
            joint_macro = true;

            jointMotor.setTargetPosition(-1000); // TUNE THE HECK OUT OF THIS POR FAVOR
            jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (Math.abs(gamepad.left_stick_y) > 0.1) {
            joint_macro = false;
            jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double input = -gamepad.left_stick_y;

            power = input*mult;
        } else if (joint_macro) {
            if (jointMotor.getCurrentPosition() <= -1000) {
                jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                joint_macro = false;
                power = 0;
            } else {
                power = 1;
            }
        }

        return power;
    }
    private double setArmPower(DcMotor armMotor, Gamepad gamepad) {
        double power = 0;
        double mult = 0.7;

        if (!arm_macro && gamepad.a) {
            arm_macro = true;

            armMotor.setTargetPosition(-1000); // TUNE THE HECK OUT OF THIS POR FAVOR
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (Math.abs(gamepad.right_stick_y) > 0.1) {
            arm_macro = false;
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double input = -gamepad.right_stick_y;
            power = input*mult;
        } else if (arm_macro) {
            if (armMotor.getCurrentPosition() <= -1000) {
                arm_macro = false;
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                power = 0;
            } else {
                power = 0.5;
            }
        }

        return power;
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

        if (gamepad.a) {
            telemetry.addLine("A pressed");
        }
        if (gamepad.b) {
            telemetry.addLine("B pressed");
        }
        if (gamepad.x) {
            telemetry.addLine("X pressed");
        }
        if (gamepad.y) {
            telemetry.addLine("Y pressed");
        }

        if (gamepad.left_bumper) {
            telemetry.addLine("Left bumper clicked");
        }
        if (gamepad.right_bumper) {
            telemetry.addLine("Right bumper clicked");
        }
    }

    private void motorTelemetry(DcMotor motor, String name) {
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData(name + " Power", motor.getPower());
        telemetry.addData(name + " Position", motor.getCurrentPosition());
        telemetry.addData(name + " Target Position", motor.getTargetPosition());
    }

    private void servoTelemetry(Servo wrist, Servo FrontRight, Servo FrontLeft) {
        telemetry.addLine("--- Servo ---");
        telemetry.addData("FrontRight Closed", fr_closed);
        telemetry.addData("Front Right Location", FrontRight.getPosition());
        telemetry.addData("FrontLeft Closed", fl_closed);
        telemetry.addData("Front Left Location", FrontLeft.getPosition());
        telemetry.addData("CSR Timer", CSR.seconds());
        telemetry.addData("CSL Timer", CSL.seconds());
        telemetry.addData("Wrist", wrist.getPosition());
    }

    private void gyroTelemetry(double heading) {
        telemetry.addLine("--- Gyro ---");
        if (heading == Double.doubleToLongBits(-0.0)) {
            telemetry.addLine("!!! Heading: -0.0 !!!");
            telemetry.addLine("\n\n\n\n\n\n\n\n\n\n");
        }
        telemetry.addData("Heading", heading);
    }

    // still in progress
    private void turnToHeading(double targetHeading, double currentHeading, double max_power, double sleepTime) {
        double error = targetHeading - currentHeading;

        // PID
        double p = error * 0.01;
        double i = 0;
        double d = 0;

        double power = p + i + d;

        if (power > max_power) {
            power = max_power;
        } else if (power < -max_power) {
            power = -max_power;
        }
    }
}