package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//https://github.com/NerdHerd-FTC/CAMS-FTC/blob/develop_Jadon/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumControlDriverOriented.java
@TeleOp(name = "Arm Control Driver Oriented - SIM")
public class ArmControlDriverOriented extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors (F=front, B=back, R=right, L=left)
        DcMotor motorFL = hardwareMap.dcMotor.get("frontLeft");



        // Right motors should move in reverse
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double ArmPower = 0;
            double y = gamepad1.left_trigger; // Y stick is reversed
            double x = gamepad1.right_trigger; // Counteract imperfect strafing
                if(gamepad1.right_trigger >= 0.4){
                    ArmPower = 0.1;
                }
                else if(gamepad1.left_trigger >= 0.4){
                    ArmPower = -0.1;
                }

                motorFL.setPower(ArmPower);

                telemetry.update();

                    // Pace this loop so jaw action is reasonable speed.
                sleep(50);



            telemetry.addData("Arm Encoder", motorFL.getCurrentPosition());
        }
    }
}