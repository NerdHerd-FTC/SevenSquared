package org.firstinspires.ftc.teamcode.Archive;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "DC Testing")
@Disabled
public class DCTest extends LinearOpMode {

    enum DroneCoverState {
        closing,
        opening,
        idle
    }


    public static double DC_pow = 0;
    private ElapsedTime DroneActive = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        CRServo DroneCover = hardwareMap.get(CRServo.class, "DC");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            DroneCover.setPower(DC_pow);

            telemetry.addData("DC Position", DC_pow);
            telemetry.update();
        }
    }
}