package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Simple")
public class SuperDangSimple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotorEx jointMotor = (DcMotorEx) hardwareMap.dcMotor.get("joint");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean done = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double input = -gamepad1.left_stick_y;

            jointMotor.setPower(input * 0.5);

            telemetry.addData("Encoder location", jointMotor.getCurrentPosition());
            telemetry.addData("Power", jointMotor.getPower());
            telemetry.addData("done", done);
            telemetry.addData("target position", jointMotor.getTargetPosition());
            telemetry.update();

            sleep(50);
        }
    }
}
//-621, 0
// -600
