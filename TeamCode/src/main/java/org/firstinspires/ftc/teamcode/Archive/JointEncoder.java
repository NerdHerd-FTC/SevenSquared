package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Joint Encoder")
@Disabled
public class JointEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Get servos
        DcMotor jointMotor = hardwareMap.dcMotor.get("frontLeft");

        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jointMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        jointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean done = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (!done) {
                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                jointMotor.setTargetPosition(200);
                done = true;
            }
            jointMotor.setPower(1);

            telemetry.addData("Encoder location", jointMotor.getCurrentPosition());
            telemetry.addData("done", done);
            telemetry.addData("target position", jointMotor.getTargetPosition());
            telemetry.update();

            sleep(50);
        }
    }

    //-621, 0
    // -600
}