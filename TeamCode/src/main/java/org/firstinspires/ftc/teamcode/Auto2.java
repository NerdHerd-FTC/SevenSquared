// Written By Derrick
// Blue Alliance Auto
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoCode2")
public class Auto2 extends LinearOpMode {
    // Define motor variables
    private final DcMotor flMotor = hardwareMap.dcMotor.get("frontLeft");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("frontRight");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("backLeft");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("backRight");

    // Define the duration of movement
    private final double movementDuration = 2.5; // 2.5 seconds

    @Override
    public void runOpMode() {
        // Set motor directions
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Run the motors for the specified duration
        double startTime = getRuntime();
        while (opModeIsActive() && (getRuntime() - startTime) < movementDuration) {
            flMotor.setPower(-1.0); // Backward
            frMotor.setPower(1.0);  // Forward
            blMotor.setPower(1.0);  // Forward
            brMotor.setPower(-1.0); // Backward
        }

        // Stop the motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}
