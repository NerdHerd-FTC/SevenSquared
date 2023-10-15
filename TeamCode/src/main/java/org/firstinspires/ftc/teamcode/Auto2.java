//Written By Derrick FTC 49
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FreightFenzy_REDAuton1 (Java)")
public class Auto2 extends LinearOpMode {

    private DcMotor RightDrive;
    private DcMotor LeftDrive;


    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Drive function with 3 parameters
    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = RightDrive.getCurrentPosition() + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = LeftDrive.getCurrentPosition() + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            LeftDrive.setTargetPosition(leftTarget);
            RightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            LeftDrive.setPower(power);
            RightDrive.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (LeftDrive.isBusy() || RightDrive.isBusy())) {
            }

            // set motor power back to 0
            LeftDrive.setPower(0);
            RightDrive.setPower(0);
        }
    }


    @Override
    public void runOpMode() {

        RightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        LeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");





        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive() && runtime.seconds() <= 2) {

                drive(1, 12, 0);
            }
        }
    }
}