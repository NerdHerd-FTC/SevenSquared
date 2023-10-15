package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FreightFenzy_REDAuton1 (Java)")
public class AutoPark extends LinearOpMode {

    private DcMotor RightDrive;
    private DcMotor LeftDrive;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private ElapsedTime runtime = new ElapsedTime();

    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            rightTarget = RightDrive.getCurrentPosition() + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = LeftDrive.getCurrentPosition() + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            LeftDrive.setTargetPosition(leftTarget);
            RightDrive.setTargetPosition(rightTarget);

            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftDrive.setPower(power);
            RightDrive.setPower(power);

            while (opModeIsActive() && (LeftDrive.isBusy() || RightDrive.isBusy())) {
                // Wait until both motors are no longer busy running to position
            }

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
            // Move left for 12 inches
            drive(1, -12, -12);
        }
    }
}
