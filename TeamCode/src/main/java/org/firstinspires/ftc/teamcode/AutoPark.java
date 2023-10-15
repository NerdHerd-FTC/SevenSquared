package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "FreightFenzy_REDAuton1 (Java)")
public class AutoPark extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        waitForStart();

        if (opModeIsActive()) {
            // Move left for 12 inches
            mecanumDrive(1, -12, -12, -12, -12);
        }
    }

    private void mecanumDrive(double power, double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches) {
        int frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget;

        if (opModeIsActive()) {
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * DRIVE_COUNTS_PER_IN);
            frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * DRIVE_COUNTS_PER_IN);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * DRIVE_COUNTS_PER_IN);
            backRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * DRIVE_COUNTS_PER_IN);

            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
                // Wait until all motors are no longer busy running to position
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
}
