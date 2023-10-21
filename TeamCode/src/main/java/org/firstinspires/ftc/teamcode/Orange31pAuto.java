//blue alliance
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoCode")
public class Orange31pAuto extends LinearOpMode {
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
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        // Run the motors for the specified duration
        double startTime = getRuntime();



        //LEFT TEAM PROP
       // if(position == Left) {
            while (opModeIsActive() && (getRuntime() - startTime) < 0.3) {
                //goes left
                flMotor.setPower(-1.0); // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(-1.0); // Backward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 1.5) {
                //goes forward to drop pixel with orange piece
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) {
                //goes backward after placing pixel on line
                flMotor.setPower(-1.0);  // Backward
                frMotor.setPower(-1.0);  // Backward
                blMotor.setPower(-1.0);  //Backward
                brMotor.setPower(-1.0); // Backward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.3) {
                //turns left 90 degrees
                flMotor.setPower(-1.0);  // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(-1.0);  //Backward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 1.0) {
                //goes forward to backdrop and parks
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
       // }


        //RIGHT TEAM PROP
        //if(position == Right) {
            while (opModeIsActive() && (getRuntime() - startTime) < 1.7) {
                //goes forward to drop pixel with orange piece
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.3) {
                //rotate right
                flMotor.setPower(1.0); // Forward
                frMotor.setPower(-1.0);  // Backward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(-1.0); // Backward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) {
                //goes forward to drop pixel with orange piece
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 2.5) {
                //goes backward after placing pixel on line and PARKS
                flMotor.setPower(-1.0);  // Backward
                frMotor.setPower(-1.0);  // Backward
                blMotor.setPower(-1.0);  //Backward
                brMotor.setPower(-1.0); // Backward
            }
       // }




        //CENTER TEAM PROP
      //  if(position == center) {
            while (opModeIsActive() && (getRuntime() - startTime) < 0.1) {
                //goes left
                flMotor.setPower(-1.0); // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(-1.0); // Backward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 1.5) {
                //goes forward to drop pixel with orange piece
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) {
                //goes backward after placing pixel on line
                flMotor.setPower(-1.0);  // Backward
                frMotor.setPower(-1.0);  // Backward
                blMotor.setPower(-1.0);  //Backward
                brMotor.setPower(-1.0); // Backward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.3) {
                //turns left 90 degrees
                flMotor.setPower(-1.0);  // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(-1.0);  //Backward
                brMotor.setPower(1.0); // Forward
            }
            while (opModeIsActive() && (getRuntime() - startTime) < 1.0) {
                //goes forward to backdrop and parks
                flMotor.setPower(1.0);  // Forward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  //Forward
                brMotor.setPower(1.0); // Forward
            }
    //    }
        // Stop the motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}
