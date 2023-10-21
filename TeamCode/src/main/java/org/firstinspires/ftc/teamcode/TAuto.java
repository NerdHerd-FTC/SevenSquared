//Written By Derrick
//Red Alliance Auto
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoCode")
public class TAuto extends LinearOpMode {
    // Define motor variables
    private final DcMotor flMotor = hardwareMap.dcMotor.get("frontLeft");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("frontRight");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("backLeft");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("backRight");
    private final DcMotor jointMotor = hardwareMap.dcMotor.get("joint");
    private final DcMotor armMotor = hardwareMap.dcMotor.get("arm");

    // Define the duration of movement
    // 2.5 seconds

    @Override
    public void runOpMode() {
        // Set motor directions
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        // Run the motors for the specified duration
        //CENTER CODE
        double startTime = getRuntime();
   //     if(Position == Center) {
            while (opModeIsActive() && (getRuntime() - startTime) < 1) { //FORWARD
                flMotor.setPower(1.0); // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(1.0); // Backward

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                jointMotor.setPower(1.0);
                armMotor.setPower(1.0);

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                jointMotor.setPower(-1.0);
                armMotor.setPower(-1.0);

            }

            while (opModeIsActive() && (getRuntime() - startTime) < 2.5) { //STRAFE LEFT
                flMotor.setPower(-1.0); // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(-1.0); // Backward

     //       }
            // ----------------------------------------------------------------------------------------------------------------
            //LeftCode
          //  if (Position == Left) {
                while (opModeIsActive() && (getRuntime() - startTime) < 1) { //STRAFE LEFT
                    flMotor.setPower(-1.0); // Backward
                    frMotor.setPower(1.0);  // Forward
                    blMotor.setPower(1.0);  // Forward
                    brMotor.setPower(-1.0); // Backward

                }
                while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { //FORWARD
                    flMotor.setPower(1.0); // Backward
                    frMotor.setPower(1.0);  // Forward
                    blMotor.setPower(1.0);  // Forward
                    brMotor.setPower(1.0); // Backward

                }
                while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                    jointMotor.setPower(1.0);
                    armMotor.setPower(1.0);

                }
                while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                    jointMotor.setPower(-1.0);
                    armMotor.setPower(-1.0);

                }
                while (opModeIsActive() && (getRuntime() - startTime) < 2) { //STRAFE LEFT
                    flMotor.setPower(-1.0); // Backward
                    frMotor.setPower(1.0);  // Forward
                    blMotor.setPower(1.0);  // Forward
                    brMotor.setPower(-1.0); // Backward

                }
            }
    //    }

// ----------------------------------------------------------------------------------------------------------------
        //RightCode
        //if(position == Right) {
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Forward
                flMotor.setPower(1.0); // Backward
                frMotor.setPower(1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(1.0); // Backward

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Rotate
                flMotor.setPower(1.0); // Backward
                frMotor.setPower(-1.0);  // Forward
                blMotor.setPower(1.0);  // Forward
                brMotor.setPower(-1.0); // Backward

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                jointMotor.setPower(1.0);
                armMotor.setPower(1.0);

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 0.5) { // Put Down Pixel
                jointMotor.setPower(-1.0);
                armMotor.setPower(-1.0);

            }
            while (opModeIsActive() && (getRuntime() - startTime) < 2) { //MoveBack
                flMotor.setPower(-1.0); // Backward
                frMotor.setPower(-1.0);  // Forward
                blMotor.setPower(-1.0);  // Forward
                brMotor.setPower(-1.0); // Backward

            }
      //  }

        // Stop the motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}
