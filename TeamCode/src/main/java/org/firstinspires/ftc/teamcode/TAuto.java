//Written By Derrick
//Red Alliance Auto
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoCode")
public class TAuto extends LinearOpMode {
    // Define motor variables
    private final DcMotor flMotor = hardwareMap.dcMotor.get("frontLeft");
    private final DcMotor frMotor = hardwareMap.dcMotor.get("frontRight");
    private final DcMotor blMotor = hardwareMap.dcMotor.get("backLeft");
    private final DcMotor brMotor = hardwareMap.dcMotor.get("backRight");
    private final DcMotor jointMotor = hardwareMap.dcMotor.get("joint");
    private final DcMotor armMotor = hardwareMap.dcMotor.get("arm");
    static final double COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    static final double WHEEL_DIAMETER_INCH = 3.65;    // For figuring circumference: 90mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * Math.PI);

    static final double ARM_POWER = 0.65; //for quick adjustments

    static final int HIGH_JUNCTION_TICKS = 740;
    static final int MEDIUM_JUNCTION_TICKS = 420;
    static final int LOW_JUNCTION_TICKS = 290;

    final double turnAngle = -58;
    final int inchAdvance = 3;
    int coneStack = 0; //know how high to reach to get the next cone

    static final double clawOpen = 0.5;
    static final double clawClose = 0.1;

    OpenCvCamera camera;
    PropDetectionPipeline propDetectionPipeline;

    public void runOpMode() {


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jointMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        CRServo DroneServo = hardwareMap.get(CRServo.class, "DS");
        Servo WristServo = hardwareMap.get(Servo.class, "WS");

        //core hex motors are facing opposite each other and will rotate in opposite directions
        jointMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        jointMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.Fo);

        DR4BMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DR4BMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DR4BMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DR4BMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DR4BMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DR4BMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(20);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("CAMERA ERROR=%d", errorCode);
            }
        });

        // Set motor directions


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
    private void wheels(int position, int cpower){
        flMotor.getCurrentPosition();
        frMotor.getCurrentPosition();
        blMotor.getCurrentPosition();
        brMotor.getCurrentPosition();





    }
}
