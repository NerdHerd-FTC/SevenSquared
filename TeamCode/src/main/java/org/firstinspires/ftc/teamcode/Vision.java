package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.tensorflow.lite.support.common.Processor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name= "Vision", group="Roboto")
public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)// Draws on the tag where axis are pointing
                .setDrawCubeProjection(true)// Draws a cube projected off of the tag
                .setDrawTagID(true)// Draws out tag id number
                .setDrawTagOutline(true) //outlines ta
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .build();


        waitForStart();

        while (! isStopRequested() && opModeIsActive()){


            if(tagProcessor.getDetections().size() > 0){ //This makes sure the processor see's atleast 1 tag
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("x", tag.ftcPose.yaw);
                telemetry.addData("range", tag.ftcPose.range);



//By using this you will be able to get the x y z Pitch x rotation Roll y rotation Yaw is z rotation
//Range uses py theorm to get distance  Bearing how much it needs to rotate side to side  to be center of tag Elevation is rotate up and down
            }
        }

        telemetry.update();

    }
}