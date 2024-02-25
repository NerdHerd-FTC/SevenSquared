package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

// Currently only supports left camera - right camera may need to be added separately...
@Config
public class BlueCubeDetectionPipeline implements VisionProcessor {
    private Telemetry telemetry;  // Add Telemetry object
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat morphed = new Mat();

    // Create rectangle zones for the blue cubes
    public float side_width = 240;
    public float side_height = 312;
    public float center_width = 300;
    public float center_height = 330;

    // Set center coordinates for the blue cubes
    public float left_x = 23;
    public float left_y = 91;

    public float center_x = 267;
    public float center_y = 62;

    public float right_x = 572;
    public float right_y = 100;

    public static int lBx = 50;
    public static int lBy  = 40;
    public static int lBz = 50;

    public static int uBx = 130;
    public static int uBy = 190;
    public static int uBz = 190;

    public static Scalar lowerBlue = new Scalar(lBx, lBy, lBz);
    public static Scalar upperBlue = new Scalar(uBx, uBy, uBz);

    public enum Detection {
        LEFT,
        CENTER,
        RIGHT
    }

    Detection detected = Detection.RIGHT;

    public BlueCubeDetectionPipeline(Telemetry telemetry) {  // Constructor to initialize telemetry
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowerBlue, upperBlue, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the zones in green
        Imgproc.rectangle(frame, new Point(center_x, center_y), new Point(center_x + center_width, center_y + center_height), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(frame, new Point(left_x, left_y), new Point(left_x + side_width, left_y + side_height), new Scalar(0,255, 0), 1);

        // Define rectangles for specific zones as before
        Rect leftRect = new Rect((int)left_x, (int)left_y, (int)side_width, (int)side_height);
        Rect centerRect = new Rect((int)center_x, (int)center_y, (int)center_width, (int)center_height);

        List<MatOfPoint> valid_contours = new ArrayList<>();

        // Calculate area within each predefined zone
        double leftArea = calculateAreaInZone(mask, leftRect);
        double centerArea = calculateAreaInZone(mask, centerRect);

        telemetry.addData("Left Area", leftArea);
        telemetry.addData("Center Area", centerArea);

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            double centerX = rect.x + rect.width / 2.0;
            double centerY = rect.y + rect.height / 2.0;

            double area = Imgproc.contourArea(contour);

            // detect location
            if (area < 3000) {
                break;
            } else {
                valid_contours.add(contour);
            }

            if (centerX > center_x && centerX < center_x + center_width && centerY > center_y && centerY < center_y + center_height) {
                telemetry.addData("Location", "Center");
                detected = Detection.CENTER;
            } else if (centerX > left_x && centerX < left_x + side_width && centerY > left_y && centerY < left_y + side_height) {
                telemetry.addData("Location", "Left");
                detected = Detection.LEFT;
            } else {
                telemetry.addData("Location (Assumed)", "Right");
                detected = Detection.RIGHT;
            }

            // Telemetry data for the area of each blue object
            telemetry.addData("Area", area);
            telemetry.update();

            Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
        }

        if (leftArea > 8000) {
            telemetry.addData("Location", "Left (WHOLE)");
            detected = Detection.LEFT;

            if (centerArea > leftArea) {
                detected = Detection.CENTER;
            }
        } else if (centerArea > 8000) {
            telemetry.addData("Location", "Center (WHOLE)");
            detected = Detection.CENTER;
        } else if (valid_contours.size() < 1) {
            detected = Detection.RIGHT;
            telemetry.addData("Location (Assumed)", "Right");
        }

        telemetry.update();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Detection getDetection() {
        return detected;
    }

    private double calculateAreaInZone(Mat mask, Rect zone) {
        Mat zoneMask = mask.submat(zone);
        return Core.countNonZero(zoneMask);
    }
}