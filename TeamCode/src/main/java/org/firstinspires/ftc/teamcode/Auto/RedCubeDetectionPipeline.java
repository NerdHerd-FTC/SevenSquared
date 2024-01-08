package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

// Currently only supports left camera - right camera may need to be added separately...
public class RedCubeDetectionPipeline implements VisionProcessor {
    private Telemetry telemetry;  // Add Telemetry object
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat mask1 = new Mat(); // Added for the lower red range
    Mat mask2 = new Mat(); // Added for the upper red range
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

    // Adjust the HSV range for red
    public Scalar lowerRed1 = new Scalar(0, 100, 100);
    public Scalar upperRed1 = new Scalar(10, 255, 255);
    public Scalar lowerRed2 = new Scalar(160, 100, 100);
    public Scalar upperRed2 = new Scalar(180, 255, 255);

    enum Detection {
        LEFT,
        CENTER,
        RIGHT
    }

    Detection detected = Detection.RIGHT;

    public RedCubeDetectionPipeline(Telemetry telemetry) {  // Constructor to initialize telemetry
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowerRed1, upperRed1, mask1);
        Core.inRange(hsv, lowerRed2, upperRed2, mask2);
        Core.bitwise_or(mask1, mask2, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw rectangles around the zones in green
        Imgproc.rectangle(frame, new Point(center_x, center_y), new Point(center_x + center_width, center_y + center_height), new Scalar(0, 255, 0), 1);
        Imgproc.rectangle(frame, new Point(left_x, left_y), new Point(left_x + side_width, left_y + side_height), new Scalar(0,255, 0), 1);

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            double centerX = rect.x + rect.width / 2.0;
            double centerY = rect.y + rect.height / 2.0;

            double area = Imgproc.contourArea(contour);

            // detect location
            if (area < 3000) {
                break;
            }

            if (centerX > center_x && centerX < center_x + center_width && centerY > center_y && centerY < center_y + center_height) {
                telemetry.addData("Location", "Center");
                detected = Detection.CENTER;
            } else if (centerX > left_x && centerX < left_x + side_width && centerY > left_y && centerY < left_y + side_height) {
                telemetry.addData("Location", "Left");
                detected = Detection.LEFT;
            } else if (detected == null) {
                telemetry.addData("Location (Assumed)", "Right");
                detected = Detection.RIGHT;
            }

            // Telemetry data for the area of each blue object
            telemetry.addData("Area", area);

            Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
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
}