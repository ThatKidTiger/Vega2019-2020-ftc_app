package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvTracker;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.BotConstants.CV_CONSTS.blur;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.CV_CONSTS.sigX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.CV_CONSTS.threshold;

public class stoneIdentifier extends OpenCvTracker {
    Scalar color;

    public stoneIdentifier(Scalar color) {
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        //input.release();

        Mat average = new Mat();
        Imgproc.GaussianBlur(hsv, average, new Size(blur, blur), sigX);
        hsv.release();

        Mat dilate = new Mat();
        Imgproc.dilate(average, dilate, new Mat());
        average.release();

        Mat thresh = new Mat();
        Core.inRange(dilate, new Scalar(20, 90, 210), new Scalar(30, 255, 255), thresh);
        dilate.release();

        Mat cannyOutput = new Mat();
        Imgproc.Canny(thresh, cannyOutput, threshold, threshold * 3);
        thresh.release();

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        /*Map<MatOfPoint, Double> contourmap = new HashMap<MatOfPoint, Double>();
        for(int i = 0; i < contours.size(); i++) {
            contourmap.put(contours.get(i), Imgproc.contourArea(contours.get(i)));
        }*/

        double maxVal = 0;
        int maxValIdx = 0;
        //Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);
        cannyOutput.release();
        if(contours.size() > 0) {
            for(int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if(maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }

            Imgproc.drawContours(input, contours, maxValIdx, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            Rect bounds = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.rectangle(input, bounds, color, 2);
        }
        return input;
    }
}
