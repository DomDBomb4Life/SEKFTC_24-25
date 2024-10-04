package org.firstinspires.ftc.teamcode.util.pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelLocator extends OpenCvPipeline {
    private Mat gray = new Mat();
    private Mat binary = new Mat();
    private OpMode opmode;

    public PixelLocator(OpMode opmode) {
        this.opmode = opmode;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(gray, binary, 230, 255, Imgproc.THRESH_BINARY);

        // Debug input matrix
        opmode.telemetry.addData("Input Mat", matToString(input));
        opmode.telemetry.addData("Binary Mat", matToString(binary));
        opmode.telemetry.update();

        return binary;
    }

    public int getPixelLocation() {
        Mat[] thirds = getThirds(binary);

        for (int i = 0; i < thirds.length; i++) {
            opmode.telemetry.addData("Processing Third", i);
            opmode.telemetry.update();

            boolean found = determinePixelDensity(thirds[i]);
            if (found) {
                return i;
            }
        }
        return -1;
    }

    public boolean determinePixelDensity(Mat input) {
        int minDensity = 50;
        int maxDensity = 100;

        int minX = 50;
        int maxX = 100;

        int minY = 10;
        int maxY = 50;

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        // Debug contours and input
        opmode.telemetry.addData("Contours Size", contours.size());
        opmode.telemetry.addData("Input Mat in determinePixelDensity", matToString(input));
        for (int i = 0; i < contours.size(); i++) {
            opmode.telemetry.addData("Contour " + i, contours.get(i).toArray());
        }
        opmode.telemetry.update();

        boolean found = false;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            double density = area / (rect.width * rect.height);

            if ((density >= minDensity && density <= maxDensity) &&
                    (rect.width <= maxX && rect.width >= minX) &&
                    (rect.height <= maxY && rect.height >= minY)) {
                found = true;
                break;
            }
        }
        return found;
    }

    public Mat[] getThirds(Mat input) {
        int width = input.cols();
        int thirdWidth = width / 3;

        Mat firstThird = input.submat(new Rect(0, 0, thirdWidth, input.rows()));
        Mat secondThird = input.submat(new Rect(thirdWidth, 0, thirdWidth, input.rows()));
        Mat thirdThird = input.submat(new Rect(2 * thirdWidth, 0, thirdWidth, input.rows()));

        return new Mat[]{firstThird, secondThird, thirdThird};
    }

    // Helper method to convert Mat to String
    private String matToString(Mat mat) {
        StringBuilder sb = new StringBuilder();
        for (int row = 0; row < mat.rows(); row++) {
            for (int col = 0; col < mat.cols(); col++) {
                double[] data = mat.get(row, col);
                sb.append("[");
                for (double datum : data) {
                    sb.append(datum).append(",");
                }
                sb.append("] ");
            }
            sb.append("\n");
        }
        return sb.toString();
    }
}
