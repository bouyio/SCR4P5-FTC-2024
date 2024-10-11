package org.firstinspires.ftc.teamcode.processor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;


public class PropReco implements VisionProcessor {


    public static PropReco.AllianceColor alliance = AllianceColor.NONE;
    PropReco.Selected selection = PropReco.Selected.NONE;


    int width = 640;
    int height = 240;
    int sectionWidth = width / 3;

    public double averageRedLeft;
    public double averageBlueLeft;
    public double averageRedMiddle;
    public double averageBlueMiddle;
    public double averageRedRight;
    public double averageBlueRight;


    public Rect rectLeft = new Rect(0, 0, sectionWidth, height);
    public Rect rectMiddle = new Rect(sectionWidth, 0, sectionWidth, height);
    public Rect rectRight = new Rect(sectionWidth * 2, 0, sectionWidth, height);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        averageRedLeft = getAvgRed(frame, rectLeft);
        averageBlueLeft = getAvgBlue(frame, rectLeft);

        averageRedMiddle = getAvgRed(frame, rectMiddle);
        averageBlueMiddle = getAvgBlue(frame, rectMiddle);

        averageRedRight = getAvgRed(frame, rectRight);
        averageBlueRight = getAvgBlue(frame, rectRight);

        double averageRed = (averageRedLeft + averageRedRight + averageRedMiddle) / 3;
        double averageBlue = (averageBlueLeft + averageBlueRight + averageBlueMiddle) / 3;

        if(averageRed > averageBlue){
            alliance = AllianceColor.RED;
        }else {
            alliance = AllianceColor.BLUE;
        }

        if (AllianceColor.RED == alliance){
            if((averageRedLeft > averageRedMiddle) && (averageRedLeft > averageRedRight)){
                return Selected.LEFT;
            } else if ((averageRedMiddle > averageRedRight) && (averageRedMiddle > averageRedLeft)) {
                return Selected.MIDDLE;
            }
            return Selected.RIGHT;
        }else {
            if((averageBlueLeft > averageBlueMiddle) && (averageBlueLeft > averageBlueRight)){
                return Selected.LEFT;
            } else if ((averageBlueMiddle > averageBlueRight) && (averageBlueMiddle > averageBlueLeft)) {
                return Selected.MIDDLE;
            }
            return Selected.RIGHT;
        }
    }

    protected double getAvgRed(Mat input, Rect rect) {
        try {
            Mat submat = new Mat(input, rect);
            Scalar color = Core.mean(submat);
            return color.val[0];
        } catch (CvException e){
            e.printStackTrace();
        }
        return 0;
    }


    protected double getAvgBlue(Mat input, Rect rect) {
        try {
            Mat submat = new Mat(input, rect);
            Scalar color = Core.mean(submat);
            return color.val[2];
        } catch (CvException e){
            e.printStackTrace();
        }
        return 0;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft,
                scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle,
                scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (PropReco.Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }

    public PropReco.Selected getSelection() {
        return selection;
    }


    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum AllianceColor{
        NONE,
        BLUE,
        RED
    }
}
