package org.firstinspires.ftc.teamcode.processor;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;


public class PropReco implements VisionProcessor {

    
    public static PropReco.AllianceColor alliance = AllianceColor.NONE;
    PropReco.Selected selection = PropReco.Selected.NONE;

    // these 2 line are for the processing of the frame 
    
    Mat frame = new Mat();
    Mat submat = new Mat();

    // these 3 lines are for the division webcam(video feed) input in 3 seperate parts
    
    int width = frame.width();
    int height = frame.height();
    int sectionWidth = width / 3;

    // the following lines are for the inistialisation of the rectangles
    
    public Rect rectLeft = new Rect(0, 0, sectionWidth, height);
    public Rect rectMiddle = new Rect(sectionWidth, 0, sectionWidth, height);
    public Rect rectRight = new Rect(sectionWidth * 2, 0, sectionWidth, height);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // the following lines are for geting the averge RGB values of the RED and BLUE value value contained in the squeres
        
        double averageRedLeft = getAvgRed(frame, rectLeft);
        double averageBlueLeft = getAvgBlue(frame, rectLeft);

        double averageRedMiddle = getAvgRed(frame, rectMiddle);
        double averageBlueMiddle = getAvgBlue(frame, rectMiddle);

        double averageRedRight = getAvgRed(frame, rectRight);
        double averageBlueRight = getAvgBlue(frame, rectRight);

        double averageRed = (averageRedLeft + averageRedRight + averageRedMiddle) / 3;
        double averageBlue = (averageBlueLeft + averageBlueRight + averageBlueMiddle) / 3;

        // this statement determens what alliance is the robot placed in
        
        if(averageRed > averageBlue){
            alliance = AllianceColor.RED;
        }else {
            alliance = AllianceColor.BLUE;
        }

        // these statements are for determening the postition of the prop 
        
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

    // this fanction gets the average RED value of the subsection passed in
    
    protected double getAvgRed(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[0];
    }

    // this fanction gets the average BLUE value of the subsection passed in
    
    protected double getAvgBlue(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[2];
    }

    // this function is responsible for drawing the rectangles
    
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        
        // these lines handle the colour of the rectangles that the prop is in
        
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        // these lines handle the colour of the rectangle that the prop is NOT in
        
        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        // these lines set the spaces for the rectangles
        
        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft,
                scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle,
                scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        // these lines draw the rectangles

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
