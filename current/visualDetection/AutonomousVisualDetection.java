package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.processor.ToFindDaProp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "Visual Autonomous")
public class AutonomousVisualDetection extends OpMode {

    private VisionPortal visionPortal;
    private PropProcessor propProcessor;
    private int desiredTagID = -1;

    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private boolean targetFound;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init_loop() {
        if(propProcessor.getSelection() == PropProcessor.Selected.LEFT && propProcessor.getSelection()== PropProcessor.Selected.RIGHT){
            desiredTagID = 3;
        }else {
            desiredTagID = 2;
        }
    }

    @Override
    public void init() {
        telemetry.addLine("initializing...");
        propProcessor = new PropProcessor();
        initPropDetection(propProcessor);



    }

    @Override
    public void start() {
        telemetry.addLine("Starting...");
        visionPortal.close();
        visionPortal = null;

        initAprilTag();

        try {
            setManualExposure(6, 250);
        }catch (InterruptedException e){
            e.printStackTrace();
        }

    }

    @Override
    public void loop() {
        telemetry.addData("Desired Tag", desiredTagID);
        targetFound = false;
        desiredTag = null;

        List<AprilTagDetection> currentDetection = aprilTag.getDetections();
        for (AprilTagDetection detection: currentDetection){
            if(detection.metadata != null){
                if((desiredTagID < 0) || (detection.id == desiredTagID)){
                    targetFound = true;
                    desiredTag = detection;
                    telemetry.addLine("Desired tag found.");
                }else {
                    telemetry.addData("Skipping", "Tag ID %d is not found", detection.id);
                }
            }else {
                telemetry.addData("Unknown", "Tag ID %d not in TagLibrary", detection.id);
            }
        }
    }

    private void initPropDetection(PropProcessor processor){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, processor);

    }

    /*private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }


        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
        telemetry.addData("Camera", "Ready");
        telemetry.update();

    }*/

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

    }
}
