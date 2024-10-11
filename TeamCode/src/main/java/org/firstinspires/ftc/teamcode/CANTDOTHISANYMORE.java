package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processor.Simpl;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class CANTDOTHISANYMORE extends OpMode {
    VisionPortal visionPortal;
    Simpl simpl;
    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, simpl);
    }

    @Override
    public void loop() {
        telemetry.addData("red",simpl.redValue);
    }
}
