package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processor.PropReco;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous()
public class VisualDetection extends OpMode {

    VisionPortal visionPortal;
    PropReco propReco;
    int desiredTag;


    @Override
    public void init_loop() {
        if (propReco.getSelection() == PropReco.Selected.LEFT && propReco.getSelection() == PropReco.Selected.RIGHT) {
            desiredTag = 3;
        } else {
            desiredTag = 2;
        }

        telemetry.addData("alliance", PropReco.alliance);
        telemetry.addData("selected", propReco.getSelection());
        telemetry.addData("desired tag", desiredTag);

        telemetry.addData("BR", propReco.averageBlueRight);
        telemetry.addData("BL", propReco.averageBlueLeft);
        telemetry.addData("BM", propReco.averageBlueMiddle);
        telemetry.addData("RR", propReco.averageRedRight);
        telemetry.addData("RL", propReco.averageRedLeft);
        telemetry.addData("RM", propReco.averageRedMiddle);

        telemetry.update();
    }

    @Override
    public void init() {
        propReco = new PropReco();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propReco);
    }

    @Override
    public void loop() {

    }
}