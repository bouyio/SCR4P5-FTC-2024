
/*
*   This code was meant to be the complete
*   autonomous for the old driving base
*   but it did not work. Started in the
*   10th of December 2023 and eventually
*   finished in the 17th of December 2023
*/

package org.firstinspires.ftc.teamcode;

//imports:
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "Autonomous Test")
public class AutonomousOld extends OpMode {

    private final double desiredDistance = 3.0;

    //Sets camera enviroment
    private VisionPortal visionPortal;
    private ToFindDaProp propProcessor;
    private int desiredTagID = -1;

    //Setting tag detection settings
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private boolean targetFound;

    //Sets variables for the motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    //Sets autonomus drive settings
    private final double maxAutoSpeed = 0.5;
    private final double maxAutoTurn = 0.25;
    private final double speedGain = 0.02;
    private final double turnGain = 0.25;
    private static double drive = 0;
    private static double turn = 0;

    @Override
    public void init_loop() {
        //PropDetection
        if(propProcessor.getSelection() == ToFindDaProp.Selected.LEFT && propProcessor.getSelection()== ToFindDaProp.Selected.RIGHT){
            desiredTagID = 3;
        }else {
            desiredTagID = 2;
        }
    }

    @Override
    public void init() {
        //Prints the string "initializing..." to telemetry
        telemetry.addLine("initializing...");
        //Using functions from library to detect the prop
        propProcessor = new ToFindDaProp();
        initPropDetection(propProcessor);
        
        //Sets motor Values
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        //Setting motors direction so they move in the same direction
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    @Override
    public void start() {
        telemetry.addLine("Starting...");
        //Closing the vision portal
        visionPortal.close();
        visionPortal = null;

        initAprilTag();

        //Setting the manualExposure
        try {
            setManualExposure(6, 250);
        }catch (InterruptedException e){
            e.printStackTrace();
        }

    }

    @Override
    public void loop() {
        //telemetry.addData("Desired Tag", desiredTagID);
        targetFound = false;
        desiredTag = null;

        //Setting up a list with all the aprilTags that have been detecting
        List<AprilTagDetection> currentDetection = aprilTag.getDetections();
        //Running through all the tags that are in the list
        for (AprilTagDetection detection: currentDetection){
            //Checks if the aprlilTag has metadata
            if(detection.metadata != null){
                //Checks if it has found the wanted tag
                if((desiredTagID < 0) || (detection.id == desiredTagID)){
                    //Setting up the variables to the desired tag
                    targetFound = true;
                    desiredTag = detection;
                    telemetry.addLine("Desired tag found.");
                }else {
                    //Sends meessage to telemetry that the tag found is not the wanted one 
                    telemetry.addData("Skipping", "Tag ID %d is not found", detection.id);
                }
            }else {
                //Send meessage to telemetry that the tag's list is not found
                telemetry.addData("Unknown", "Tag ID %d not in TagLibrary", detection.id);
            }
        }

        //If tag target not found do not continue 
        if(!targetFound){
            return;
        }

        // sets distance veriables 
        double rangeError = desiredTag.ftcPose.range - desiredDistance;
        double headingError = desiredTag.ftcPose.bearing;

        // sets variables for movement, and prints them to the telemetry 
        drive = Range.clip(rangeError * speedGain, -maxAutoSpeed, maxAutoSpeed);
        turn = Range.clip(headingError * turnGain, -maxAutoTurn, maxAutoTurn);
        telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);

        moveRobot(drive, turn);

        // waits for 10 seconds 
        try {
            sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    // fanction tht is used for the initialisting proccess of the detection 
    private void initPropDetection(ToFindDaProp processor){
        // lninks the camera to the script(visionportal)
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, processor);

    }

    
    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
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

    }

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

    // this is the fanction that moves the robot
    public void moveRobot(double x, double yaw){
        double leftPower = x- yaw;
        double rightPower = x + yaw;

        // fixes motor power if it's over the limits [-1,1]
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if(max > 1.0){
            leftPower /= max;
            rightPower /= max;
        }

        //seting up power
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
