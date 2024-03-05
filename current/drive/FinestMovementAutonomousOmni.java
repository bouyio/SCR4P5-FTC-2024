package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.processor.PropReco;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous()
public class FinestMovementAutonomousOmni extends OpMode {

    // Declaring OpMode members.
    TouchSensor touchSensor;

    final double DISIEREDDISTANCE = 1.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEEDGAIN =  0.02  ;   //  Forward Speed Control "Gain". 
    final double STRAFEGAIN =  0.015 ;   //  Strafe Speed Control "Gain".  
    final double TURNGAIN =  0.01  ;   //  Turn Control "Gain".  

    final double MAXAUTOSPEED = 0.7;   //  Clip the approach speed to this max value 
    final double MAXAUTOSTRAFE = 0.7;   //  Clip the approach speed to this max value
    final double MAXAUTOTURN = 0.3;   //  Clip the turn speed to this max value 

    // The componets required for the prop detection.
    PropReco propReco;
    VisionPortal visionPortal;

    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;

    // The componets required for the april tag tracking.
    int desiredTagId = -1;
    private AprilTagDetection desiredTag = null;
    private boolean targetFound  = false;
    private AprilTagProcessor aprilTag;
    
    private double  drive        = 0;
    private double  strafe       = 0;
    private double  turn         = 0;
    private boolean arrivedToTarget = false;

    IMU gyroscope;

    @Override
    public void init_loop() {

        // These lines determine which april tags to track based on the position of the prop.
        if(propReco.getSelection() == PropReco.Selected.LEFT && propReco.getSelection() == PropReco.Selected.RIGHT){
            if(PropReco.alliance == PropReco.AllianceColor.BLUE){
                desiredTagId = 2;
            }else if (PropReco.alliance == PropReco.AllianceColor.RED){
                desiredTagId = 5;
            }
        } else if (propReco.getSelection() == PropReco.Selected.MIDDLE){
            if(PropReco.alliance == PropReco.AllianceColor.BLUE){
                desiredTagId = 3;
            }else if (PropReco.alliance == PropReco.AllianceColor.RED){
                desiredTagId = 6;
            }
        }

        // Printing debug information to telemetry.
        telemetry.addData("alliance", PropReco.alliance);
        telemetry.addData("selected", propReco.getSelection());
        telemetry.addData("desired tag", desiredTagId);

        telemetry.addData("BR", propReco.averageBlueRight);
        telemetry.addData("BL", propReco.averageBlueLeft);
        telemetry.addData("BM", propReco.averageBlueMiddle);
        telemetry.addData("RR", propReco.averageRedRight);
        telemetry.addData("RL", propReco.averageRedLeft);
        telemetry.addData("RM", propReco.averageRedMiddle);
    }

    @Override
    public void init() {
        // Initialization of the prop rocognition.
        propReco = new PropReco();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propReco);

        // Intialization of the motors.
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialization of the gyroscope.
        IMU.Parameters gyroscopeParameters;

        gyroscopeParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        gyroscope = hardwareMap.get(IMU.class, "gyroscope");

        gyroscope.initialize(gyroscopeParameters);
        
        // Initialization of the touch sensor.
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
    }

    @Override
    public void loop() {

    }

    private void searchForTag(int tagId){
        targetFound = false;
        desiredTag  = null;

        // If the robot has hit the backdrop then we are stopping.
        arrivedToTarget = touchSensor.isPressed();
        
        // Add all the detections to a list.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        
        for (AprilTagDetection detection : currentDetections) {
            // Check if there is size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagId < 0) || (detection.id == tagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Printing debug info to the telemetry.
        if (targetFound) {

            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Unable to find valid target sowwy\n");
            leftBackDrive.setPower(0.5);
            leftFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);

            opModeSleep(500);

            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);


        }

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (desiredTag.ftcPose.range - DISIEREDDISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;double  yawError = desiredTag.ftcPose.yaw;
        drive  = Range.clip(rangeError * SPEEDGAIN, -MAXAUTOSPEED, MAXAUTOSPEED);
        turn   = Range.clip(headingError * TURNGAIN, -MAXAUTOTURN, MAXAUTOTURN) ;
        strafe = Range.clip(-yawError * STRAFEGAIN, -MAXAUTOSTRAFE, MAXAUTOSTRAFE);
        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        // Check whether the robot has arrive to the target.
        if(headingError == 0){
            arrivedToTarget = true;
        }

        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);

        opModeSleep(10);
    }

    // This function is resonsible for the movement of the robot.
    private void moveRobot(double x, double y, double yaw){
        // Calculate motor powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize motor powers to be less than max (1.0)
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the motors.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void start() {
        // These lines clear the telemetry and the vision portal.
        telemetry.clearAll();
        visionPortal = null;

        // These lines initialize the apriltag detection.
        initAprilTagDetection();

        try {
            setManualExposure(6, 250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // These lines drive the robot to the prop.
        if(propReco.getSelection() == PropReco.Selected.LEFT){
            rightBackDrive.setPower(0.7);
            leftFrontDrive.setPower(0.7);

            opModeSleep(700);

            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
        } else if(propReco.getSelection() == PropReco.Selected.RIGHT){

            leftBackDrive.setPower(0.7);
            rightFrontDrive.setPower(0.7);

            opModeSleep(700);

            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        } else {
            leftBackDrive.setPower(0.7);
            rightBackDrive.setPower(0.7);

            opModeSleep(700);

            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        
        leftBackDrive.setPower(-0.7);
        rightBackDrive.setPower(-0.7);

        opModeSleep(300);

        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        YawPitchRollAngles robotOrientation;
        robotOrientation = gyroscope.getRobotYawPitchRollAngles();

        double rotation = robotOrientation.getYaw(AngleUnit.DEGREES);

        rotate(90, rotation);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        while (!arrivedToTarget){
            searchForTag(desiredTagId);
        }

        arrivedToTarget = false;

        leftBackDrive.setPower(-1.00);
        rightBackDrive.setPower(-1.00);

        opModeSleep(500);

        leftBackDrive.setPower(0.00);
        rightBackDrive.setPower(0.00);

        gyroscope.resetYaw();
        rotation = 0;

        rotate(180, rotation);

        leftBackDrive.setPower(0.00);
        leftFrontDrive.setPower(0.00);
        rightBackDrive.setPower(0.00);
        rightFrontDrive.setPower(0.00);

        while (!arrivedToTarget){
            if (PropReco.alliance == PropReco.AllianceColor.BLUE) {
                searchForTag(7);
            } else if (PropReco.alliance == PropReco.AllianceColor.RED) {
                searchForTag(8);
            }
        }

        leftBackDrive.setPower(-1.00);
        rightBackDrive.setPower(-1.00);

        opModeSleep(500);

        leftBackDrive.setPower(0.00);
        rightBackDrive.setPower(0.00);

        gyroscope.resetYaw();
        rotation = 0;

        rotate(180, rotation);

        while (!arrivedToTarget){
            searchForTag(desiredTagId);
        }
    }

    private void initAprilTagDetection(){
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();
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
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

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

    }

    private void rotate(double angle, double rotation){
        while (rotation < angle){
            leftBackDrive.setPower(MAXAUTOTURN);
            leftFrontDrive.setPower(MAXAUTOTURN);
            rightBackDrive.setPower(-MAXAUTOTURN);
            rightFrontDrive.setPower(-MAXAUTOTURN);

            YawPitchRollAngles robotOrientation = gyroscope.getRobotYawPitchRollAngles();

            rotation = robotOrientation.getYaw(AngleUnit.DEGREES);
        }
    }

    // This function is tho generally clean up the code by removing a lot of try/catch statements.
    private void opModeSleep(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
