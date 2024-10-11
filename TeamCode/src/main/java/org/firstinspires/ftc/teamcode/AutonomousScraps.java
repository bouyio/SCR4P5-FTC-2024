    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.Range;

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
    import org.firstinspires.ftc.teamcode.processor.PropReco;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
    import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

    import java.util.List;

    @Autonomous()
    public class AutonomousScraps extends OpMode {

        final double DISIEREDDISTANCE = 1.0; //  this is how close the camera should get to the target (inches)




        final double SPEEDGAIN =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFEGAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURNGAIN =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAXAUTOSPEED = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAXAUTOSTRAFE = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAXAUTOTURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        PropReco propReco;
        VisionPortal visionPortal;
        VisionPortal vpTag;
        int desiredTagId = -1;
        private AprilTagDetection desiredTag = null;

        DcMotor leftBackDrive;
        DcMotor rightBackDrive;
        DcMotor leftFrontDrive;
        DcMotor rightFrontDrive;

        private boolean targetFound  = false;
        private double  drive        = 0;
        private double  strafe       = 0;
        private double  turn         = 0;
        private boolean arrivedToTarget = false;

        IMU gyroscope;

        private AprilTagProcessor aprilTag;

        DcMotor cascadeRotateL;
        DcMotor cascadeRotateR;
        CRServo cascadeExtendL;
        CRServo cascadeExtendLii;
        DcMotor cascadeExtendR;
        Servo clawL;
        Servo clawR;
        CRServo clawRotate;
        Servo droneServo;

        PropReco.Selected currentSelection;


        @Override
        public void init() {
            propReco = new PropReco();
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, propReco);
            leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            cascadeRotateL = hardwareMap.get(DcMotor.class, "CRL");
            cascadeRotateR = hardwareMap.get(DcMotor.class, "CRR");
            cascadeRotateL.setDirection(DcMotorSimple.Direction.REVERSE);

            clawL = hardwareMap.get(Servo.class, "cL");
            clawR = hardwareMap.get(Servo.class, "cR");
            clawRotate = hardwareMap.get(CRServo.class,"clawRotate");

            IMU.Parameters gyroscopeParameters;

            gyroscopeParameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                    )
            );

            gyroscope = hardwareMap.get(IMU.class, "gyroscope");

            gyroscope.initialize(gyroscopeParameters);

        }

        @Override
        public void init_loop() {
            if(propReco.getSelection() == PropReco.Selected.LEFT && propReco.getSelection() == PropReco.Selected.RIGHT){
                if(PropReco.alliance == PropReco.AllianceColor.BLUE){
                    desiredTagId = 2;
                }else if (PropReco.alliance == PropReco.AllianceColor.RED){
                    desiredTagId = 6;
                }
            } else if (propReco.getSelection() == PropReco.Selected.MIDDLE){
                if(PropReco.alliance == PropReco.AllianceColor.BLUE){
                    desiredTagId = 1;
                }else if (PropReco.alliance == PropReco.AllianceColor.RED){
                    desiredTagId = 5;
                }
            }

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
        public void start() {
            initAprilTagDetection();

            currentSelection = propReco.getSelection();


            clawR.setPosition(0.6);
            clawL.setPosition(0.48);

            clawRotate.setPower(1);
            opModeSleep(1200);
            clawRotate.setPower(0);


            moveRobot(0.5, 0, 0);

            opModeSleep(1150);

            moveRobot(0, 0, 0);

            if (currentSelection == PropReco.Selected.LEFT) {
                rightBackDrive.setPower(-0.3);
                rightFrontDrive.setPower(-0.3);
                leftBackDrive.setPower(0.3);
                leftFrontDrive.setPower(0.3);

                opModeSleep(1500);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);

                clawL.setPosition(0.7);

                rightBackDrive.setPower(0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(-0.3);
                leftFrontDrive.setPower(-0.3);

                opModeSleep(3000);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
            } else if (currentSelection == PropReco.Selected.RIGHT) {
                rightBackDrive.setPower(0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(-0.3);
                leftFrontDrive.setPower(-0.3);

                opModeSleep(900);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);

                clawL.setPosition(0.7);

                rightBackDrive.setPower(-0.3);
                rightFrontDrive.setPower(-0.3);
                leftBackDrive.setPower(0.3);
                leftFrontDrive.setPower(0.3);

                opModeSleep(1100);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);

                moveRobot(-0.5, 0, 0);

                opModeSleep(1100);

                moveRobot(0, 0, 0);

                rightBackDrive.setPower(0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(-0.3);
                leftFrontDrive.setPower(-0.3);

                opModeSleep(1500);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);


            } else if (currentSelection == PropReco.Selected.MIDDLE) {
                moveRobot(0.5, 0, 0);

                opModeSleep(200);

                moveRobot(0, 0, 0);


                rightBackDrive.setPower(0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(-0.3);
                leftFrontDrive.setPower(-0.3);

                opModeSleep(1600);

                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                leftFrontDrive.setPower(0);

            }


            //while(!arrivedToTarget) searchForTag(desiredTagId);
            moveRobot(0.5,0,0);
            opModeSleep(8000);
            moveRobot(0,0,0);

        }

        @Override
        public void loop() {

        }

        void opModeSleep(long millis){
            try {
                Thread.sleep(millis);
            } catch (InterruptedException e){
                e.printStackTrace();
            }
        }

        private void moveRobot(double x, double y, double yaw){
            // Calculate wheel powers.
            double leftFrontPower    =  x +yaw +y;
            double rightFrontPower   =  x -yaw -y;
            double leftBackPower     =  -x +yaw -y;
            double rightBackPower    =  -x -yaw +y;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

        private void searchForTag(int tagId) {

            targetFound = false;
            desiredTag  = null;



            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
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

            if (targetFound) {

                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Unable to find valid target sowwy\n");
                return;
            }

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DISIEREDDISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError = desiredTag.ftcPose.yaw;// Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEEDGAIN, -MAXAUTOSPEED, MAXAUTOSPEED);
            turn   = Range.clip(headingError * TURNGAIN, -MAXAUTOTURN, MAXAUTOTURN) ;
            strafe = Range.clip(-yawError * STRAFEGAIN, -MAXAUTOSTRAFE, MAXAUTOSTRAFE);
            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            if(rangeError < 25){
                arrivedToTarget = true;
            }

            telemetry.update();


            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

            opModeSleep(10);


        }

        private void rotate(double angle){
            double rotation = 0;

            while (rotation <= angle){
                rightBackDrive.setPower(-0.3);
                rightFrontDrive.setPower(-0.3);
                leftBackDrive.setPower(0.3);
                leftFrontDrive.setPower(0.3);


                YawPitchRollAngles robotOrientation = gyroscope.getRobotYawPitchRollAngles();

                rotation = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("rotation", rotation);
                telemetry.update();
            }

            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);

        }

        private void rotate(double angle, boolean reverse){
            double rotation = 0;

            while (rotation >= angle){
                rightBackDrive.setPower(0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(-0.3);
                leftFrontDrive.setPower(-0.3);

                YawPitchRollAngles robotOrientation = gyroscope.getRobotYawPitchRollAngles();

                rotation = robotOrientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("rotation", rotation);
                telemetry.update();
            }

            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
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
            aprilTag.setDecimation(1);

            // Create the vision portal by using a builder.

            vpTag = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .enableLiveView(false)
                    .build();
        }


    }
