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
public class FoivosMaster extends OpMode {

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


    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;

    private boolean targetFound  = false;
    private double  drive        = 0;
    private double  strafe       = 0;
    private double  turn         = 0;
    private boolean arrivedToTarget = false;



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







    }

    @Override
    public void init_loop() {
        if(propReco.getSelection() == PropReco.Selected.LEFT) {
            currentSelection = PropReco.Selected.LEFT;
        }
        } else if (propReco.getSelection() == PropReco.Selected.MIDDLE){
            if(PropReco.alliance == PropReco.AllianceColor.BLUE){
                desiredTagId = 1;
            }else if (PropReco.alliance == PropReco.AllianceColor.RED){
                desiredTagId = 5;
            }
        }

        //telemetry.addData("alliance", PropReco.alliance);
        //telemetry.addData("selected", propReco.getSelection());
        //telemetry.addData("desired tag", desiredTagId);

        //telemetry.addData("BR", propReco.averageBlueRight);
        //telemetry.addData("BL", propReco.averageBlueLeft);
        //telemetry.addData("BM", propReco.averageBlueMiddle);
        //telemetry.addData("RR", propReco.averageRedRight);
        //telemetry.addData("RL", propReco.averageRedLeft);
        //telemetry.addData("RM", propReco.averageRedMiddle);
    }

    @Override
    public void start(){


        currentSelection = PropReco.Selected.RIGHT;


        clawR.setPosition(0.6);
        clawL.setPosition(0.48);

        clawRotate.setPower(1);
        opModeSleep(1200);
        clawRotate.setPower(0);


        moveRobot(0.5,0,0);

        opModeSleep(1150);

        moveRobot(0,0,0);
        opModeSleep(500);

        if(currentSelection == PropReco.Selected.LEFT){
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

            // rightBackDrive.setPower(0.3);
            //rightFrontDrive.setPower(0.3);
            //leftBackDrive.setPower(-0.3);
            //leftFrontDrive.setPower(-0.3);

            opModeSleep(3000);

            //rightBackDrive.setPower(0);
            //rightFrontDrive.setPower(0);
            //leftBackDrive.setPower(0);
            //leftFrontDrive.setPower(0);
        } else if(currentSelection == PropReco.Selected.RIGHT){
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

            moveRobot(-0.5,0,0);

            opModeSleep(1100);

            moveRobot(0,0,0);

            rightBackDrive.setPower(0.3);
            rightFrontDrive.setPower(0.3);
            leftBackDrive.setPower(-0.3);
            leftFrontDrive.setPower(-0.3);

            opModeSleep(1500);

            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);



        } else if (currentSelection == PropReco.Selected.MIDDLE){
            moveRobot(0.2,0,0);

            opModeSleep(300);

            moveRobot(0,0,0);

            clawL.setPosition(0.7);
            moveRobot(-0.2,0,0);
            opModeSleep(200);


            rightBackDrive.setPower(0.1);
            rightFrontDrive.setPower(0.1);
            leftBackDrive.setPower(0.1);
            leftFrontDrive.setPower(0.1);

            opModeSleep(500);

            moveRobot(0,0,0);
            opModeSleep(10000);
        }

        moveRobot(-0.5,0,0);

        opModeSleep(1000);

        moveRobot(0,0,0);

        gyroscope.resetYaw();

        rotate(-90, true);



        while(!arrivedToTarget) searchForTag(desiredTagId);

        /*moveRobot(0,0,0);

        moveRobot(0.5,0,0);

        opModeSleep(100);

        moveRobot(0,0,0);

        rotate(180);

        cascadeRotateL.setPower(1);
        cascadeRotateR.setPower(1);

        opModeSleep(200);

        cascadeRotateL.setPower(0);
        cascadeRotateR.setPower(0);



        clawR.setPosition(0.6);*/

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


    }



