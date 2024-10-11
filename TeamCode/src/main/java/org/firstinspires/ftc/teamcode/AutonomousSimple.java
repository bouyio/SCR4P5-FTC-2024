package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processor.PropReco;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous()
public class AutonomousSimple extends OpMode {


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


    DcMotor cascadeRotateL;
    DcMotor cascadeRotateR;
    CRServo cascadeExtendL;
    CRServo cascadeExtendLii;
    DcMotor cascadeExtendR;
    Servo clawL;
    Servo clawR;
    CRServo clawRotate;
    Servo droneServo;
    @Override
    public void start() {
        clawR.setPosition(0.6);
        clawL.setPosition(0.48);

        leftBackDrive.setPower(0.5);
        leftFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);

        opModeSleep(2000);

        leftBackDrive.setPower(0.5);
        leftFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
    }

    @Override
    public void init() {

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
    public void loop() {


    }

    void opModeSleep(long millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e){
            e.printStackTrace();
        }
    }
}
