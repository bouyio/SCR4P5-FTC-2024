package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous()
public class AutonomousBorrowed extends OpMode {
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;

    private boolean targetFound  = false;
    private double  drive        = 0;
    private double  strafe       = 0;
    private double  turn         = 0;
    private boolean arrivedToTarget = false;
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


    @Override
    public void start() {

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
    }

    @Override
    public void loop() {

    }
}
