package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class LockIn extends OpMode {

    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;

    boolean isOriginSet = false;

    int[] motorOriginPosition = new int[3];


    @Override
    public void init() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            new Thread(this::lock);
        } else {
            isOriginSet = false;
            motorOriginPosition[0] = 0;
            motorOriginPosition[1] = 0;
            motorOriginPosition[2] = 0;
            motorOriginPosition[3] = 0;

        }
    }

    private void lock(){
        if(!isOriginSet){
            motorOriginPosition[0] = leftBackDrive.getCurrentPosition();
            motorOriginPosition[1] = rightBackDrive.getCurrentPosition();
            motorOriginPosition[2] = leftFrontDrive.getCurrentPosition();
            motorOriginPosition[3] = rightFrontDrive.getCurrentPosition();
        }

        returnToOrigin(motorOriginPosition);
    }

    private void returnToOrigin(int[] origin){
        leftBackDrive.setTargetPosition(origin[0]);
        rightBackDrive.setTargetPosition(origin[1]);
        leftFrontDrive.setTargetPosition(origin[2]);
        rightFrontDrive.setTargetPosition(origin[3]);
    }
}
