package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Localization extends OpMode {
    // Declaring OpMode members.
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;

    boolean isOriginSet = false;

    int[] motorOriginPosition = new int[3];


    @Override
    public void init() {
        // Initialization of the motors.

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
        // Running the localization when the a button is pressed.
        if (gamepad1.a){
            // Beginning a new thread in order to run other code simultaneously.
            new Thread(this::lock);
        } else {
            // Reset the variable back to normal.
            isOriginSet = false;
            motorOriginPosition[0] = 0;
            motorOriginPosition[1] = 0;
            motorOriginPosition[2] = 0;
            motorOriginPosition[3] = 0;

        }
    }

    private void lock(){
        // Set the origin position if it is not set.
        if(!isOriginSet){
            motorOriginPosition[0] = leftBackDrive.getCurrentPosition();
            motorOriginPosition[1] = rightBackDrive.getCurrentPosition();
            motorOriginPosition[2] = leftFrontDrive.getCurrentPosition();
            motorOriginPosition[3] = rightFrontDrive.getCurrentPosition();
        }

        returnToOrigin(motorOriginPosition);
    }

    private void returnToOrigin(int[] origin){
        // Initializing the variable required to return to the origin point.
        int[] motorDirection = new int[4];
        int[] positionDiff = new int[4];
        int[] currentPosition = new int[4];

        // Getting the current position.
        currentPosition[0] = leftBackDrive.getCurrentPosition();
        currentPosition[1] = rightBackDrive.getCurrentPosition();
        currentPosition[2] = leftFrontDrive.getCurrentPosition();
        currentPosition[3] = rightFrontDrive.getCurrentPosition();

        // Calculating the difference between the current and origin position.
        positionDiff[0] = currentPosition[0] - origin[0];
        positionDiff[1] = currentPosition[1] - origin[1];
        positionDiff[2] = currentPosition[2] - origin[2];
        positionDiff[3] = currentPosition[3] - origin[3];

        // Calculating the direction of the motors.
        for (int i = 0; i < 4; i++){
            if(positionDiff[i] > 0){
                motorDirection[i] = -1;
            } else if (positionDiff[i] < 0){
                motorDirection[i] = 1;
            } else {
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                return;
            }
        }

        // Running the motors.
        leftBackDrive.setPower(motorDirection[0]);
        rightBackDrive.setPower(motorDirection[1]);
        leftFrontDrive.setPower(motorDirection[2]);
        rightFrontDrive.setPower(motorDirection[3]);
    }
}
