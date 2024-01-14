package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class MecanumDrive extends OpMode {

    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;

    private double lStickX;
    private double lStickY;
    private double rStickX;

    private double rFrontMotorPower;
    private double lFrontMotorPower;
    private double rBackMotorPower;
    private double lBackMotorPower;

    private double max;

    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;

    private boolean isTurboEnabled;


    @Override
    public void init() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        isTurboEnabled = gamepad1.left_bumper;

        lStickX = gamepad1.left_stick_x;
        lStickY = gamepad1.left_stick_y;
        rStickX = gamepad1.right_stick_x;

        lFrontMotorPower = lStickY-lStickX-rStickX;
        lBackMotorPower = lStickY+lStickX-rStickX;
        rBackMotorPower = lStickY-lStickX+rStickX;
        rFrontMotorPower = lStickY+lStickX+rStickX;

        max = Math.max(Math.abs(lFrontMotorPower), Math.abs(lBackMotorPower));

        max = Math.max(max, Math.abs(rBackMotorPower));

        max = Math.max(max, Math.abs(rFrontMotorPower));

        if(max>1.0) {
            lFrontMotorPower = lFrontMotorPower / max;
            lBackMotorPower = lBackMotorPower / max;
            rFrontMotorPower = rFrontMotorPower / max;
            rBackMotorPower = rBackMotorPower / max;
        }


        if(!isTurboEnabled) {
            backLeftMotor.setPower(lBackMotorPower * 0.7);
            backRightMotor.setPower(rBackMotorPower * 0.7);
            frontLeftMotor.setPower(lFrontMotorPower * 0.7);
            frontRightMotor.setPower(rFrontMotorPower * 0.7);
            return;
        }

        backLeftMotor.setPower(lBackMotorPower);
        backRightMotor.setPower(rBackMotorPower);
        frontLeftMotor.setPower(lFrontMotorPower);
        frontRightMotor.setPower(rFrontMotorPower);
    }






}
