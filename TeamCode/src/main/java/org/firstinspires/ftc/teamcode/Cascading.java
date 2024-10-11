package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp()
public class Cascading extends OpMode {
    DcMotor cascadeRotateL;
    DcMotor cascadeRotateR;
    CRServo cascadeExtendL;
    CRServo cascadeExtendLii;
    DcMotor cascadeExtendR;
    Servo clawL;
    Servo clawR;
    @Override
    public void init() {
        cascadeRotateL = hardwareMap.get(DcMotor.class, "CRL");
        cascadeRotateR = hardwareMap.get(DcMotor.class, "CRR");
        cascadeRotateL.setDirection(DcMotorSimple.Direction.REVERSE);

        cascadeExtendL = hardwareMap.get(CRServo.class, "CEL");
        cascadeExtendLii = hardwareMap.get(CRServo.class, "CELii");
        cascadeExtendR = hardwareMap.get(DcMotor.class, "CER");
        cascadeExtendL.setDirection(DcMotorSimple.Direction.REVERSE);

        clawL = hardwareMap.get(Servo.class, "cL");
        clawR = hardwareMap.get(Servo.class, "cR");

    }

    @Override
    public void loop() {
        cascadeRotateL.setPower(gamepad1.left_stick_y * 0.5);
        cascadeRotateR.setPower(gamepad1.left_stick_y * 0.5);


        if (gamepad1.a) {
            cascadeExtendR.setPower(-0.8);
        } else if (gamepad1.b ) {
            cascadeExtendR.setPower(0.8);
        } else if(gamepad1.y){
            cascadeExtendL.setPower(-1);
            cascadeExtendLii.setPower(-1);
        } else if(gamepad1.x){
            cascadeExtendL.setPower(1);
            cascadeExtendLii.setPower(1);
        } else {
            cascadeExtendR.setPower(0);
            cascadeExtendL.setPower(0);
            cascadeExtendLii.setPower(0);
        }



        if(gamepad1.left_bumper) {
            clawL.setPosition(0.38);
        } else {
            clawL.setPosition(0.6);
        }

        if (gamepad1.right_bumper){
            clawR.setPosition(0.7);
        } else {
            clawR.setPosition(0.48);
        }
    }
}