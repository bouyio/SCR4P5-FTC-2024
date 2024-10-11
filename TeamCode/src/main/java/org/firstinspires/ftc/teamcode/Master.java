package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp()
public class Master extends OpMode {


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
    //DigitalChannel touchSensor;


    DcMotor cascadeRotateL;
    DcMotor cascadeRotateR;
    CRServo cascadeExtendL;
    CRServo cascadeExtendLii;
    DcMotor cascadeExtendR;
    Servo clawL;
    Servo clawR;
    Servo clawRotate;
    Servo droneServo;


    @Override
    public void init() {

        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        cascadeRotateL = hardwareMap.get(DcMotor.class, "CRL");
        cascadeRotateR = hardwareMap.get(DcMotor.class, "CRR");
        cascadeRotateL.setDirection(DcMotorSimple.Direction.REVERSE);

        cascadeExtendL = hardwareMap.get(CRServo.class, "CEL");
        cascadeExtendLii = hardwareMap.get(CRServo.class, "CELii");
        cascadeExtendR = hardwareMap.get(DcMotor.class, "CER");
        cascadeExtendL.setDirection(DcMotorSimple.Direction.REVERSE);

        clawL = hardwareMap.get(Servo.class, "cL");
        clawR = hardwareMap.get(Servo.class, "cR");
        clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        droneServo= hardwareMap.get(Servo.class,"droneServo");

        //touchSensor = hardwareMap.get(DigitalChannel.class, "touch");
    }

    @Override
    public void loop() {

        isTurboEnabled = gamepad1.dpad_left;


        lStickX = -gamepad1.left_stick_x;
        lStickY = gamepad1.left_stick_y;
        rStickX = -gamepad1.right_stick_x;

        rFrontMotorPower = lStickY+lStickX+rStickX;
        lFrontMotorPower = lStickY-lStickX-rStickX;
        lBackMotorPower = -lStickY-lStickX+rStickX;
        rBackMotorPower = -lStickY+lStickX-rStickX;

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
        }
        else {
            backLeftMotor.setPower(lBackMotorPower);
            backRightMotor.setPower(rBackMotorPower);
            frontLeftMotor.setPower(lFrontMotorPower);
            frontRightMotor.setPower(rFrontMotorPower);
        }


        cascadeRotateL.setPower(gamepad2.left_stick_y * 0.5);
        cascadeRotateR.setPower(gamepad2.left_stick_y * 0.5);



        if (gamepad2.dpad_up ){
            clawRotate.setPosition(0.70);
        }
        if (gamepad2.dpad_down){

            clawRotate.setPosition(0);
        }



        if (gamepad2.left_stick_y !=0){
            clawRotate.setPosition(gamepad2.right_stick_y);
        }


        if (gamepad2.a) {
            cascadeExtendR.setPower(-0.5);
            cascadeExtendL.setPower(1);
            cascadeExtendLii.setPower(1);
        } else if (gamepad2.b) {   //&& !touchSensor.getState()
            cascadeExtendR.setPower(0.4);
            cascadeExtendL.setPower(-1);
            cascadeExtendLii.setPower(-1);
        }else if(gamepad2.y){
            cascadeExtendL.setPower(1);
            cascadeExtendLii.setPower(1);
        } else{
            cascadeExtendR.setPower(0);
            cascadeExtendL.setPower(0);
            cascadeExtendLii.setPower(0);
        }

        if(gamepad1.dpad_right){
            droneServo.setPosition(0);
        }
        else{
            droneServo.setPosition(0.27);
        }

        if(gamepad2.right_stick_y !=0){
            clawRotate.setPosition(gamepad2.right_stick_y);
        }

        if(gamepad1.left_bumper) {
            //clawL.setPosition(0.6); old
            clawL.setPosition(0.7);
        } else { // ok values do not change
            //clawL.setPosition(0.38); old
            clawL.setPosition(0.48);
        }

        if (gamepad1.right_bumper){
            //clawR.setPosition(0.48);
            clawR.setPosition(0.38);
        } else {
            //clawR.setPosition(0.7);
                clawR.setPosition(0.6);
        }
    }
}