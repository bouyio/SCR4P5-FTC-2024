package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class TestDrive extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    @Override
    public void init() {
        frontRight=hardwareMap.get(DcMotor.class,"front_right_motor");
        frontLeft=hardwareMap.get(DcMotor.class,"front_left_motor");
        backRight=hardwareMap.get(DcMotor.class,"back_right_motor");
        backLeft=hardwareMap.get(DcMotor.class,"back_left_motor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {




    }
}
