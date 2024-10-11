package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lesson extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;



    @Override
    public void stop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void init() {

        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double lstickx = gamepad1.left_stick_x;
        double lsticky = gamepad1.left_stick_y;
        leftMotor.setPower(lsticky - lstickx);
        rightMotor.setPower(lsticky + lstickx);

    }
}
