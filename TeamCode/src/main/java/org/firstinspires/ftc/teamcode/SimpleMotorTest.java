package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class SimpleMotorTest extends OpMode {
    DcMotor m1;
    DcMotor m2;
    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotor.class, "1");
        m2 = hardwareMap.get(DcMotor.class, "2");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        m1.setPower(gamepad1.left_stick_y);
        m2.setPower(gamepad1.left_stick_y);
    }
}
