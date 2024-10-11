package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class FunkAhhMotorThing extends OpMode {

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
        if(gamepad1.a){
            m1.setPower(1.00);
            m2.setPower(1.00);
        } else if(gamepad1.b){
            m1.setPower(-1.00);
            m2.setPower(-1.00);
        } else {
            m1.setPower(0.00);
            m2.setPower(0.00);
        }
    }
}
