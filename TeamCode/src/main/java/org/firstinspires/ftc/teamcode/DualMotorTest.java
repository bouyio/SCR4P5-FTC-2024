package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class DualMotorTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "1");
        motor2 = hardwareMap.get(DcMotor.class, "2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            motor2.setPower(1.00);
            motor1.setPower(1.00);
        }

        if(gamepad1.b){
            motor2.setPower(-1.00);
            motor1.setPower(-1.00);
        }

        motor2.setPower(0);
        motor1.setPower(0);
    }
}
