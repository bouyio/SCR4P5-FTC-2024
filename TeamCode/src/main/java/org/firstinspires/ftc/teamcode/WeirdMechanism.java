package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class WeirdMechanism extends OpMode {
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor intake;
    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            m2.setPower(1.00);
            m1.setPower(1.00);
        }else{
            m2.setPower(0);
            m1.setPower(0);
        }

        if(gamepad1.left_bumper){
            intake.setPower(1.00);
        }else {
            intake.setPower(0);
        }

    }
}
