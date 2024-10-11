package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class ServoAll extends OpMode {
    Servo serbian;
    Servo bersian;


    @Override
    public void init() {
        serbian = hardwareMap.get(Servo.class, "servo");
        bersian = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {
        if(gamepad1.b) {
            serbian.setPosition(0.48);
        } else {
            serbian.setPosition(0.7);
        }

        if (gamepad1.a){
            bersian.setPosition(0.7);
        } else {
            bersian.setPosition(0.48);
        }
    }

}
