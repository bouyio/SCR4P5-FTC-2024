package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class Servoo extends OpMode {
    Servo serbian;


    @Override
    public void init() {
        serbian = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            serbian.setPosition(0.7);
        } else {
            serbian.setPosition(0.5);
        }
    }

}
