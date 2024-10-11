package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class TonKosmoGyrizw extends OpMode {
    Servo s;
    @Override
    public void init() {
        s = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        s.setPosition(s.getPosition() + 1);
    }
}
