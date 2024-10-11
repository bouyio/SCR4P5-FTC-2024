package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "saita")
public class Saita extends OpMode {
    DcMotor saitaMotor;
    @Override
    public void init() {
        saitaMotor = hardwareMap.get(DcMotor.class, "saita_motor");
    }

    @Override
    public void loop() {
        boolean saitaDispenserSwitch = gamepad1.left_bumper;
        if (saitaDispenserSwitch){
            saitaMotor.setPower(1);
        }
    }
}
