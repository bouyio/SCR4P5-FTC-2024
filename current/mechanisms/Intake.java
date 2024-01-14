package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class Intake extends OpMode {

    private DcMotor intake;

    private boolean activateIntake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        activateIntake = gamepad1.dpad_up;

        if (activateIntake){
            intake.setPower(-0.5);
            try {
                sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intake.setPower(0);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intake.setPower(0.5);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intake.setPower(0);

        }
    }
}
