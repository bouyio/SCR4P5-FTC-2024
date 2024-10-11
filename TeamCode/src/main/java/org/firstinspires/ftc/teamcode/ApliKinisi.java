package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous()
public class ApliKinisi extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(550);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(-0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);
        try {
            sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    @Override
    public void loop() {

    }
}
