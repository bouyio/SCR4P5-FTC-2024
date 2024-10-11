package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp()
public class WANNABEMOVEMENT extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Reverse the left motor if needed
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean turboBtn = gamepad1.right_bumper;

        // Gamepad left stick controls forward and backward movement
        double forward = -gamepad1.left_stick_y;
        // Gamepad right stick controls right and left movement
        double right = gamepad1.right_stick_x;

        // Calculate motor powers
        double leftPower = forward + right;
        double rightPower = forward - right;

        if(turboBtn) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        }else {
            accelerate(forward, right);
        }
        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }

    private void accelerate(double yStickValue, double xStickValue){
        if(!isStartingAccelerationTimeSet){
            startingAccelerationTime = (double) runtime.time(TimeUnit.SECONDS);
            isStartingAccelerationTimeSet = true;
        }

        if(!isTimeDiffSet){
            timeDiff = (double) runtime.time(TimeUnit.SECONDS) - startingAccelerationTime;
            if(timeDiff >= 2){
                isTimeDiffSet = true;
            }
        }


        accelerationMultiplier = (double) timeDiff / 2;

        if(yStickValue == 0 && xStickValue == 0){
            rightMotor.setPower(0.00);
            leftMotor.setPower(0.00);
            accelerationMultiplier = 0;
            isStartingAccelerationTimeSet = false;
            isTimeDiffSet = false;

            return;
        }

        rightMotor.setPower(((yStickValue + xStickValue) * accelerationMultiplier) * 0.5);
        leftMotor.setPower(((yStickValue - xStickValue) * accelerationMultiplier) * 0.5);
    }
}
