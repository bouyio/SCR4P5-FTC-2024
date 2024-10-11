
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp()
public class SCR4P5OPMode extends OpMode {
    /* Declare OpMode members. */
    DcMotor right_motor;
    DcMotor left_motor;

    private double LStickY = gamepad1.left_stick_y;
    private double RStickX = gamepad1.right_stick_x;

    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;


    @Override
    public void init() {


        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

    }





    @Override
    public void loop() {
        boolean turboMode = gamepad1.left_bumper;

        LStickY = gamepad1.left_stick_y;
        RStickX = gamepad1.right_stick_x;
        telemetry.addData("time difference", timeDiff);
        telemetry.addData("acceleration multiplier", accelerationMultiplier);
        telemetry.addData("left stick y", LStickY);
        telemetry.addData("motor R", right_motor.getPower());
        telemetry.addData("motor L", left_motor.getPower());
        if(!turboMode){
            accelerate();

        }else {
            right_motor.setPower(-LStickY + RStickX);
            left_motor.setPower(-LStickY - RStickX);

        }


    }




    private void accelerate(){
        if(!isStartingAccelerationTimeSet){
            startingAccelerationTime = (double) ElapsedTime.SECOND_IN_NANO / 1000000000;
            isStartingAccelerationTimeSet = true;
        }

        if(!isTimeDiffSet){
            timeDiff = (double) ElapsedTime.MILLIS_IN_NANO / 1000000000 - startingAccelerationTime;
            if(timeDiff >= 3){
                isTimeDiffSet = true;
            }
        }


        accelerationMultiplier = (double) timeDiff / 3;

        if(LStickY == 0){
            right_motor.setPower(0.00);
            left_motor.setPower(0.00);
            accelerationMultiplier = 0;
            isStartingAccelerationTimeSet = false;
            isTimeDiffSet = false;

            return;
        }






        right_motor.setPower(-LStickY + RStickX * accelerationMultiplier);
        left_motor.setPower(-LStickY - RStickX * accelerationMultiplier);
    }
}