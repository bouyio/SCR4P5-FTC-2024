
/*
*   This code was written before the kit arrived.
*   So, it was written and tested entirely using
*   simulations. Written to test the movement of
*   our at the time driving base in October 27 of 
*   2023.
*/

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp()
public class SCR4P5MovementOPMode extends OpMode {
    /* Declare OpMode members. */
    DcMotor right_motor;
    DcMotor left_motor;

    // Declare Y and X of gamepads (Controller) joysticks
    private double LStickY = gamepad1.left_stick_y;
    private double RStickX = gamepad1.right_stick_x;

    //Declare variables used for the acceleration
    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;


    @Override
    public void init() {

        //setting motors to the respective variables
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        //Setting one of the motors to reverse
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

    }





    @Override
    public void loop() {
        //Checks if boost button is pressed
        boolean turboMode = gamepad1.left_bumper;
        //Sets the values of left and right controller joysticks 
        LStickY = gamepad1.left_stick_y;
        RStickX = gamepad1.right_stick_x;
        //Prints motor functions to telemetry
        telemetry.addData("time difference", timeDiff);
        telemetry.addData("acceleration multiplier", accelerationMultiplier);
        telemetry.addData("left stick y", LStickY);
        telemetry.addData("motor R", right_motor.getPower());
        telemetry.addData("motor L", left_motor.getPower());
        
        //Checks whether to accelerarate or to run motors at full speed
        if(!turboMode){
            accelerate();

        }else {
            
            right_motor.setPower(-LStickY + RStickX);
            left_motor.setPower(-LStickY - RStickX);

        }


    }




    private void accelerate(){
        //Checks if it has not already started accelerating
        if(!isStartingAccelerationTimeSet){
            startingAccelerationTime = (double) ElapsedTime.SECOND_IN_NANO / 1000000000;
            isStartingAccelerationTimeSet = true;
        }

        //Checks time difference from the beggining of the acceleration if it is false 
        if(!isTimeDiffSet){
            timeDiff = (double) ElapsedTime.MILLIS_IN_NANO / 1000000000 - startingAccelerationTime;
            if(timeDiff >= 3){
                isTimeDiffSet = true;
            }
        }

        //Sets the accelerationMultiplier 
        accelerationMultiplier = (double) timeDiff / 3;

        //Checks if the Y of the left Joystick is 0
        if(LStickY == 0
            //Sets motor power to 0 and resets acceleration
            right_motor.setPower(0.00);
            left_motor.setPower(0.00);
            accelerationMultiplier = 0;
            isStartingAccelerationTimeSet = false;
            isTimeDiffSet = false;

            return;
        }





        //Setting motor power that is multiplied by accelerationMultiplier
        right_motor.setPower(-LStickY + RStickX * accelerationMultiplier);
        left_motor.setPower(-LStickY - RStickX * accelerationMultiplier);
    }
}
