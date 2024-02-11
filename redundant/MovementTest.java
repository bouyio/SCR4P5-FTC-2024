
/*
*   This code was written in December 3rd
*   of 2023 for at the time driving base
*   after the previous code started to 
*   throw up an unfixable error.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp()
public class MovementTest extends OpMode {
    // This section decrales OpMode members.

    // The declaration of the motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    
    // These are the fields required for the acceleration.
    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Initialization of devices.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Update of the controller inputs.
        
        boolean turboBtn = gamepad1.right_bumper;

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.right_stick_x;

        // This calculates the necessery power for the motors.
        double leftPower = forward + right;
        double rightPower = forward - right;

        // This check whether to accelarate or run the motor at full speed.
        if(turboBtn) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
        }else {
            accelerate(forward, right);
        }

        // This updates the data needed to debug.
        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }

    // This function handles the acceleration of the robot.
    private void accelerate(double yStickValue, double xStickValue){
        /*
        * This is a guard set in place to initialize the necessary variables
        * for the acceleration when the driver stops the robot.
        */
        if(yStickValue == 0 && xStickValue == 0){
            rightMotor.setPower(0.00);
            leftMotor.setPower(0.00);
            accelerationMultiplier = 0;
            isStartingAccelerationTimeSet = false;
            isTimeDiffSet = false;

            return;
        }

        /*
        * This is a check to make sure that the time the robot
        * has started accelerating has been set.
        */
        if(!isStartingAccelerationTimeSet){
            startingAccelerationTime = (double) runtime.time(TimeUnit.SECONDS);
            isStartingAccelerationTimeSet = true;
        }

        //Checks time difference from the beggining of the acceleration if it is false 
        if(!isTimeDiffSet){
            timeDiff = (double) runtime.time(TimeUnit.SECONDS) - startingAccelerationTime;
            if(timeDiff >= 2){
                isTimeDiffSet = true;
            }
        }

        //Sets the accelerationMultiplier 
        accelerationMultiplier = (double) timeDiff / 2;


         //Setting motor power that is multiplied by accelerationMultiplier
        rightMotor.setPower(((yStickValue + xStickValue) * accelerationMultiplier) * 0.5);
        leftMotor.setPower(((yStickValue - xStickValue) * accelerationMultiplier) * 0.5);
    }
}
