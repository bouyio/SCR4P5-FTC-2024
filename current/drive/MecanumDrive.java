package org.firstinspires.ftc.teamcode;
//Imports:
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class MecanumDrive extends OpMode {

    //Setting variables for the motors
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;

    //Setting variables for the joystick
    private double lStickX;
    private double lStickY;
    private double rStickX;
    
    //Setting variables for the motors Power
    private double rFrontMotorPower;
    private double lFrontMotorPower;
    private double rBackMotorPower;
    private double lBackMotorPower;

    private double max;

    //Setting variables for accelerations
    private double accelerationMultiplier;
    private double timeDiff;
    private boolean isTimeDiffSet = false;
    private boolean isStartingAccelerationTimeSet = false;
    private double startingAccelerationTime;

    private boolean isTurboEnabled;


    @Override
    public void init() {
        //Links the motors to the motor variables
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");

        //setting some motors Direction to reverse so all motors move to the same direction
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //Sets the left bumper as the turbo
        isTurboEnabled = gamepad1.left_bumper;

        //Sets joystick variables
        lStickX = gamepad1.left_stick_x;
        lStickY = gamepad1.left_stick_y;
        rStickX = gamepad1.right_stick_x;

        //Setting the motor power variables based on the joysticks
        lFrontMotorPower = lStickY-lStickX-rStickX;
        lBackMotorPower = lStickY+lStickX-rStickX;
        rBackMotorPower = lStickY-lStickX+rStickX;
        rFrontMotorPower = lStickY+lStickX+rStickX;

        //Sets the max variables for further usage
        max = Math.max(Math.abs(lFrontMotorPower), Math.abs(lBackMotorPower));

        max = Math.max(max, Math.abs(rBackMotorPower));

        max = Math.max(max, Math.abs(rFrontMotorPower));
        
        // fixes motor power if it's over the limits [-1,1]
        if(max>1.0) {
            lFrontMotorPower = lFrontMotorPower / max;
            lBackMotorPower = lBackMotorPower / max;
            rFrontMotorPower = rFrontMotorPower / max;
            rBackMotorPower = rBackMotorPower / max;
        }

        //Checks if turbo is not enabled, if it is setting the power to all the motors after multiplying them by 0.7
        if(!isTurboEnabled) {
            backLeftMotor.setPower(lBackMotorPower * 0.7);
            backRightMotor.setPower(rBackMotorPower * 0.7);
            frontLeftMotor.setPower(lFrontMotorPower * 0.7);
            frontRightMotor.setPower(rFrontMotorPower * 0.7);
            return;
        }
        //Setting the power to all the motors
        backLeftMotor.setPower(lBackMotorPower);
        backRightMotor.setPower(rBackMotorPower);
        frontLeftMotor.setPower(lFrontMotorPower);
        frontRightMotor.setPower(rFrontMotorPower);
    }






}
