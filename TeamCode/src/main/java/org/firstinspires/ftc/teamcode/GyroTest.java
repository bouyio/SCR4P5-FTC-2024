package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class GyroTest extends OpMode {
    private IMU imu;

    @Override
    public void init() {
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu = hardwareMap.get(IMU.class, "gyro");

        imu.initialize(myIMUparameters);



    }

    @Override
    public void loop() {
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Now use these simple methods to extract each angle
        // (Java type double) from the object you just created:
        double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

        telemetry.addData("yaw", yaw);
        telemetry.addData("pitch", pitch);
        telemetry.addData("roll", roll);


    }
}
