package org.firstinspires.ftc.teamcode.Configuration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotConfiguration;

/**
 * It is assumed that there is a configuration that is currently activated on the robot controller
 * (run menu / Configure Robot ) with the same name as this class.
 * It is also assumed that the device names in the 'init()' method below are the same as the devices
 * named on the activated configuration on the robot.
 */
public class WorldsConfiguration extends RobotConfiguration {

    public DcMotor front_left_motor = null;
    public DcMotor front_right_motor = null;
    public DcMotor rear_left_motor = null;
    public DcMotor rear_right_motor = null;

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {
        setTelemetry(telemetry);


        front_left_motor  = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        front_right_motor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        rear_left_motor = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        rear_right_motor = hardwareMap.get(DcMotor.class,"BackRightMotor");

        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        rear_left_motor.setDirection(DcMotor.Direction.REVERSE);
        rear_right_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Initialized","True");
        telemetry.update();
    }
    //down 1.35
    //up 0.8

    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetry
     * @return
     */
    public static WorldsConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        WorldsConfiguration config = new WorldsConfiguration();
        config.init(hardwareMap, telemetry);
        return config;

    }


}
