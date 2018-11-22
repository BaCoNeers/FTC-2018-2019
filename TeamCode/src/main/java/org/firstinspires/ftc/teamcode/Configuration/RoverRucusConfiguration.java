package org.firstinspires.ftc.teamcode.Configuration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotConfiguration;

/**
 * It is assumed that there is a configuration that is currently activated on the robot controller
 * (run menu / Configure Robot ) with the same name as this class.
 * It is also assumed that the device names in the 'init()' method below are the same as the devices
 * named on the activated configuration on the robot.
 */
public class RoverRucusConfiguration extends RobotConfiguration {

    public DcMotor front_left_motor = null;
    public DcMotor front_right_motor = null;
    public DcMotor rear_left_motor = null;
    public DcMotor rear_right_motor = null;
    public DcMotor prim_lift_motor = null;
    public DcMotor sec_lift_motor = null;
    public DcMotor arm_lift_motor = null;
    public CRServo prim_box_arm_servo = null;
    public CRServo sec_box_arm_servo = null;

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     * @param hardwareMap
     * @param telemetry
     */
    @Override
    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {

        setTelemetry(telemetry);

        front_left_motor  = hardwareMap.get(DcMotor.class, "front_left_drive");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_drive");
        rear_left_motor = hardwareMap.get(DcMotor.class,"rear_left_drive");
        rear_right_motor = hardwareMap.get(DcMotor.class,"rear_right_drive");

        front_left_motor.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        rear_left_motor.setDirection(DcMotor.Direction.FORWARD);
        rear_right_motor.setDirection(DcMotor.Direction.REVERSE);

        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rear_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prim_lift_motor  = hardwareMap.get(DcMotor.class, "prim_lift_motor");
        sec_lift_motor = hardwareMap.get(DcMotor.class, "sec_lift_motor");

        prim_lift_motor.setDirection(DcMotor.Direction.REVERSE);
        sec_lift_motor.setDirection(DcMotor.Direction.FORWARD);

        prim_lift_motor.setPower(0);
        sec_lift_motor.setPower(0);

        arm_lift_motor  = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        prim_box_arm_servo = hardwareMap.get(CRServo.class, "prim_box_arm_servo");
        sec_box_arm_servo = hardwareMap.get (CRServo.class, "sec_box_arm_servo");


        arm_lift_motor.setDirection(DcMotor.Direction.FORWARD);



        telemetry.addData("Initialized","True");
        telemetry.update();
    }


    /**
     * Factory method for this class
     *
     * @param hardwareMap
     * @param telemetry
     * @return
     */
    public static RoverRucusConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        RoverRucusConfiguration config = new RoverRucusConfiguration();
        config.init(hardwareMap, telemetry);
        return config;

    }


}
