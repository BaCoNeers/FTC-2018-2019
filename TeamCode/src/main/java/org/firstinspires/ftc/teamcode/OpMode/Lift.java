package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;


/**
 * Created by Baconeers on 11/11/2018.
 * This class is the program for the main lift of the robot. This allows for the robot to attach itself to the launcher.
 */
// change gamepad to gamepad1 when testing or gamepad when competing

public class Lift {
    //members:
    private OpMode opmode = null;
    private RoverRucusConfiguration config = null;


    public Lift(OpMode opmodeIn, RoverRucusConfiguration configIn) {
        super();
        opmode = opmodeIn;
        config = configIn;
    }


    public void updateLift(){

        //Get left stick y value and assign to lift motor
        double left_control = - opmode.gamepad2.left_stick_y;
        config.robot_lift_motor.setPower(left_control);




    }
}

