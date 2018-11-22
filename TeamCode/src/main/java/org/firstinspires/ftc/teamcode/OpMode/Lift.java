package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        config = configIn

    }

    //Bumper Variables


    public double rightBumper() {
        if (opmode.gamepad1.right_bumper) {
            return 1.0;
        }
        else {
            return 0.0;

        }
    }


    public double leftBumper() {
        if (opmode.gamepad1.left_bumper) {
            return 1.0;
        }
        else {
            return 0.0;

        }
    }

    // End of Bumper Variables


    public void updateLift(){

        double right_trigger = opmode.gamepad1.right_trigger - rightBumper();
        double left_trigger = opmode.gamepad1.left_trigger - leftBumper();

        config.prim_lift_motor.setPower(right_trigger);
        config.sec_lift_motor.setPower(left_trigger);

    }
}

