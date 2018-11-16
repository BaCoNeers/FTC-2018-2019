package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Baconeers on 11/11/2018.
 */
// change gamepad to gamepad1 when testing or gamepad when competing

public class Lift {
    //members:
    private OpMode opmode = null;
    private DcMotor prim_lift_motor = null;
    private DcMotor sec_lift_motor = null;


    public Lift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        prim_lift_motor  = opmode.hardwareMap.get(DcMotor.class, "prim_lift_motor");
        sec_lift_motor = opmode.hardwareMap.get(DcMotor.class, "sec_lift_motor");

        prim_lift_motor.setDirection(DcMotor.Direction.REVERSE);
        sec_lift_motor.setDirection(DcMotor.Direction.FORWARD);

        prim_lift_motor.setPower(0);
        sec_lift_motor.setPower(0);
    }

    //Bumper Variables

    double right_bumper;
    double left_bumper;


    public void RightBumper() {
        if (opmode.gamepad1.right_bumper) {
            right_bumper = 1.0;
        }
        else {
            right_bumper = 0.0;

        }
    }


    public void LeftBumper() {
        if (opmode.gamepad1.left_bumper) {
            left_bumper = 1.0;
        }
        else {
            left_bumper = 0.0;

        }
    }

    // End of Bumper Variables


    public void updateLift(){
        double prim_lift_power;
        double sec_lift_power;


        double right_trigger = opmode.gamepad1.right_trigger - right_bumper;
        double left_trigger = opmode.gamepad1.left_trigger - left_bumper;

        prim_lift_motor.setPower(right_trigger);
        sec_lift_motor.setPower(left_trigger);

    }
}

