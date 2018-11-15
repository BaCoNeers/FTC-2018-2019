package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Baconeers on 11/11/2018.
 */

public class Lift {
    //members:
    private OpMode opmode = null;
    private DcMotor lift_motor = null;

    public Lift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        lift_motor  = opmode.hardwareMap.get(DcMotor.class, "lift_motor");

        lift_motor.setDirection(DcMotor.Direction.FORWARD);

        //Todo: complete the constructor
    }

    public void updateLift(){
        double lift_power;

        double right_bumper = opmode.gamepad1.right_trigger - opmode.gamepad1.left_trigger ;

        lift_motor.setPower(right_bumper);


        //Todo: complete the method
    }
}

