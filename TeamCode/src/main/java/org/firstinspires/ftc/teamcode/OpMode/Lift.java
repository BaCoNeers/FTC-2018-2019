package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Baconeers on 11/11/2018.
 */

public class PrimLift {
    //members:
    private OpMode opmode = null;
    private DcMotor prim_lift_motor = null;

    public PrimLift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        prim_lift_motor  = opmode.hardwareMap.get(DcMotor.class, "prim_lift_motor");

        prim_lift_motor.setDirection(DcMotor.Direction.FORWARD);
    }


    public void updatePrimLift(){
        double prim_lift_power;
        double right_bumper;

        public void RightBumper() {
            if (opmode.gamepad1.left_bumper) {
                right_bumper = 1.0;
            }
            else {
                right_bumper = 0.0;

            }
        }
        double right_trigger = opmode.gamepad1.left_trigger - right_bumper ;

        prim_lift_motor.setPower(right_trigger);

    }
}

