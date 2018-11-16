package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Baconeers on 16/11/2018.
 */

public class SecLift {
    //members:
    private OpMode opmode = null;
    private DcMotor sec_lift_motor = null;

    public SecLift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        sec_lift_motor  = opmode.hardwareMap.get(DcMotor.class, "sec_lift_motor");

        sec_lift_motor.setDirection(DcMotor.Direction.FORWARD);

    }



    public void updateSecLift(){
        double sec_lift_power;
        double left_bumper;

        private public void LeftBumper() {
            if (opmode.gamepad1.left_bumper) {
            left_bumper = 1.0;
            }
            else {
            left_bumper = 0.0;

            }
        }
        double left_trigger = opmode.gamepad1.left_trigger - left_bumper ;

        sec_lift_motor.setPower(left_trigger);

    }
}

