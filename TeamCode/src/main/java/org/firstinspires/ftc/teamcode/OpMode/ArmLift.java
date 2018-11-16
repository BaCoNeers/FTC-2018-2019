package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

public class ArmLift {
    //members:
    private OpMode opmode = null;
    private DcMotor arm_lift_motor = null;
    private CRServo prim_box_arm_servo = null;
    private CRServo sec_box_arm_servo = null;


    public ArmLift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        //arm_lift_motor  = opmode.hardwareMap.get(DcMotor.class, "arm_lift_motor");
        //prim_box_arm_servo = opmode.hardwareMap.get(CRServo.class, "prim_box_arm_servo");
        //sec_box_arm_servo = opmode.hardwareMap.get (CRServo.class, "sec_box_arm_servo");


        arm_lift_motor.setDirection(DcMotor.Direction.FORWARD);


    }

    //Bumper Variables

    double right_bumper;
    double left_bumper;
    double x_button;
    double y_button;


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



    // End of Bumper & X/Y Variables


    public void updateArmLift(){


        double arm_lift_power = right_bumper - left_bumper ;

        arm_lift_motor.setPower(arm_lift_power);

    }
}
