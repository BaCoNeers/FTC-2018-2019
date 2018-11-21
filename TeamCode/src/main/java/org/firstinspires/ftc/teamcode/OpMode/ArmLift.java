package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

public class ArmLift {
    //members:
    private OpMode opmode = null;
    private DcMotor arm_lift_motor = null;
    private CRServo prim_box_arm_servo = null;
    private CRServo sec_box_arm_servo = null;
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    private double motor_power;

    //public static final double MAX = 0.45;
    //public static final double STOP = 0.5;
    //public static final double MIN = -0.45;

    // private boolean buttonState = false;
   // private boolean lastButtonState = false;



    public ArmLift(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        arm_lift_motor  = opmode.hardwareMap.get(DcMotor.class, "arm_lift_motor");
        prim_box_arm_servo = opmode.hardwareMap.get(CRServo.class, "prim_box_arm_servo");
        sec_box_arm_servo = opmode.hardwareMap.get (CRServo.class, "sec_box_arm_servo");


        arm_lift_motor.setDirection(DcMotor.Direction.FORWARD);


    }

    //Bumper/Trigger Variables

    //private double left_trigger = -opmode.gamepad2.left_trigger;
    //private double right_trigger = opmode.gamepad2.right_trigger;


    private double rightBumper() {
        if (opmode.gamepad2.right_bumper) {
            return 1.0;
        }
        else {
            return 0.0;

        }
    }


    private double leftBumper() {
        if (opmode.gamepad2.left_bumper) {
            return 1.0;
        }
        else {
            return 0.0;

        }
    }


    // End of Bumper & X/Y Variables


    public void updateArmLift(){


        double arm_lift_power = rightBumper() - leftBumper();
        double arm_servo_power = opmode.gamepad2.right_trigger - opmode.gamepad2.left_trigger;


        arm_lift_motor.setPower(arm_lift_power);
        prim_box_arm_servo.setPower(arm_servo_power);
        sec_box_arm_servo.setPower(-arm_servo_power);


    }
}
