package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

public class ArmLift {
    //members:
    private OpMode opmode = null;
    private RoverRucusConfiguration config = null;
    // add members below to engineering notebook if used in code
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    private double motor_power;

    //lift variables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop};
    LiftState PrimliftState = LiftState.LiftBottom;
    LiftState SecliftState = LiftState.LiftBottom;

    boolean PrevPrimState = false;
    long PrevPrimStateTime = 0;
    boolean PrevSecState = false;
    long PrevSecStateTime = 0;

    //public static final double MAX = 0.45;
    //public static final double STOP = 0.5;
    //public static final double MIN = -0.45;

    // private boolean buttonState = false;
   // private boolean lastButtonState = false;



    public ArmLift(OpMode opmodeIn, RoverRucusConfiguration configIn) {
        super();
        opmode = opmodeIn;
        config = configIn;

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


        config.arm_lift_motor.setPower(arm_lift_power);
        config.prim_box_arm_servo.setPower(arm_servo_power);
        config.sec_box_arm_servo.setPower(-arm_servo_power);


    }
}
