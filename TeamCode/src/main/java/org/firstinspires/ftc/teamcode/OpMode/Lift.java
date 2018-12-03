package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

import static org.firstinspires.ftc.teamcode.OpMode.Lift.LiftState.LiftBottom;


/**
 * Created by Baconeers on 11/11/2018.
 * This class is the program for the main lift of the robot. This allows for the robot to attach itself to the launcher.
 */
// change gamepad to gamepad1 when testing or gamepad when competing

public class Lift {
    //members:
    private OpMode opmode = null;
    private RoverRucusConfiguration config = null;

    //lift variables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop,None};
    private LiftState PrimliftState = LiftBottom;
    private LiftState SecliftState = LiftBottom;

    private boolean PrevPrimState = false;
    private long PrevPrimStateTime = 0;
    private boolean PrevSecState = false;
    private long PrevSecStateTime = 0;

    public boolean buttonState_a = false;
    public boolean lastButtonState_a = false;
    public boolean state_a = false;
    public boolean toggleFunction = false;

    public boolean buttonState_y = false;
    public boolean lastButtonState_y = false;
    public boolean state_y = false;

    public boolean buttonState_b = false;
    public boolean lastButtonState_b = false;
    public boolean state_b = false;



    private double Toggle() {
        //button a (bottom)
        buttonState_a = opmode.gamepad2.a;
        if (buttonState_a && !lastButtonState_a) {
            state_a = !state_a;
        }

        if (buttonState_a != lastButtonState_a) {
            lastButtonState_a = buttonState_a;
        }

        if (state_a) {
            //On state
            toggleFunction = true;

        }
        else {
            //Off state

            return 1.0;

        }

        //button y (top)
        buttonState_y = opmode.gamepad2.y;
        if (buttonState_y && !lastButtonState_y) {
            state_y = !state_y;
        }

        if (buttonState_y != lastButtonState_y) {
            lastButtonState_y = buttonState_y;
        }

        if (state_y) {
            //On state
            toggleFunction = true;

        }
        else {
            //Off state

            return 1.0;

        }

        // button b (middle)
        buttonState_b = opmode.gamepad2.b;
        if (buttonState_b && !lastButtonState_b) {
            state_b = !state_b;
        }

        if (buttonState_b != lastButtonState_b) {
            lastButtonState_b = buttonState_b;
        }

        if (state_b) {
            //On state
            toggleFunction = true;

        }
        else {
            //Off state

            return 1.0;

        }

        if (toggleFunction) {

            if (opmode.gamepad1.left_stick_x > 0.3) {
                return 0.5;
            }

            if (opmode.gamepad1.left_stick_x < -0.3) {
                return 0.5;
            }

            else {
                return 0.2;
            }

        }
        return 1.0;
    }





    public Lift(OpMode opmodeIn, RoverRucusConfiguration configIn) {
        super();
        opmode = opmodeIn;
        config = configIn;
    }

    //Bumper Variables


    public double liftPrim(double value) {

        long CurrentTime = System.nanoTime();

        boolean PrimTimeElapsed = (PrevPrimStateTime + 200000000) < CurrentTime;

        boolean PrimStateLowToHigh = PrimTimeElapsed && !PrevPrimState && config.PrimLimitSwitch.getState();
        boolean PrimStateHighToLow = PrimTimeElapsed && PrevPrimState && !config.PrimLimitSwitch.getState();

        boolean state = value > 0;



        if(PrimStateHighToLow || PrimStateLowToHigh) {
            PrevPrimState = config.PrimLimitSwitch.getState();
            PrevPrimStateTime = CurrentTime;
        }


        if (state) {
            // Going up to top
            switch (PrimliftState) {
                case LiftBottom:
                    if (PrimStateLowToHigh) {
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    return value;
                case LiftMiddle:
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    return value;
                case LiftTop:
                    return 0;
            }
        } else {
            //Going Down
            switch (PrimliftState){
                case LiftTop:
                    if(PrimStateLowToHigh){
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    return value;
                case LiftMiddle:
                    if(PrimStateHighToLow){
                        PrimliftState = LiftBottom;
                    }
                    return value;
                case LiftBottom:
                    return 0;
            }
        }
        return value;
    }

    public double liftSec(double value) {

        long CurrentTime = System.nanoTime();

        boolean SecTimeElapsed = (PrevSecStateTime + 200000000) < CurrentTime;

        boolean SecStateLowToHigh = SecTimeElapsed && !PrevSecState && config.SecLimitSwitch.getState();
        boolean SecStateHighToLow = SecTimeElapsed && PrevSecState && !config.SecLimitSwitch.getState();

        boolean state = value > 0;

        if(SecStateHighToLow || SecStateLowToHigh) {
            PrevSecState = config.SecLimitSwitch.getState();
            PrevSecStateTime = CurrentTime;
        }


        if (state) {
            // Going up to top
            switch (SecliftState) {
                case LiftBottom:
                    if (SecStateLowToHigh) {
                        SecliftState = LiftState.LiftMiddle;
                    }
                    return value;
                case LiftMiddle:
                    if (SecStateHighToLow) {
                        SecliftState = LiftState.LiftTop;
                    }
                    return value;
                case LiftTop:
                    return 0;
            }
        } else {
            //Going Down
            switch (SecliftState){
                case LiftTop:
                    if(SecStateLowToHigh){
                        SecliftState = LiftState.LiftMiddle;
                    }
                    return value;
                case LiftMiddle:
                    if(SecStateHighToLow){
                        SecliftState = LiftBottom;
                    }
                    return value;
                case LiftBottom:
                    return 0;
            }
        }
        return value;
    }

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

        double right_control = -opmode.gamepad2.right_stick_y;
        double left_control = - opmode.gamepad2.left_stick_y;

        config.prim_lift_motor.setPower(liftPrim(left_control));
        config.sec_lift_motor.setPower(liftSec(right_control));



    }
}

