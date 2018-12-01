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

    //lift variables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop};
    private LiftState PrimliftState = LiftState.LiftBottom;
    private LiftState SecliftState = LiftState.LiftBottom;

    private boolean PrevPrimState = false;
    private long PrevPrimStateTime = 0;
    private boolean PrevSecState = false;
    private long PrevSecStateTime = 0;


    public Lift(OpMode opmodeIn, RoverRucusConfiguration configIn) {
        super();
        opmode = opmodeIn;
        config = configIn;
    }

    //Bumper Variables


    /* public double liftPrim(double value) {

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
                        PrimliftState = LiftState.LiftBottom;
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
                        SecliftState = LiftState.LiftBottom;
                    }
                    return value;
                case LiftBottom:
                    return 0;
            }
        }
        return value;
    }
    */

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

        config.prim_lift_motor.setPower(left_control);
        config.sec_lift_motor.setPower(right_control);



    }
}

