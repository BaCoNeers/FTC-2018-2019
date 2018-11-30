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

    //lift variables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop};
    LiftState PrimliftState = LiftState.LiftBottom;
    LiftState SecliftState = LiftState.LiftBottom;

    boolean PrevPrimState = false;
    long PrevPrimStateTime = 0;
    boolean PrevSecState = false;
    long PrevSecStateTime = 0;


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
                    return -value;
                case LiftMiddle:
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    return -value;
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
        return -value;
    }

    public double liftSec(double value) {

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
                    return -value;
                case LiftMiddle:
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    return -value;
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
        return -value;
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

        double right_trigger = opmode.gamepad1.right_trigger - rightBumper();
        double left_trigger = opmode.gamepad1.left_trigger - leftBumper();

        config.prim_lift_motor.setPower(liftPrim(-right_trigger));
        config.sec_lift_motor.setPower(liftSec(left_trigger));

    }
}

