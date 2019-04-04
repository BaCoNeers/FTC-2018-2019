package org.firstinspires.ftc.teamcode.OpMode.WorldsCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;
import org.firstinspires.ftc.teamcode.Configuration.WorldsConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class WorldsLatchLift {
    // members:
    private OpMode opmode = null;
    private WorldsConfiguration config = null;



    public WorldsLatchLift(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }

    public float power(){
        if(opmode.gamepad1.y == true && opmode.gamepad1.x==false){
            return 1f;
        }
        else if(opmode.gamepad1.x == true && opmode.gamepad1.y ==false){
            return -1f;
        }
        else if(opmode.gamepad1.x == true && opmode.gamepad1.y == true){
            return 1f;
        }
        return 0;
    }

    enum LiftState {LiftBottom,LiftMiddle,LiftTop}
    private LiftState PrimliftState = LiftState.LiftBottom;
    private boolean PrevPrimState = false;
    private long PrevPrimStateTime = 0;

    public void update(){

        long CurrentTime = System.nanoTime();

        boolean PrimTimeElapsed = (PrevPrimStateTime + 200000000) < CurrentTime;

        boolean PrimStateLowToHigh = PrimTimeElapsed && !PrevPrimState && config.latch_limit_switch.getState();
        boolean PrimStateHighToLow = PrimTimeElapsed && PrevPrimState && !config.latch_limit_switch.getState();


        if(PrimStateHighToLow || PrimStateLowToHigh) {
            PrevPrimState = config.latch_limit_switch.getState();
            PrevPrimStateTime = CurrentTime;
        }

        if (opmode.gamepad1.y && !opmode.gamepad1.x) {
            // Going up to top
            switch (PrimliftState) {
                case LiftBottom:
                    config.latch_lift.setPower(1);
                    if (PrimStateLowToHigh) {
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.latch_lift.setPower(1);
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    break;
                default:
                    break;
            }
        } else if(opmode.gamepad1.x && !opmode.gamepad1.y) {
            //Going Down
            switch (PrimliftState){
                case LiftTop:
                    config.latch_lift.setPower(-1);
                    if(PrimStateLowToHigh){
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.latch_lift.setPower(-1);
                    if(PrimStateHighToLow){
                        PrimliftState = LiftState.LiftBottom;
                    }
                    break;
                default:
                    break;
            }
        }
        else{
            config.latch_lift.setPower(0);
        }


        //To use incase lift is faulty, comment out code above and uncomment out line below
        //config.latch_lift.setPower(power());
    }
}
