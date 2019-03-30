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
        else{
            return 1f;
        }
    }

    public void update(){
        config.latch_lift.setPower(power());
    }
}
