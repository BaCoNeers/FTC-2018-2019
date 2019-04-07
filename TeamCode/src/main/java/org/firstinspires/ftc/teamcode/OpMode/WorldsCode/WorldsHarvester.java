package org.firstinspires.ftc.teamcode.OpMode.WorldsCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.WorldsConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class WorldsHarvester {
    // members:
    private OpMode opmode = null;
    private WorldsConfiguration config = null;
    public boolean AbuttonState = false;
    public boolean AlastButtonState = false;
    public boolean BbuttonState = false;
    public boolean BlastButtonState = false;
    public boolean Astate = false;
    public boolean Bstate = false;
    public boolean toggleFunction = false;
    int direction = 1;

    public int direction(){
        BbuttonState = opmode.gamepad2.b;


        if (BbuttonState && !BlastButtonState) {
            Bstate = !Bstate;
        }

        if (BbuttonState != BlastButtonState) {
            BlastButtonState = BbuttonState;
        }

        if (Bstate) {
            return 1;
        }
        else {
            return -1;
        }
    }

    public double Toggle() {
        AbuttonState = opmode.gamepad2.a;


        if (AbuttonState && !AlastButtonState) {
            Astate = !Astate;
        }

        if (AbuttonState != AlastButtonState) {
            AlastButtonState = AbuttonState;
        }

        if (Astate) {
            return 1*direction;
        }
        else {
            return 0;
        }

    }


    public WorldsHarvester(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }

    float power;

    public void update(){

        power = opmode.gamepad2.right_trigger-opmode.gamepad2.left_trigger;
        power = Range.clip(power,-1,1);
        config.harvester_lift.setPower(power);

        config.harvester.setPower(Toggle());




    }
}
