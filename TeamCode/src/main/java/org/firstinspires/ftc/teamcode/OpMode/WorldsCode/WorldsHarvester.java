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
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    public boolean toggleFunction = false;

    public double Toggle() {
        buttonState = opmode.gamepad1.y;
        if (buttonState && !lastButtonState) {
            state = !state;
        }

        if (buttonState != lastButtonState) {
            lastButtonState = buttonState;
        }

        if (state) {
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


    public WorldsHarvester(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }

    public void update(){

        float power = opmode.gamepad2.right_trigger-opmode.gamepad2.left_trigger;
        power = Range.clip(power,-1,1);
        config.havester_lift.setPower(power);


    }
}
