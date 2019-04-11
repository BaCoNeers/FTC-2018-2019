package org.firstinspires.ftc.teamcode.OpMode.WorldsCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.WorldsConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class WorldsDrive {
    // members:
    private OpMode opmode = null;
    private WorldsConfiguration config = null;
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    public boolean toggleFunction = false;

   public double toggle(){
        buttonState = opmode.gamepad1.dpad_down;
        if (buttonState && !lastButtonState) {
            state = !state;
        }

        if (buttonState != lastButtonState) {
            lastButtonState = buttonState;
        }


        if (state) {
            //On state
            return 0.5f;

        }
        else {
            //Off state

            return 1.0f;

        }


   }
       public WorldsDrive(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }

    public void update(){
        //Drive not working, strafing works fine, turning is faulty and forward isn't working
        //When going forward the front and back wheels are going opposite directions
        //when turning it seems that some wheel might be going slower then others causing it to NOT turn on the centre


        // Setup a variable for each drive wheel to save power level for telemetry
        double front_left_power;
        double front_right_power;
        double rear_left_power;
        double rear_right_power;

        // Implement Mecanum drive using drive equations from Internet:
        double left_y = opmode.gamepad1.left_stick_y; //forward
        double strafe_left_x  = opmode.gamepad1.left_stick_x; //strafe
        double right_x = -opmode.gamepad1.right_stick_x; //turning
        front_left_power = Range.clip(left_y + strafe_left_x + right_x, -1.0, 1.0);
        rear_left_power = Range.clip(left_y - strafe_left_x + right_x, -1.0, 1.0);
        front_right_power = Range.clip(left_y - strafe_left_x - right_x, -1.0, 1.0);
        rear_right_power = Range.clip(left_y + strafe_left_x - right_x, -1.0, 1.0);

        // Send calculated power to wheels
        config.front_left_motor.setPower(-front_left_power*toggle());
        config.front_right_motor.setPower(-front_right_power*toggle());
        config.rear_left_motor.setPower(-rear_left_power*toggle());
        config.rear_right_motor.setPower(-rear_right_power*toggle());

        // Show the elapsed game time and wheel power.
        opmode.telemetry.addData("Motors", "front left (%.2f), front right (%.2f)",
                front_left_power, front_right_power);
        opmode.telemetry.addData("Motors", "rear left (%.2f), rear right (%.2f)",
                rear_left_power, rear_right_power);
        opmode.telemetry.update();

    }
}
