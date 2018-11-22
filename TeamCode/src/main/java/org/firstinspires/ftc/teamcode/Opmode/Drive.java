package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class Drive {
    // members:
    private OpMode opmode = null;
    private RoverRucusConfiguration config = null;


    public Drive(OpMode opmodeIn, RoverRucusConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery




    }
    public void updateDrive(){
        // Setup a variable for each drive wheel to save power level for telemetry
        double front_left_power;
        double front_right_power;
        double rear_left_power;
        double rear_right_power;

        // Implement Mecanum drive using drive equations from Internet:
        double left_y = opmode.gamepad1.left_stick_y;
        double left_x  = -opmode.gamepad1.left_stick_x;
        double right_x = -opmode.gamepad1.right_stick_x;
        front_left_power = Range.clip(left_y + left_x + right_x, -1.0, 1.0);
        rear_left_power = Range.clip(left_y - left_x + right_x, -1.0, 1.0);
        front_right_power = Range.clip(left_y - left_x - right_x, -1.0, 1.0);
        rear_right_power = Range.clip(left_y + left_x - right_x, -1.0, 1.0);

        // Send calculated power to wheels
        config.front_left_motor.setPower(front_left_power);
        config.front_right_motor.setPower(front_right_power);
        config.rear_left_motor.setPower(rear_left_power);
        config.rear_right_motor.setPower(rear_right_power);

        // Show the elapsed game time and wheel power.
        opmode.telemetry.addData("Motors", "front left (%.2f), front right (%.2f)",
                front_left_power, front_right_power);
        opmode.telemetry.addData("Motors", "rear left (%.2f), rear right (%.2f)",
                rear_left_power, rear_right_power);
        opmode.telemetry.update();

    }
}
