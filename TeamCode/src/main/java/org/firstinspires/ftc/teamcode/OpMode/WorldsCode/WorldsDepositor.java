package org.firstinspires.ftc.teamcode.OpMode.WorldsCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;
import org.firstinspires.ftc.teamcode.Configuration.WorldsConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class WorldsDepositor {
    // members:
    private OpMode opmode = null;
    private WorldsConfiguration config = null;

    double servo_positions[][] = {
            {0.0,0.0},
            {0.0,0.0},
            {0.2,0.15},
            {0.4,0.3},
            {0.6,0.2}
    };
    int servo_index = 0;


    public WorldsDepositor(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }

    public void update(){

        // Manipulate the position of the servos
        if (opmode.gamepad2.b) {
            servo_index += 1;
        } else  if (opmode.gamepad2.a) {
            servo_index -= 1;
        }
        if (servo_index < 0) servo_index = 0;
        if (servo_index >= servo_positions.length) servo_index = servo_positions.length - 1;

        config.depositor_arm.setPosition(servo_positions[servo_index][0]);
        config.depositor.setPosition(servo_positions[servo_index][1]);
        


    }
}
