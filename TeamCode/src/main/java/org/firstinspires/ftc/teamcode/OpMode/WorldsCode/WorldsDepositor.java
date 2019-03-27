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
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    public boolean toggleFunction = false;

    double servo_positions[][] = {
            {0.0,0.0},
            {0.0,0.0},
            {0.2,0.15},
            {0.4,0.3},
            {0.6,0.2}
    };
    int servo_index = 0;

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

       // config.dep_arm_servo.setPosition(servo_positions[servo_index][0]);
        //config.dep_bkt_servo.setPosition(servo_positions[servo_index][1]);




        // Setup a variable for each drive wheel to save power level for telemetry
        double front_left_power;
        double front_right_power;
        double rear_left_power;
        double rear_right_power;

        // Implement Mecanum drive using drive equations from Internet:
        double left_y = opmode.gamepad2.left_stick_y; //forward
        double left_x  = -opmode.gamepad1.left_stick_x; //strafe
        double right_x = -opmode.gamepad1.right_stick_x; //turning
        front_left_power = Range.clip(left_y + left_x + right_x, -1.0, 1.0);
        rear_left_power = Range.clip(left_y - left_x + right_x, -1.0, 1.0);
        front_right_power = Range.clip(left_y - left_x - right_x, -1.0, 1.0);
        rear_right_power = Range.clip(left_y + left_x - right_x, -1.0, 1.0);

        // Send calculated power to wheels
        config.front_left_motor.setPower(-front_left_power*Toggle());
        config.front_right_motor.setPower(-front_right_power*Toggle());
        config.rear_left_motor.setPower(-rear_left_power*Toggle());
        config.rear_right_motor.setPower(-rear_right_power*Toggle());

        // Show the elapsed game time and wheel power.
        opmode.telemetry.addData("Motors", "front left (%.2f), front right (%.2f)",
                front_left_power, front_right_power);
        opmode.telemetry.addData("Motors", "rear left (%.2f), rear right (%.2f)",
                rear_left_power, rear_right_power);
        opmode.telemetry.update();

    }
}
