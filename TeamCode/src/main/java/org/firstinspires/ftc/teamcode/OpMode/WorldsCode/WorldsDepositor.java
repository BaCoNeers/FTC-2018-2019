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


    public WorldsDepositor(OpMode opmodeIn, WorldsConfiguration configIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;
        config = configIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
    }


    //Variables for depositor arm
    boolean buttonState3 = false;
    boolean lastButtonState3 = false;
    boolean state3 = false;
    double arm_servo_value = 0;
    double depositor_servo_value = 0;




        public void update() {


            //// DEPOSITOR LIFT CODE ///


            //Code for the linear motion lift (mineral depositor lift)
            double liftPower;

            liftPower = opmode.gamepad1.right_stick_y;

            config.depositor_lift.setPower(liftPower);



            //// MINERAL DEPOSITOR CODE ///



            //Depositor code (set to position)


            if (opmode.gamepad1.dpad_right) {
                depositor_servo_value = 0.63;

            }
            else if (opmode.gamepad1.dpad_down) {
                depositor_servo_value = 0.0;

            }
            else if (opmode.gamepad1.dpad_up) {
                depositor_servo_value = 1.0;

            }
            else if (opmode.gamepad1.dpad_left) {
                depositor_servo_value = 0.2;

            }

            //// SERVO ARM CODE ////



            //Servo Increments

            /*
            if (opmode.gamepad1.b) {
                arm_servo_value = arm_servo_value + 0.1;

            }
            else if (opmode.gamepad1.a) {
                arm_servo_value = arm_servo_value - 0.1;

            }
            */


            //Servo Arm Toggle
            buttonState3 = opmode.gamepad1.left_bumper;
            if (buttonState3 && !lastButtonState3) {
                state3 = !state3;
            }

            if (buttonState3 != lastButtonState3) {
                lastButtonState3 = buttonState3;
            }


            if (state3) {
                //On state
                arm_servo_value = 1;

            }
            else {
                //Off state
                arm_servo_value = 0;

            }



            Range.clip(arm_servo_value, 0, 1);
            config.depositor_arm.setPosition(arm_servo_value);
            config.mineral_depositor.setPosition(depositor_servo_value);
            //config.mineral_depositor.setPosition(depositor_servo_value);

        }
    }



