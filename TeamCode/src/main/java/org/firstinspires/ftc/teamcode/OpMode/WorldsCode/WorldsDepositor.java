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


    /*
    //First set of variables for depositor
    boolean buttonState = false;
    boolean lastButtonState = false;
    boolean state = false;
    double depositor_servo_value = 0.5;

    //Second set of variables for depositor
    boolean buttonState2 = false;
    boolean lastButtonState2 = false;
    boolean state2 = false;
    */


    //Variables for depositor arm
    boolean buttonState3 = false;
    boolean lastButtonState3 = false;
    boolean state3 = false;
    double arm_servo_value = 0;



    //Variables for depositor
    boolean depositor_left;
    boolean depositor_right;
    boolean depositor_default;



        public void update() {


            //Code for the linear motion lift (mineral depositor)
            double liftPower;

            liftPower = opmode.gamepad1.right_stick_y;

            config.depositor_lift.setPower(liftPower);


            /*

            //Servo depositor toggle 1
            buttonState = opmode.gamepad1.a;
            if (buttonState && !lastButtonState) {
                state = !state;
            }

            if (buttonState != lastButtonState) {
                lastButtonState = buttonState;
            }


            if (state) {
                //On state
                depositor_servo_value = 0.0;

            }
            else {
                //Off state
                depositor_servo_value = 0.5;
            }



            //Servo depositor toggle 2
            buttonState2 = opmode.gamepad1.b;
            if (buttonState2 && !lastButtonState2) {
                state2 = !state2;
            }

            if (buttonState2 != lastButtonState2) {
                lastButtonState2 = buttonState2;
            }


            if (state2) {
                //On state
                depositor_servo_value = 1.0;

            }
            else {
                //Off state
                depositor_servo_value = 0.5;

            }

            */

            Range.clip(arm_servo_value, 0, 1);


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




            //Servo Increments
            if (opmode.gamepad1.right_trigger >= 0) {
                arm_servo_value += 0.2;

            }
            else if (opmode.gamepad1.right_bumper) {
                arm_servo_value += 0.1;

            }
            else if (opmode.gamepad1.left_bumper) {
                arm_servo_value -= 0.1;

            }

            //Mineral Depositor

            depositor_left = opmode.gamepad1.dpad_left;
            depositor_default = opmode.gamepad1.dpad_up;
            depositor_right = opmode.gamepad1.dpad_right;


            /*
            //Depositor code (set to position)
            if (depositor_default) {
                config.mineral_depositor.setPosition(0.5);

            }
            else if (depositor_left) {
                config.mineral_depositor.setPosition(0.0);

            }
            else if (depositor_right) {
                config.mineral_depositor.setPosition(1.0);

            }
            else {


            }
            */


            //Depositor code (continuous)
            if (depositor_left) {
                config.mineral_depositor.setPower(-1);

            }

            else if (depositor_right) {
                config.mineral_depositor.setPower(1);

            }

            else {
                config.mineral_depositor.setPower(0);

            }



            config.depositor_arm.setPosition(arm_servo_value);
            //config.mineral_depositor.setPosition(depositor_servo_value);

        }
    }



