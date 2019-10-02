package org.firstinspires.ftc.teamcode.OpMode.DaVinci;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

/**
 * Created by Baconeers on 8/11/2018.
 */

public class DaVinciDriveCode {
    // members:
    private OpMode opmode = null;
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;

    boolean buttonState = false;
    boolean lastButtonState = false;
    boolean state = false;


    public DaVinciDriveCode(OpMode opmodeIn) {
        super();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        opmode = opmodeIn;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightMotor = opmode.hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = opmode.hardwareMap.get(DcMotor.class, "leftMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void updateDaVinciDrive(){
        double leftStickY = opmode.gamepad1.left_stick_y;
        double rightStickY = opmode.gamepad1.right_stick_y;
        double power;

        buttonState = opmode.gamepad1.a;
        if (buttonState && !lastButtonState) {
            state = !state;
        }

        if (buttonState != lastButtonState) {
            lastButtonState = buttonState;
        }


        if (state) {
            //On state
            power = 0.5d;

        }
        else {
            //Off state

            power = 1.0d;
        }


        double rightPower = rightStickY*power;
        double leftPower = leftStickY*power;

        rightMotor.setPower(rightPower);
        leftMotor.setPower(leftPower);

    }
}
