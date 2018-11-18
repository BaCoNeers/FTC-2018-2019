package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ToggleTemplate {

    //Code
    private OpMode opmode = null;
    public boolean buttonState = false;
    public boolean lastButtonState = false;
    public boolean state = false;
    private double motor_power;
    private DcMotor _motor = null;

    public ToggleTemplate(OpMode opmodeIn) {
        super();
        opmode = opmodeIn;
        //_motor  = opmode.hardwareMap.get(DcMotor.class, "_motor");

        //_motor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void Toggle() {
        if (buttonState && !lastButtonState){
            state = !state;

        }

        if (buttonState != lastButtonState){
            lastButtonState = buttonState;

        }

        if (state){
            //On state

            motor_power = 1.0;
        }

        else{
            //Off state

            motor_power = 0.0;

        }
    }

    public void updatemotor(){

        //_motor.setPower(motor_power);

    }
}

