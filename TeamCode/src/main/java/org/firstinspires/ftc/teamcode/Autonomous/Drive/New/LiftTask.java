package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

/**
 * Created by Simon on 19/12/2018.
 */

public class LiftTask extends MainTask{

    public LiftTask(boolean state,float Power){
        LiftState = state;
        power = Power;
        context = "Lift";
    }
}
