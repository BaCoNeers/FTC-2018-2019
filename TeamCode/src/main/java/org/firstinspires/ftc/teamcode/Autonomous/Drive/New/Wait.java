package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

/**
 * Created by Simon on 14/03/2019.
 */

public class Wait extends MainTask {

    public Wait(float seconds){
        value = (float) (System.nanoTime() +  (seconds* Math.pow(10,6)));
        context = "Wait";
    }

}
