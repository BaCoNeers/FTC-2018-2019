package org.firstinspires.ftc.teamcode.Autonomous.NewAttempt;

/**
 * Created by Simon on 14/11/2018.
 */

public class Task {
    float Distance = 0;
    float Power;
    float Angle = 0;
    float Strafe = 0;
    boolean Lift = false;

    Task(float Value, float Power, String Context) {
        this.Power = Power;
        if (Context == "Forward") {
            this.Distance = Value;
        }
        if (Context == "Turning") {
            this.Angle = Value;
        }
        if (Context == "Strafing") {
            this.Strafe = Value;
        }
    }

    Task(float Power){
        this.Lift = true;
        this.Power = Power;
    }
}
