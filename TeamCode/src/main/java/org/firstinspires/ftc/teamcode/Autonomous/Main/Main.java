package org.firstinspires.ftc.teamcode.Autonomous.Main;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;
import org.firstinspires.ftc.teamcode.Autonomous.Path_Finder.PathFinder;

/**
 * Created by Simon on 27/09/2018.
 */

public class Main extends PathFinder {

    @Override
    public void init_loop() {
        AddTask(new Coordinates(10,10,0));
    }

    @Override
    public void loop() {
        Update();

    }
}
