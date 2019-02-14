package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;


import org.firstinspires.ftc.teamcode.Autonomous.Drive.Old.AutoDrive;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.Old.Coordinates;

import java.util.ArrayList;

/**
 * Created by Simon on 5/10/2018.
 */

public class PathFinder{

    //Task management
    public ArrayList<Coordinates> Task = new ArrayList<Coordinates>();
    private boolean completed = false;

    private AutoDrive Drive;




    public void Update(){


    }

    public void AddTask(Coordinates coords){
        Task.add(coords);
    }

}
