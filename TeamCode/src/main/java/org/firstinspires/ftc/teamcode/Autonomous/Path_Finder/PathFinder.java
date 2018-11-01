package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.CoordinateDrive;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Simon on 5/10/2018.
 */

public class PathFinder{

    //Task management
    public ArrayList<Coordinates> Task = new ArrayList<Coordinates>();
    private boolean completed = false;

    private CoordinateDrive Drive;




    public void Update(){


    }

    public void AddTask(Coordinates coords){
        Task.add(coords);
    }

}
