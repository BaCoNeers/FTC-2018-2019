package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.CoordinateDrive;
import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Simon on 5/10/2018.
 */

public class PathFinder extends CoordinateDrive{

    //Node Setup
    public int Size = 21;
    public float FieldSize = 3910;
    private float NodeSpacing = FieldSize/Size;
    public Node[][] Nodes = new Node[Size][Size];

    //Task management
    public List<Coordinates> Task = new ArrayList<Coordinates>();
    private boolean completed = false;


    //Path finding
    private Node Current;
    private Coordinates Goal;
    private List<Node[]> Path;

    public void SetUpNodes(){
        for(int x=0;x<Size;x++){
            for(int y=0;y<Size;y++){
                Nodes[x][y] = new Node(
                        new Coordinates(x*NodeSpacing,y*NodeSpacing,0),
                        true,0);
            }
        }
        SetCurrentPosition();
    }

    private void SetCurrentPosition(){
        Current = Nodes[Math.round(Coords.x/NodeSpacing)][Math.round(Coords.y/NodeSpacing)];
    }

    private void CalculateChildScore(Node N){
        for(int i=0;i<N.Children.size();i++){
            N.Children.get(i).Score = GetDistance(Current.Coord,N.Children.get(i).Coord)+
                    GetDistance(N.Children.get(i).Coord,Goal);
        }
    }

    private void AssignChildren(){
        Current.AddChild(Nodes[Math.round(Current.Coord.x+1/NodeSpacing)]
                [Math.round(Current.Coord.y/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x+1/NodeSpacing)]
                [Math.round(Current.Coord.y+1/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x/NodeSpacing)]
                [Math.round(Current.Coord.y+1/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x-1/NodeSpacing)]
                [Math.round(Current.Coord.y+1/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x-1/NodeSpacing)]
                [Math.round(Current.Coord.y/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x-1/NodeSpacing)]
                [Math.round(Current.Coord.y-1/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x/NodeSpacing)]
                [Math.round(Current.Coord.y-1/NodeSpacing)]);
        Current.AddChild(Nodes[Math.round(Current.Coord.x+1/NodeSpacing)]
                [Math.round(Current.Coord.y-1/NodeSpacing)]);
    }

    public void Update(){
        if(Task.size() == 0){
            return;
        }
        if(completed = false){
            //Need work//
            completed = SetCoordinate(Task.get(0).x,Task.get(0).y,0.5f);
        }
        else{
            completed = false;
            Task.remove(0);
        }
    }

    public void AddTask(Coordinates coords){
        Task.add(Coords);
    }

    private void Getpath(){
        AssignChildren();
        CalculateChildScore(Current);
        float score = 0;
        for(int i=0;i<Current.Children.size();i++){
            if(Current.Children.get(i).Score<score){
                score = Current.Children.get(i).Score;
            }
        }
    }

    private float GetDistance(Coordinates a, Coordinates b){
        return Math.abs((b.x-a.x)+(b.y-a.y));
    }
}
