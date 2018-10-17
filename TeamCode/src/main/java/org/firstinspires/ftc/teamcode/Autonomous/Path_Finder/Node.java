package org.firstinspires.ftc.teamcode.Autonomous.Path_Finder;

import org.firstinspires.ftc.teamcode.Autonomous.Drive.Coordinates;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Simon on 16/10/2018.
 */

public class Node {

    public Coordinates Coord;
    public boolean Walkable;
    public float Score;

    private List<Node> Children = new ArrayList<>();
    private Node Parent = null;

    public Node(Coordinates coords, Boolean _walkable, float _score){
        Coord = coords;
        Walkable = _walkable;
        Score = _score;
    }

    public void AddChild(Node Child){
        Children.add(Child);
    }
    public void RemoveChild(Node Child){
        for(int i=0;i<Children.size();i++){
            if(Children.get(i)==Child){
                Children.remove(i);
            }
        }
    }

    public void SetParent(Node _parent){
        Parent = _parent;
    }

}
