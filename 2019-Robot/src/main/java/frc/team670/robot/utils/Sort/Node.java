package frc.team670.robot.utils.Sort;

import java.util.List;

public interface Node{  
    double xN = 0;
    double yN = 0;

    /**
     * Models a node on a graph. Needed for A* sort 
     */
    public String getID();
    /**
     * @return a list of the edges that hit this node
     */
    public List<Edge> getEdges();

    public double getXN();

    public double getYN();
}