package frc.team670.robot.utils.Sort;

import java.util.List;
import java.awt.geom.Point2D;

/**
* Models a node on a graph. Needed for A* sort 
* https://www.geeksforgeeks.org/a-search-algorithm/ 
*/
public class Node{  
    private Point2D.Double node;
    private boolean q; 
    private int ID;   

    public Node(double x, double y){
        this.node = new Point2D.Double(x, y);
    }

    public Node(Point2D.Double node, int ID){
        this.node = node;
        q = false;
        this.ID = ID;
    }

    public int getID(){
        return ID;
    }
    
    /**
     * @return a list of the edges that hit this node
     */
    public List<Edge> getEdges(){
        return null;
    }

    public double getX(){
        return node.getX();
    }

    public double getY(){
        return node.getY();
    }

    public boolean getQ(){
        return q;
    }

    public void setQ(boolean q){
        this.q = q;
    }
}