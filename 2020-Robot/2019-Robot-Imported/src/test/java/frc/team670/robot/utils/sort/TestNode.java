package frc.team670.robot.utils.sort;

import frc.team670.robot.utils.sort.Node;
import frc.team670.robot.utils.sort.Edge;

public class TestNode implements Node{

    private int x, y;
    private Edge[] edges;

    public TestNode(int x, int y, Edge[] edges){
        this.x = x;
        this.y = y;
        this.edges = edges;
    }

    public Edge[] getEdges(){
        return edges;
    }

    public int getX(){
        return x;
    }

    public int getY(){
        return y;
    }

    public int getHeuristicDistance(Node target){
        TestNode t = (TestNode)target;
        return (int)(Math.sqrt((x-t.getX())*(x-t.getX())+(y-t.getY())*(y-t.getY())));
    }

    public String toString(){
        return "("+this.x+","+this.y+")";
    }

    public void setEdges(Edge[] edges){
        this.edges = edges;
    }

    @Override
    public int compareTo(Node o) {
        return (this == o) ? 0 : 1;
    }
}