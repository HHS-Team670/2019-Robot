package frc.team670.robot.utils.Sort;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.awt.geom.Point2D;

/**
 * Implementation of an A* search algorithm
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * @author ctchen, laksh, rohit
 */
public class AStarSearch implements Edge{

    private List<Node> openNodes = new ArrayList<>();
    private List<Node> closedNodes = new ArrayList<>();
    private List<Edge> values = new ArrayList<>();
    private int numRecursion = -1;
    private Node start, destination;
    private double indexX, indexY;

    public AStarSearch(Node star, Node destination){
        this.start = start;
        this.destination = destination;
    }


    /**
     * Implements the AStar search algorithm given start and end/goal
     * @param startNode Node to start searching at
     * @param destination The goal
     * @return The quickest path (list of edges) to get from start to destination
     */
    public List<Edge> search(Node startNode, Node destination){

        int i = 0;
        int ID = 0;

        numRecursion++;

        Node smallest = new Node(0, 0);
        Node current = openNodes.get(i);

        if(startNode != destination){

            if(openNodes.size() != 0){
                while(i < openNodes.size()){
                    current = openNodes.get(i);
                    if(calculateF(startNode, current) < calculateF(startNode, smallest)){
                        smallest = current;
                        ID = i;                                                                                                
                    }                                                                                                
                }
            
             indexX = openNodes.get(ID).getX();
             indexY = openNodes.get(ID).getY();
             Node popped = openNodes.remove(ID);




            search(popped, destination);
            } 

        }  else{
            //Reached Destination
            return null;

        }
        
        

       return values;
    }


    public String getID(){
        return "TODO";
    }

    public List<Edge> getEdges(){
        return new ArrayList<>();
    }

    public double getCost(Node d1, Node d2){
        return Math.sqrt((d2.getY() - d1.getY()) * (d2.getY() - d1.getY())
        + (d2.getX() - d1.getX()) * (d2.getX() - d1.getX()));
    }

    /** 
     * 
     * @return the f-value for a node, d1
     */ 
    public double calculateF(Node d1, Node d2){
        return getCost(d1, d2) + getCost(d1, destination);
    }


}