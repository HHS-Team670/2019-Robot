package frc.team670.robot.utils.Sort;


import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.awt.geom.Point2D;



/**
 * Implementation of an A* search algorithm
 * @author ctchen, laksh, rohit
 */
public class AStarSearch implements Edge{

    private List<Point2D.Double> openNodes = new ArrayList<>();
    private List<Point2D.Double> closedNodes = new ArrayList<>();
    private List<Edge> values = new ArrayList<>();
    private int numRecursion = -1;


    /**
     * Implements the AStar search algorithm given start and end/goal
     * @param start Node to start searching at
     * @param destination The goal
     * @return The quickest path (list of edges) to get from start to destination
     */
    public List<Edge> search(Point2D.Double start, Point2D.Double destination){

        int i = 0;
        int ID = 0;

        numRecursion++;

        Point2D.Double smallest = new Point2D.Double(0, 0);
        Point2D.Double current = openNodes.get(i);

        if(start != destination){

            if(openNodes.size() != 0){


                while(i < openNodes.size()){
                    current = openNodes.get(i);
                    if(getCost(start, current) < getCost(start, smallest)){
                        smallest = current;
                        ID = i;                                                                                                
                    }                                                                                                
                }
               Point2D.Double popped = openNodes.remove(ID);


            if(numRecursion == 8){
                //TODO
                return null;
            }

            search(popped, destination);

            } else{
            
            } 
        }  else{

        }
        
        

       return values;
    }


    public String getID(){
        return "TODO";
    }

    public List<Edge> getEdges(){
        return new ArrayList<>();
    }

    public double getCost(Point2D.Double d1, Point2D.Double d2){
        return Math.sqrt((d2.getY() - d1.getY()) * (d2.getY() - d1.getY())
        + (d2.getX() - d1.getX()) * (d2.getX() - d1.getX()));
    }

    

}