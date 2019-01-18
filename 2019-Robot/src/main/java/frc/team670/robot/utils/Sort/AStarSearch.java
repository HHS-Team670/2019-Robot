package frc.team670.robot.utils.sort;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

/**
 * Implementation of an A* search algorithm
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * @author ctchen, laksh, rghosh670
 */
public class AStarSearch {
    private Queue<Node> openList;
    private List<Node> closedList;
    private Node start;
    private Node destination;

    /**
     * @param start Desired node to start from
     * @param destination Desired node to reach 
     */
    public AStarSearch(Node start, Node destination){
        this.start = start;
        this.destination = destination;

        openList = new LinkedList<Node>();
        closedList = new ArrayList<Node>();
    }


    //TODO change it to return Edges
    /**
     * Runs the search function
     * @return A path of nodes that the search algorithm has found
     */
    public List<Node> search(){
        Node current = null;
        openList.add(start);

        while(!openList.isEmpty()){
            current = openList.poll();
            closedList.add(current);

            for(Edge e : current.getEdges()){
                Node child = e.getDest();
                double tempG = current.getG() + getCost(current, child);
                double tempF = tempG + calcH(child);

                 if(openList.contains(child) && (tempF >= child.getF())){
                        continue;
                  } else if(!openList.contains(child) || (tempF < child.getF())){
                    child.setParent(current);
                    child.setG(tempG);
                    child.setF(tempF);

                    if(openList.contains(child)){
                        openList.remove(child);
                    }

                    openList.add(child);
                  }
    
            }

            if(current.getState().equals(destination.getState())){
                break;
            }
        }
        return getPath();
    }


    private List<Node> getPath(){
        List<Node> path = new ArrayList<Node>();

            for(Node node = destination; node != null; node = node.getParent()){
                path.add(node);
            }

        Collections.reverse(path);
        return path;
    }

    private double getCost(Node d1, Node d2){
        double d1X = d1.getCoord().getX();
        double d1Y = d1.getCoord().getX();
        double d2X = d2.getCoord().getX();
        double d2Y = d2.getCoord().getX();

        return Math.sqrt((d2Y - d1Y) * (d2Y - d1Y) + (d2X - d1X) * (d2X - d1X));
    }

    private double calcH(Node node){
        return getCost(node, destination);
    }
    
}

