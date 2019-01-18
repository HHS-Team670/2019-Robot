package frc.team670.robot.utils.sort;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

/**
 * Implementation of an A* search algorithm
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * @author ctchen, laksh, rghosh670
 */
public class AStarSearch {
    /**
     * Runs the search function
     * @param start The node that the user wishes to start from
     * @param destination The target node that the use wishes to reach
     * @return A path of nodes that the search algorithm has found
     */
    public static List<Node> search(Node start, Node destination){

        Queue<Node> openList = new LinkedList<Node>();
        List<Node> closedList = new ArrayList<Node>();

        Node current = null;
        openList.add(start);

        boolean end = false;

        HashMap<Node, Edge> cameFrom = new HashMap<Node, Edge>();
        Map<Node, Integer> fValues = new HashMap<Node, Integer>();
        Map<Node, Integer> gValues = new HashMap<Node, Integer>();
        gValues.put(start, 0);

        while(!openList.isEmpty() && !end){
            current = openList.poll();
            closedList.add(current);
           
            if(current.equals(destination)){
                end = true;
            }

            for(Edge e : current.getEdges()){
                Node child = e.getDest();
                int tempG = gValues.get(current) + e.getCost();
                int tempF = tempG + child.getHeuristicDistance(destination);

                 if(closedList.contains(child) && (fValues.get(child) == null || tempF >= fValues.get(child))) {
                        continue;
                  } else if(!openList.contains(child) || (tempF < deal with fvalues child.getF())){
                    cameFrom.put(child, e);

                    gValues.set
                    fValues.set 
                    child.setG(tempG);
                    child.setF(tempF);

                    if(openList.contains(child)){
                        openList.remove(child);
                    }

                    openList.add(child);
                  }
    
            }

            
        }
        return getPath(destination);
    }


    private static List<Node> getPath(Node destination, Map<Node, Node> cameFrom){
        List<Node> path = new ArrayList<Node>();

            for(Node node = destination; node != null; node = node.getParent()){
                path.add(node);
            }

        Collections.reverse(path);
        return path;
    }

    private static double getDist(Node d1, Node d2){
        double d1X = d1.getCoord().getX();
        double d1Y = d1.getCoord().getX();
        double d2X = d2.getCoord().getX();
        double d2Y = d2.getCoord().getX();

        return Math.sqrt((d2Y - d1Y) * (d2Y - d1Y) + (d2X - d1X) * (d2X - d1X));
    }

    private static double calcH(Node node, Node destination){
        return getDist(node, destination);
    }
    
}

