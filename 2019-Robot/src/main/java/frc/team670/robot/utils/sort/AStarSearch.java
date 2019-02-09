package frc.team670.robot.utils.sort;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;

import frc.team670.robot.utils.Logger;

/**
 * Implementation of an A* search algorithm
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * 
 * @author ctchen, laksh, rghosh670
 */
public class AStarSearch {
    /**
     * Runs the search function, returning a List ordered in the Edges to take from one start Node to destination
     * 
     * @param start       The node that the user wishes to start from
     * @param destination The target node that the user wishes to reach
     * @return A path of nodes that the search algorithm has found. 
     * @exception IllegalArgumentException throws if the Node you are starting from has no open paths from it
     * Returns empty if destination and start are same node
     */
    public static List<Edge> search(Node start, Node destination) {
        
        // System.out.println("StartNode: " + start.getClass().getName() + ", DestNode: " + destination.getClass().getName());
        
        if (start.equals(destination)) {
            return new ArrayList<Edge>();
        }
        HashMap<Node, Edge> cameFrom = new HashMap<Node, Edge>();
        Map<Node, Integer> fValues = new HashMap<Node, Integer>();
        Map<Node, Integer> gValues = new HashMap<Node, Integer>();

        Comparator<Node> fValueComp = new Comparator<Node>() {
            @Override
            public int compare(Node o1, Node o2) { // This compare is dumb and if 2 have the same F-value, one won't get added
                int o1f = Integer.MAX_VALUE;
                int o2f = Integer.MAX_VALUE;

                if (fValues.get(o1) != null)
                    o1f = fValues.get(o1);

                if (fValues.get(o2) != null)
                    o2f = fValues.get(o2);

                return ((Integer) (o1f)).compareTo(o2f);
            }
        };

        SortedSet<Node> openList = new TreeSet<Node>(fValueComp);
        List<Node> closedList = new ArrayList<Node>();

        Node current = null;
        openList.add(start);

        boolean end = false;

        gValues.put(start, 0);

        fValues.put(start, start.getHeuristicDistance(destination));

        while (!openList.isEmpty() && !end) {

            current = openList.first();

            if (current.equals(destination)) {
                // System.out.println("CURRENT EQUALS DESTINATION");
                end = true;
            }

            if(!end) {
                openList.remove(current);
                closedList.add(current);

                for (Edge e : current.getEdges()) {
                    Node child = e.getDest();
                    // System.out.println("Edge: " + e.getClass().getName() +", Edge SourceNode: " + e.getSource().getClass().getName() + ", DestinationNode: " + child.getClass().getName());

                    if (child.equals(destination)) {
                        // System.out.println("CURRENT EQUALS DESTINATION IN EDGE LOOP");
                        // end = true;
                    }
                    // if(child.equals(destination)) {
                    //     cameFrom.put(child, e);
                    //     end = true;
                    //     break;
                    // }

                    if (closedList.contains(child)) {
                        continue;
                    }

                    int tempG = gValues.get(current) + e.getCost();
                    int tempF = tempG + child.getHeuristicDistance(destination);

                    if (!openList.contains(child) || fValues.get(child) == null) {
                        fValues.put(child, tempF);
                        openList.add(child); // WILL NOT ADD IF THE COORDINATES OF THE 2 ARM PIECES ARE THE SAME
                    } else if (gValues.get(child) == null || tempG >= gValues.get(child)) {
                        continue;
                    }
                    cameFrom.put(child, e);
                    gValues.put(child, tempG);
                }
                // System.out.println("OpenListLength: " + openList.size());
            }
        }

        if (openList.isEmpty()) { //either start or end is island
            throw new IllegalArgumentException("Invalid input, check for island");
        }
        return getPath(start, destination, cameFrom);
    }
    
    /**
     * @return the found path (list of edges)
     */
    private static List<Edge> getPath(Node start, Node destination, HashMap<Node, Edge> cameFrom) {
        List<Edge> path = new ArrayList<Edge>();
        Node node = destination;

        if(cameFrom.isEmpty()) { 
            throw new IllegalArgumentException("Invalid argument, check for island");
        }
        while (!node.equals(start)) {
            Edge e = cameFrom.get(node);
            path.add(e);
            node = e.getSource();
        }

        Collections.reverse(path);
        return path;
    }
}
