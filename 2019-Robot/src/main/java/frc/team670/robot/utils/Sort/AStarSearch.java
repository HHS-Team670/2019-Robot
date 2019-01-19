package frc.team670.robot.utils.sort;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Implementation oxf an A* search algorithm
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * 
 * @author ctchen, laksh, rghosh670
 */
public class AStarSearch {
    /**
     * Runs the search function
     * 
     * @param start       The node that the user wishes to start from
     * @param destination The target node that the use wishes to reach
     * @return A path of nodes that the search algorithm has found
     */
    public static List<Edge> search(Node start, Node destination) {

        HashMap<Node, Edge> cameFrom = new HashMap<Node, Edge>();
        Map<Node, Integer> fValues = new HashMap<Node, Integer>();
        Map<Node, Integer> gValues = new HashMap<Node, Integer>();

        Comparator<Node> fValueComp = new Comparator<Node>() {
            @Override


            public int compare(Node o1, Node o2) {
                int o1f = Integer.MAX_VALUE;
                int o2f = Integer.MAX_VALUE;

                if (fValues.get(o1) != null)
                    o1f = fValues.get(o1);

                if (fValues.get(o2) != null)
                    o2f = fValues.get(o2);

                return fValues.get(o1).compareTo(fValues.get(o2));
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
                end = true;
            }

            openList.remove(current);
            closedList.add(current);

            for (Edge e : current.getEdges()) {
                Node child = e.getDest();

                if (closedList.contains(child)) {
                    continue;
                }

                int tempG = gValues.get(current) + e.getCost();
                int tempF = tempG + child.getHeuristicDistance(destination);

                if (!openList.contains(child) || fValues.get(child) == null) {
                    fValues.put(child, tempF);
                    openList.add(child);
                } else if (gValues.get(child) == null || tempG >= gValues.get(child)) {
                    continue;
                }
                cameFrom.put(child, e);
                gValues.put(child, tempG);
            }

        }

        return getPath(start, destination, cameFrom);
    }

    private static List<Edge> getPath(Node start, Node destination, HashMap<Node, Edge> cameFrom) {
        List<Edge> path = new ArrayList<Edge>();
        Node node = destination;

        while (!node.equals(start)) {
            Edge e = cameFrom.get(node);
            path.add(e);
            node = e.getSource();
        }

        Collections.reverse(path);
        return path;
    }
}
