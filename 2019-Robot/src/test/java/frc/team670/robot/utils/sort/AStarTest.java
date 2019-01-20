package frc.team670.robot.utils.sort;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Test;
import org.junit.Assert.*;

/**
 * Test implementation of A* search algorithm
 * @author ctchen, rghosh670
 */
public class AStarTest{

    // TestNode t0, t1, t2, t3, t4, t5;
    // TestEdge e0, e1, e2, e3, e4;
    // t0 = new TestNode(0, 0, new Edge[]{e0, e1});
    // t1 = new TestNode(1, 1, new Edge[]{e1, e3} );
    // t2 = new TestNode(1, 4, new Edge[]{e1, e2});
    // t3 = new TestNode(4, 3, new Edge[]{e3, e4});
    // t4 = new TestNode(6, 2, null );
    // t5 = new TestNode(6, 4, new Edge[]{e2, e4});
    // e0 = new TestEdge(t0, t1);
    // e1 = new TestEdge(t0, t2);
    // e2 = new TestEdge(t2, t5);
    // e3 = new TestEdge(t1, t3);
    // e4 = new TestEdge(t3, t5);

      @Test  
      public void AStarSearchTest(){
        TestEdge e0 = new TestEdge(), e1 = new TestEdge(), e2 = new TestEdge(), e3 = new TestEdge(), e4 = new TestEdge();
        TestNode t0 = new TestNode(0, 0, new Edge[]{e0, e1});
        TestNode t1 = new TestNode(1, 1, new Edge[]{e1, e3});
        TestNode t2 = new TestNode(1, 4, new Edge[]{e1, e2});
        TestNode t3=new TestNode(4, 3, new Edge[]{e3, e4});
        TestNode t4=new TestNode(6, 2, new Edge[]{} );
        TestNode t5= new TestNode(6, 4, new Edge[]{e2, e4});
        e0.setNodes(t0, t1);
        e1.setNodes(t0, t2);
        e2.setNodes(t2, t5);
        e3.setNodes(t1, t3);
        e4.setNodes(t3, t5);
        // t1.setEdges(new Edge[]{e0, e1});
        // t2.setEdges(new Edge[]{e1, e2});
        // t3.setEdges(new Edge[]{e3, e4});
        // t4.setEdges(new Edge[]{});
        // t5.setEdges(new Edge[]{e2, e4});
        //t0 = new TestNode(0, 0, new Edge[]{e0, e1});
  //t1 = new TestNode(1, 1, new Edge[]{e1, e3} );
       // t2 = new TestNode(1, 4, new Edge[]{e1, e2});
        //t3 = new TestNode(4, 3, new Edge[]{e3, e4});
        //t4 = new TestNode(6, 2, null );
        //t5 = new TestNode(6, 4, new Edge[]{e2, e4});
        List<Edge> result = AStarSearch.search(t0, t5);
        assertArrayEquals(new Edge[]{e0, e3, e4}, result.toArray());
       // List<Edge> result1 = AStarSearch.search(t4, t5); //island node test
       // assertArrayEquals(new Edge[]{}, result1.toArray());
        try {
            assertEquals("Invalid input, check for island", AStarSearch.search(t4, t5));
            fail( "Should have thrown an exception" );
        } 
        catch (Exception e) {
            String expectedMessage = "Invalid input, check for island";
            assertEquals("Exception message must be correct", expectedMessage, e.getMessage() );
        }   
        List<Edge> result2 = AStarSearch.search(t5, t5);
        assertArrayEquals(new Edge[]{}, result2.toArray());
        
        // List<Edge> result3 = AStarSearch.search(t0, t4);
        // assertArrayEquals(new Edge[]{}, result3.toArray());
        try {
            assertEquals("Invalid input, check for island", AStarSearch.search(t0, t4));
            fail( "Should have thrown an exception" );
        } 
        catch (Exception e) {
            String expectedMessage = "Invalid input, check for island";
            assertEquals( "Exception message must be correct", expectedMessage, e.getMessage() );
        }   
        List<Edge> result4 = AStarSearch.search(t0, t3);
        assertArrayEquals(new Edge[]{e0, e3}, result4.toArray());
        // TestEdge a0 = new TestEdge(), a1 = new TestEdge(), a2 = new TestEdge(), a3 = new TestEdge(), a4 = new TestEdge();
        // TestNode p0 = new TestNode(0, 0, new Edge[]{a0, a1, a2});
        // TestNode p1 = new TestNode(1, -1, new Edge[]{a2, a3});
        // TestNode p2 = new TestNode(2, 4, new Edge[]{a0, a2, a8});
        // TestNode p3=new TestNode(3, 1, new Edge[]{a1, a3, a4, a6, a7});
        // TestNode p4=new TestNode(5, 0, new Edge[]{a4, a5});
        // TestNode p5= new TestNode();
        // TestNode p6 = new TestNode();
        // TestNode p7 = new TestNode();
        // a0.setNodes(, );
        // a1.setNodes(, );
        // a2.setNodes(t2, t5);
        // a3.setNodes(t1, t3);
        // a4.setNodes(t3, t5);
        // a5.setNodes();
        // a6.setNodes();
        // a7.setNodes();
        // a8.setNodes();
        // a9.setNodes();
        // a10.setNodes();
        // a11.setNodes();
    }
}

