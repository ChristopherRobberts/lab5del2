import edu.princeton.cs.algs4.IndexMinPQ;
import se.kth.id1020.DataSource;
import se.kth.id1020.Edge;
import se.kth.id1020.Graph;
import se.kth.id1020.Vertex;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Stack;

public class Dijkstra {
    private double[] distTo;
    private Edge[] edgeTo;
    private IndexMinPQ<Double> priority;
    private HashMap<String, Integer> ver;

    public Dijkstra(Graph g, String source){
        distTo = new double[g.numberOfVertices()];
        edgeTo = new Edge[g.numberOfVertices()];
        vertexLabel(g);
        int s = ver.get(source);
        for(int v = 0; v < g.numberOfVertices(); v++){
            distTo[v] = Double.POSITIVE_INFINITY;
        }
        distTo[s] = 0.0;

        priority = new IndexMinPQ<Double>(g.numberOfVertices());
        priority.insert(s, distTo[s]);
        while(!priority.isEmpty()){
            relaxWithoutWeight(g, priority.delMin());
        }
    }

    private void vertexLabel(Graph g){

        ver = new HashMap<String, Integer>(g.numberOfVertices());
        for(Iterator e = g.vertices().iterator(); e.hasNext();){
            Vertex vertex = (Vertex) e.next();
            ver.put(vertex.label, vertex.id);
        }
    }

    private void relaxWithoutWeight(Graph g, int v){
        double weight = 1.0;
        for(Iterator i = g.adj(v).iterator(); i.hasNext();){
            Edge e = (Edge) i.next();
            int w = e.to;
            if(distTo[w] > distTo[v] + weight){
                distTo[w] = distTo[v] + weight;
                edgeTo[w] = e;

                if(priority.contains(w))
                    priority.decreaseKey(w, distTo[w]);
                else
                    priority.insert(w, distTo[w]);
            }
        }
    }

    public boolean hasPathTo(int destination){
        return distTo[destination] < Double.POSITIVE_INFINITY;
    }

    public Iterable<Edge> pathTo(String destination){
        if(!hasPathTo(ver.get(destination))){
            return null;
        }
        Stack<Edge> path = new Stack<Edge>();
        for(Edge e = edgeTo[ver.get(destination)]; e != null; e = edgeTo[e.from]){
            path.push(e);
        }
        return path;
    }

    public static void main(String[] args){
        Graph g = DataSource.load();
        Dijkstra dijkstra = new Dijkstra(g, "Renyn");
        DijkstraWeight dijkstraWeight = new DijkstraWeight(g, "Renyn");
        int weight = 1;
        int withoutWeight = 1;
        for(Iterator i = dijkstra.pathTo("Parses").iterator(); i.hasNext();){
            i.next();
            withoutWeight++;
        }
        System.out.println("Shortest path without taking weights in to account: " + withoutWeight);

        for(Iterator i = dijkstraWeight.pathTo("Parses").iterator(); i.hasNext();){
            Edge edge = (Edge) i.next();
            System.out.println(g.vertex(edge.from));
            weight++;
        }
        System.out.println("Shortest path taking weights in to account: " + weight);
    }
}
