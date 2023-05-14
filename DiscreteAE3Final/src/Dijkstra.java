import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

public class Dijkstra {
    // trace paths from the given source using the previous property of the vertices
    public static void tracePathsFromSource(Vertex source) {
        // set distance from the source to itself as 0
        source.distance = 0.0;
        // use a priority queue to sort and store vertices based
        // on their distances from the source vertex
        PriorityQueue<Vertex> vertexQueue = new PriorityQueue<>();
        // add source vertex to priority queue
        vertexQueue.add(source);

        while (!vertexQueue.isEmpty() && vertexQueue.peek() != null) {
            // access vertex in priority queue closest to source
            Vertex current = vertexQueue.poll();
            // visit each edge connected to current
            for (Edge e : current.neighbors) {
                // next is a neighbor of current
                Vertex next = e.destination;
                // update hypothetical distance from source given neighbor
                double newDistance = current.distance + e.weight;
                // update if new distance is more efficient path
                if (newDistance < next.distance) {
                    vertexQueue.remove(next);
                    next.distance = newDistance;
                    next.previous = current;
                    vertexQueue.add(next);
                }
            }
        }
    }

    public static List<Vertex> getShortestPath(Vertex destination) {
        // use array list to store path from source to given destination
        List<Vertex> path = new ArrayList<Vertex>();
        // add each of the vertices in the path traced by tracePathsFromSource
        for (Vertex vertex = destination; vertex != null; vertex = vertex.previous)
            path.add(vertex);
        // reverse to correct ordering of path
        Collections.reverse(path);
        return path;
    }

    public static void printPath(List<Vertex> path, Vertex destination) {
        String newPath = "Path: ";
        double currentWeight = 0.0;
        // visit each vertex in path
        for (Vertex v : path) {
            // add vertex to path string
            newPath += v;
            // if the destination hasn't been reached, add the distance to the next node as well
            if (!v.name.equals(destination.name)) {
                for (Edge e : v.neighbors) {
                    if (e.destination == (path.get(path.indexOf(v) + 1))) currentWeight = e.weight;
                }
                newPath += " --" + (int) currentWeight + "--> ";
            }
        }
        // print path string
        System.out.println(newPath);
    }

    public static void main(String[] args) {
        //////////////////////////////
        //////  GRAPH 1   ////////////
        ///// 6 VERTICES  ////////////
        //////////////////////////////

        // create list to store vertices in graph 1
        ArrayList<Vertex> graph1Vertices = new ArrayList<>();

        // create all the vertices
        Vertex A = new Vertex("A");
        graph1Vertices.add(A);
        Vertex B = new Vertex("B");
        graph1Vertices.add(B);
        Vertex C = new Vertex("C");
        graph1Vertices.add(C);
        Vertex D = new Vertex("D");
        graph1Vertices.add(D);
        Vertex E = new Vertex("E");
        graph1Vertices.add(E);
        Vertex F = new Vertex("F");
        graph1Vertices.add(F);

        // add all of the edges and corresponding weights
        A.neighbors = new Edge[]{new Edge(B, 4),
                new Edge(C, 2)};
        B.neighbors = new Edge[]{new Edge(A, 4),
                new Edge(C, 1),
                new Edge(D, 5)};
        C.neighbors = new Edge[]{new Edge(A, 2),
                new Edge(B, 1),
                new Edge(D, 8),
                new Edge(E, 10)};
        D.neighbors = new Edge[]{new Edge(B, 5),
                new Edge(C, 8),
                new Edge(E, 2),
                new Edge(F, 6)};
        E.neighbors = new Edge[]{new Edge(C, 10),
                new Edge(D, 2),
                new Edge(F, 5)};
        F.neighbors = new Edge[]{new Edge(D, 6),
                new Edge(E, 5)};

        //////////////////////////////
        //////  GRAPH 2   ////////////
        /////  9 VERTICES ////////////
        //////////////////////////////

        // create another list to store vertices in graph 2
        ArrayList<Vertex> graph2Vertices = new ArrayList<>();

        // create all the vertices
        Vertex G = new Vertex("G");
        graph2Vertices.add(G);
        Vertex H = new Vertex("H");
        graph2Vertices.add(H);
        Vertex I = new Vertex("I");
        graph2Vertices.add(I);
        Vertex J = new Vertex("J");
        graph2Vertices.add(J);
        Vertex K = new Vertex("K");
        graph2Vertices.add(K);
        Vertex L = new Vertex("L");
        graph2Vertices.add(L);
        Vertex M = new Vertex("M");
        graph2Vertices.add(M);
        Vertex N = new Vertex("N");
        graph2Vertices.add(N);
        Vertex P = new Vertex("P");
        graph2Vertices.add(P);

        // add all of the edges and corresponding weights
        G.neighbors = new Edge[]{new Edge(H, 4),
                new Edge(M, 7)};
        H.neighbors = new Edge[]{new Edge(G, 4),
                new Edge(I, 9),
                new Edge(M, 11),
                new Edge(N, 20)};
        I.neighbors = new Edge[]{new Edge(H, 9),
                new Edge(J, 6),
                new Edge(K, 2)};
        J.neighbors = new Edge[]{new Edge(I, 6),
                new Edge(K, 10),
                new Edge(L, 5)};
        K.neighbors = new Edge[]{new Edge(I, 2),
                new Edge(J, 10),
                new Edge(L, 15),
                new Edge(N, 1),
                new Edge(P, 5)};
        L.neighbors = new Edge[]{new Edge(K, 15),
                new Edge(J, 5),
                new Edge(P, 12)};
        M.neighbors = new Edge[]{new Edge(G, 7),
                new Edge(H, 11),
                new Edge(N, 1)};
        N.neighbors = new Edge[]{new Edge(H, 20),
                new Edge(K, 1),
                new Edge(M, 1),
                new Edge(P, 3)};
        P.neighbors = new Edge[]{new Edge(K, 5),
                new Edge(L, 12),
                new Edge(N, 3)};

        ///////////////////////////////
        /////// PRINT RESULTS /////////
        ///////////////////////////////

        System.out.print("\nThis program demonstrates Dijkstra's Algorithm. Dijkstra's algorithm is an algorithm\n" +
                "designed to determine the shortest path between nodes in a graph, and more specifically\n" +
                "undirected weighted graphs in this demonstration. Two example graphs are represented\n" +
                "below by tables, detailing the vertices in each graph as well as the distances to\n" +
                "each of the vertices' neighbors. In these examples, the sources are node A (Graph 1) and \n" +
                "node G (Graph 2). In other words, Dijkstra's algorithm will find the shortest paths\n" +
                "to each of the nodes in the graphs originating from the graphs' respective sources. This is\n" +
                "shown by specifying the shortest distance as well as the path one must take to achieve\n" +
                "this distance.\n\n");

        // print graph 1 header
        System.out.print("//////////////////////////////\n" +
                "//////  GRAPH 1   ////////////\n" +
                "///// 6 VERTICES  ////////////\n" +
                "//////////////////////////////\n\n");

        // print graph 1 representation
        System.out.println(" Vertex |    Adjacent Vertices");
        System.out.println("---------------------------------");
        System.out.println("    A   | B: 4, C: 2");
        System.out.println("    B   | A: 4, C: 1, D: 5");
        System.out.println("    C   | A: 2, B: 1, D: 8, E: 10");
        System.out.println("    D   | B: 5, C: 8, E: 2, F: 6");
        System.out.println("    E   | C: 10, D: 2, F: 5");
        System.out.println("    F   | D: 6, E: 5");

        // make vertex A the source of graph 1
        tracePathsFromSource(A); // run Dijkstra
        System.out.println("\nSource: A");

        // determine shortest paths from A to each of the vertices in graph 1
        for (Vertex v : graph1Vertices) {
            System.out.println();
            System.out.println("The shortest distance from A to " + v + ": " + (int) v.distance);
            List<Vertex> path = getShortestPath(v);
            printPath(path, v);
        }

        System.out.print("\n\n");

        // print graph 2 header
        System.out.print("//////////////////////////////\n" +
                "//////  GRAPH 2   ////////////\n" +
                "/////  9 VERTICES ////////////\n" +
                "//////////////////////////////\n\n");

        // print graph 2 representation
        System.out.println(" Vertex |     Adjacent Vertices");
        System.out.println("------------------------------------------");
        System.out.println("    G   | H: 4, M: 7");
        System.out.println("    H   | G: 4, I: 9, M: 11, N: 20");
        System.out.println("    I   | H: 9, J: 6, K: 2");
        System.out.println("    J   | I: 6, K: 10, L: 5");
        System.out.println("    K   | I: 2, J: 10, L: 15, N: 1, P: 5");
        System.out.println("    L   | K: 15, J: 5, P: 12");
        System.out.println("    M   | G: 7, H: 11, N: 1");
        System.out.println("    N   | H: 20, K: 1, M: 1, P: 3");
        System.out.println("    P   | K: 5, L: 12, N: 3");

        // make vertex G the source of graph 2
        tracePathsFromSource(G); // run Dijkstra
        System.out.println("\nSource: G");

        // determine shortest paths from G to each of the vertices in graph 2
        for (Vertex v : graph2Vertices) {
            System.out.println();
            System.out.println("The shortest distance from G to " + v + ": " + (int) v.distance);
            List<Vertex> path = getShortestPath(v);
            printPath(path, v);
        }

    }

    public static class Vertex implements Comparable<Vertex> {
        // create properties of vertex
        public final String name;
        public Edge[] neighbors;
        public double distance = Double.POSITIVE_INFINITY;
        public Vertex previous;

        // create vertex given its name
        public Vertex(String name) {
            this.name = name;
        }

        // print name of vertex if printing vertex
        public String toString() {
            return this.name;
        }

        // use the distance to the source node when comparing vertices
        // utilized when sorting within a priority queue
        public int compareTo(Vertex other) {
            return Double.compare(distance, other.distance);
        }
    }

    public static class Edge {
        // create properties of edge
        public final Vertex destination;
        public final double weight;

        // create edge given its destination and weight (distance)
        public Edge(Vertex destination, double weight) {
            this.destination = destination;
            this.weight = weight;
        }
    }
}




