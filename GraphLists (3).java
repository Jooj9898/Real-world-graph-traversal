// Simple weighted graph representation 
// Uses an Adjacency Linked Lists, suitable for sparse graphs

import java.io.*;
import java.util.LinkedList;
import java.util.Queue;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.IOException;

enum C {White, Grey, Black};

class Heap
{
    private int[] a;	   // heap array
    private int[] hPos;	   // hPos[h[k]] == k
    private int[] dist;    // dist[v] = priority of v

    private int N;         // heap size
   
    // The heap constructor gets passed from the Graph:
    //    1. maximum heap size
    //    2. reference to the dist[] array
    //    3. reference to the hPos[] array
    public Heap(int maxSize, int[] _dist, int[] _hPos) 
    {
        N = 0;
        a = new int[maxSize + 1];
        dist = _dist;
        hPos = _hPos;
    }


    public boolean isEmpty() 
    {
        return N == 0;
    }


    public void siftUp( int k) 
    {
        int v = a[k];

        // code yourself
        // must use hPos[] and dist[] arrays
        
        //priority is the distance of our initial node
        int priority = dist[v];

        //first index of array is set to 0 as a dummy value which protects from edge cases
        a[0] = 0;
        //by setting the distance to maximum we ensure the dummy value is never compared with a node
        dist[0] = Integer.MAX_VALUE;

        //while the distance of our current node is less than the distance of its parent, swap
        while (k > 1 && priority < dist[a[k/2]]) {
            //perform swap of the nodes
            int parent = a[k / 2];
            a[k] = parent;
            hPos[parent] = k;
            //update the new position to be the parent
            k = k / 2;
        }
        //place the original node back in its original place
        a[k] = v;
        //add the node to hPos to keep track of the positions of nodes in our tree
        hPos[v] = k;
    }


    public void siftDown( int k) 
    {
        int v, j;
       
        v = a[k];  
        
        // code yourself 
        // must use hPos[] and dist[] arrays
        //setting the priority as the distance of the current node
        int priority = dist[v];
        
        //while loop continues until we reach a leaf node (end of the tree)
        while (2*k <=N)
        {
            //initialising j to the left child of the current node
            j = 2* k;
            //condition checks that children are in the heap bounds, and compares the priority of the left and right child
            if(j< N && dist[a[j]] > dist[a[j+1]])
            {
                //if the distance of the left child is bigger than right child (right child has higher priority) we now point at the right child by incrementing j
                j++;
            }
            //if the distance of the current node is smaller than the child, then we dont need to swap and break out of the loop
            if( priority <= dist[a[j]])
            {
                break;
            }
            //swapping the parent node and the child node
            a[k] = a[j];
            hPos[a[k]] = k;
            k = j;
        }
        //returning the original node to its position
        a[k] = v;
        //adding the node to the hPos to reflect the position of the nodes in the tree
        hPos[v] = k;
    }


    public void insert( int x) 
    {
        a[++N] = x;
        siftUp( N);
    }


    public int remove() 
    {   
        int v = a[1];
        hPos[v] = 0; // v is no longer in heap
        a[N+1] = 0;  // put null node into empty spot
        
        a[1] = a[N--];
        siftDown(1);
        
        return v;
    }

}

class Graph {
    class Node {
        public int vert;
        public int wgt;
        public Node next;
    }
    
    // V = number of vertices
    // E = number of edges
    // adj[] is the adjacency lists array
    private int V, E;
    private Node[] adj;
    private Node z;
    private int[] mst;
    //adding a an array to store the shortest path tree for representation as well
    private int[] spt;
    // used for traversing graph
    private C[] colour;
    private int time;

    private int[] parent, d, f ;

    private int[] visited;
    private int id;
    
    
    // default constructor
    public Graph(String graphFile)  throws IOException
    {
        int u, v;
        int e, wgt;
        Node t;

        FileReader fr = new FileReader(graphFile);
		BufferedReader reader = new BufferedReader(fr);
	           
        String splits = " +";  // multiple whitespace as delimiter
		String line = reader.readLine();        
        String[] parts = line.split(splits);
        System.out.println("Parts[] = " + parts[0] + " " + parts[1]);
        
        V = Integer.parseInt(parts[0]);
        E = Integer.parseInt(parts[1]);
        
        // create sentinel node
        z = new Node(); 
        z.next = z;
        
        // create adjacency lists, initialised to sentinel node z       
        adj = new Node[V+1];        
        for(v = 1; v <= V; ++v)
            adj[v] = z;      
        
        // initialize for DFS
        colour = new C[V + 1];
        parent = new int[V + 1];
        d = new int[V + 1];
        f = new int[V + 1];
        
       // read the edges
        System.out.println("Reading edges from text file");
        for(e = 1; e <= E; ++e)
        {
            line = reader.readLine();
            parts = line.split(splits);
            u = Integer.parseInt(parts[0]);
            v = Integer.parseInt(parts[1]); 
            wgt = Integer.parseInt(parts[2]);
            
            System.out.println("Edge " + toChar(u) + "--(" + wgt + ")--" + toChar(v));   

           
            // write code to put edge into adjacency list    
            //making a new node that represents destination being edge V
            t = new Node();
            //vertex is v
            t.vert = v;
            t.wgt = wgt;
            //v is pointing to the next node in the adjacency list (current head)
            t.next = adj[u];
            //link new node to existing list (updating head)
            adj[u] = t;

            //the same as previously but for destination
            t = new Node();
            t.vert = u;
            t.wgt = wgt;
            t.next = adj[v];
            adj[v] = t;
            
        }	       
    }
   
    // convert vertex into char for pretty printing
    private char toChar(int u)
    {  
        return (char)(u + 64);
    }
    
    // method to display the graph representation
    public void display() {
        int v;
        Node n;
        
        for(v=1; v<=V; ++v){
            System.out.print("\nadj[" + toChar(v) + "] ->" );
            for(n = adj[v]; n != z; n = n.next) 
                System.out.print(" |" + toChar(n.vert) + " | " + n.wgt + "| ->");    
        }
        System.out.println("");
    }


    
	public void MST_Prim(int s)
	{
        int v, u;
        int wgt, wgt_sum = 0;
        int[]  dist, parent, hPos;
        Node t;

        //code here
        //initialising all the integer arrays
        dist = new int[V + 1];
        hPos = new int[V +1];
        parent = new int[V + 1];
        
        for(v =1; v <= V; v++)
        {
            //dist set to infinity from source to ensure dummy value never compares with node
            dist[v] = Integer.MAX_VALUE;
            parent[v] = 0;
            hPos[v] = 0;
        }
        //distance from the source to itself is 0
        dist[s] = 0;
        
        //create new min heap 
        Heap h =  new Heap(V, dist, hPos);
        
        //insert the source node into our min-heap
        h.insert(s);
        
        //while the heap is not empty
        while (!h.isEmpty())  
        {
            //extract the weight of the minimum distance
            v = h.remove();

            //incrementing the weight sum to = weight + every new distance that is added to the heap
            wgt_sum += dist[v];
            
            //this is used to flag that dist of v is added to the MST
            dist[v] = -dist[v];
            
            //iterating or checking every neighbouring node u of v
            for(t = adj[v]; t!= z ; t = t.next)
            {
                //u is the vertex between u->v
                u = t.vert;
                
                //if the weight of the neighbouring node is less than the distance of dist[u] aka smallest edge linking u 
                if(t.wgt < dist[u] && dist[u] >= 0)
                {
                    //if true, update the new dist
                    dist[u] = t.wgt;
                    //assign parent of u now to v (the edge we're connecting current node via)
                    parent[u] = v;
                    //if u is not in the heap
                    if(hPos[u] == 0){
                        //insert u
                        h.insert(u);
                    }
                    else{
                        //if it is already, the position is updated
                        h.siftUp(hPos[u]);
                    }
                }
            }
            
            
        }
        //store the result (the MST) in the mst int array from the Graph class
        mst = parent;
        showMST();
        System.out.print("\n\nWeight of MST = " + wgt_sum + "\n\n");
       		
	}
    
    //method to display the minimum spanning tree
    public void showMST()
    {
            System.out.print("\n\nMinimum Spanning tree parent array is:\n");
            for(int v = 1; v <= V; ++v)
                System.out.println(toChar(v) + " -> " + toChar(mst[v]));
            System.out.println("");
    }

    public void SPT_Dijkstra(int s)
    {
        //initialise variables
        int v, u, d;
        int wgt, wgt_sum = 0;
        int[]  dist, parent, hPos;
        Node t;

        //initialise new integer arrays
        dist = new int[V + 1];
        hPos = new int[V +1];
        parent = new int[V + 1];
        
        for(v =1; v <= V; v++)
        {
            //dist set to infinity to avoid nodes being compared with the dummy value
            dist[v] = Integer.MAX_VALUE;
            parent[v] = 0;
            hPos[v] = 0;
        }
        //initialising a new priority queue
        Heap pq = new Heap(V, dist, hPos);
        
        //adding the source node to the priority queue(the node we are starting from)
        pq.insert(s);
        
        //distance from source node to source is 0
        dist[s] = 0;

        //while the priority queue isnt empty
        while(!pq.isEmpty())
        {
            //v is set to the minimum distance which is extracted using remove
            v = pq.remove();
            
            //weight sum incremented (adds each new weight every time)
            //for examining each neighbour of v
            for(t = adj[v]; t!=z; t= t.next)
            {
                //d is the weight of the neighbouring node
                d = t.wgt;
                //u is the vertex of the neighbouring node
                u = t.vert;
                //if the total distance from the source -> v -> u (current shortest distance + weight of v -> u) is less than the distance from source straight to u
                if(dist[v] + d < dist[u])
                {
                    //update the current shortest distance to now be distance of source -> v -> u
                    dist[u] = dist[v] + d;
                    //if u is not in the heap
                    if(hPos[u] == 0)
                    {
                        //insert u 
                        pq.insert(u);
                    }
                    
                    else
                    {
                        //update the position of u in the heap
                        pq.siftUp(hPos[u]);
                    }
                    //the parent of u is now v
                    parent[u] = v;
                    
                }
            }
        }

        //for loop to display the distance from L to each vertext before entire parent array
        for(int i =0; i<=V; i++)
        {
            if(parent[i] != 0)
            {
                System.out.println("Shortest path from " + toChar(s) + " to " + toChar(i) + " is " + dist[i]);
            }
        }
        //store the result of the SPT into the int array spt from Graphs
        spt = parent;
        showSPT(); 
    }

    //method to display the result of the SPT
    public void showSPT()
    {
        System.out.print("\n\nShortest Path Tree parent array is:\n");
        for(int v = 1; v <= V; ++v)
            System.out.println(toChar(v) + " <- " + toChar(spt[v]));
        System.out.println("");
    }

    //method to perform a depth first search on graph
    public void DF(int s)
    {   
        System.out.println("\n\nDepth First Search vertex visits");
        int v;
        
        //foreach vertex in the graph, we set the colour to white (unvisited) and parent to 0 (no parent)
        for(v=1; v <= V; ++v)
        {
            colour[v] = C.White;
            parent[v] = 0;      
        }
        //starting from the source node
        time = 0;
        dfVisit(s);
        for(v = 1; v<=V; ++v)
        {
            //if the vertex has not been visited yet, we call the recursive method to visit it
            if(colour[v] == C.White)
            {
                //recursively calling dfVisit to visit the vertex
                dfVisit(v);
            }
        }
    }
    
    //method that depth is recursively called by DF to depth first search
    public void dfVisit(int v)
    {
        
        Node t;
        
        //time is incremented to reflect the time of the vertex being visited
        ++time;
        //setting the discovery time of the vertex to the time
        d[v] = time;
        //setting the colour of the vertex to grey (being visited)
        colour[v] = C.Grey;

        //source node
        if(parent[v] ==0)
        {
            System.out.println("\nvisited vertex " + toChar(v));
        }
        //following nodes
        else
        {
            //print statement to display the vertex being visited and the edge it was reached by
            System.out.println("visited vertex " + toChar(v) + " along edge " + toChar(parent[v]) + "--" + toChar(v));
        }
        
        //visiting every neighbour of v
        for(t = adj[v]; t != z; t = t.next)
        {
            //u is the vertex between u->v
            int u = t.vert;
            
            //if the vertext hasnt been visited, recursively call dfVisit, following a whole path until reaching the end (deep search) 
            if(colour[u] == C.White)
            {
                parent[u] = v;
                dfVisit(u);
            }
        }
        //after processing all neighbours of v, we mark it as black (visited)
        colour[v] = C.Black;
        //increment the time and set the finish time of the vertex
        time++;
        f[v] = time;
    }

    //method to perform a breadth first search traversal on a graph
    public void breadthFirst(int s)
    {
        System.out.println("\n\nBreadth First Search vertex visits\n");
        Node t;  
        int v, u;    
        //creating a queue which will hold what vertices will be visited when
        Queue<Integer> q = new LinkedList<>();

        //foreach vertex in the graph, we set the colour to white (unvisited), distance to infinity, and parent to 0 (no parent)
        for(v =1; v <= V; v++)
        {
            colour[v] = C.White;
            d[v] = Integer.MAX_VALUE;
            parent[v] = 0;
        }
        //starting from the source node or adding source node to the queue
        colour[s] = C.Grey;
        d[s] = 0;
        parent[s] = 0;
        q.add(s);
        
        //whilst the queue is not empty
        while(!q.isEmpty())
        {
            //remove the next vertex to be processed (source)
            u = q.remove();
            //if the vertex just extracted has not been visited yet   
            //print statement to display vertexes being visited
                
                //visiting each neighbour of v
                for(t = adj[u]; t!=z; t = t.next)
                {
                    //u is the vertex between u->v
                    v = t.vert;
                    //if the neighbour of v has not been visited
                    if(colour[v] == C.White)
                    {
                        
                        colour[v] = C.Grey;
                        d[v] = d[u] + 1; //distance from source
                        parent[v] = u; //parent of v is now u
                        System.out.println("visited vertex " + toChar(v)  +" along edge " + toChar(u) + "--" + t.wgt + "--" + toChar(v));
                        //add the vertex to the queue to be processed
                        q.add(v);
                    }
                }
                //after processing all neighbours of v, we mark it as black (visited)
                colour[v] = C.Black;
            }
        }

    }

public class GraphLists {
    public static void main(String[] args) throws IOException
    {
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));

        System.out.println("Enter the name of the graph file:");
        String fname = reader.readLine(); 
        
        System.out.println("Enter the starting vertex:");
        String source = reader.readLine();
        int s = Integer.parseInt(source);              

        Graph g = new Graph(fname);
       
        //displaying the graph 
        g.display();

       //1. performing a depth first search traversal on our graph
       g.DF(s);
       
       //2. performing a breadth first search traevrsal on our graph
       g.breadthFirst(s);
       
       //3. running prims algorithm on the graph to find the minimum spanning tree and calculate its weight
       g.MST_Prim(s);   
       
       //4. running dijkstras algorithm on the graph to find the shortest path tree and calculate its weight
       g.SPT_Dijkstra(s);               
    }
}
