// See https://aka.ms/new-console-template for more information
//21110437
//emmanuel isaac rodriguez mendez
// arbol de expansion minima (MST) practica 4
using System; 
using System.Collections.Generic;

class Program
{
    // Clase que representa una arista del grafo
    public class Edge : IComparable<Edge>
    {
        public int From { get; set; }
        public int To { get; set; }
        public int Weight { get; set; }

        public Edge(int from, int to, int weight)
        {
            From = from;
            To = to;
            Weight = weight;
        }

        public int CompareTo(Edge other)
        {
            return Weight.CompareTo(other.Weight);
        }
    }

    // Clase que representa un grafo
    public class Graph
    {
        public List<Edge> Edges { get; set; }
        public int NodeCount { get; set; }

        public Graph(int nodeCount)
        {
            NodeCount = nodeCount;
            Edges = new List<Edge>();
        }

        public void AddEdge(int from, int to, int weight)
        {
            Edges.Add(new Edge(from, to, weight));
            Edges.Add(new Edge(to, from, weight)); // Grafo no dirigido
        }
    }

    static void Main(string[] args)
    {
        // Crear el grafo
        Graph graph = new Graph(4);
        graph.AddEdge(0, 1, 50);  // Camino 1: A -> B (50 min)
        graph.AddEdge(0, 2, 120); // Camino 2: A -> B (120 min)
        graph.AddEdge(0, 3, 20);  // Camino 3: A -> C (30 min)
        graph.AddEdge(3, 4, 20);  // Camino adicional: C -> B (20 min)
        // Aquí, consideramos que de D se puede ir a B, sumando el tiempo

        // Ejecutar el algoritmo de Prim para encontrar el Árbol de Expansión Mínima (MST)
        var mst = Prim(graph, 0);

        Console.WriteLine("Aristas en el Árbol de Expansión Mínima:");
        foreach (var edge in mst)
        {
            Console.WriteLine($"{edge.From} - {edge.To} : {edge.Weight} minutos");
        }

        // Calcular la ruta más rápida desde A hasta B en el MST
        var shortestPath = FindShortestPathInMST(mst, 0, 1);

        Console.WriteLine("La ruta más rápida en el Árbol de Expansión Mínima tiene un tiempo de: " + shortestPath + " minutos.");
    }

    public static List<Edge> Prim(Graph graph, int startNode)
    {
        var mst = new List<Edge>();
        var edges = new List<Edge>();
        var visited = new HashSet<int>();

        // Añadir el nodo inicial al conjunto de visitados
        visited.Add(startNode);

        // Añadir todas las aristas del nodo inicial a la lista de aristas
        edges.AddRange(graph.Edges.FindAll(e => e.From == startNode));

        // Mientras haya nodos no visitados
        while (visited.Count < graph.NodeCount)
        {
            // Ordenar las aristas por peso
            edges.Sort();

            Edge edge = edges[0];
            edges.RemoveAt(0);

            if (visited.Contains(edge.To))
                continue;

            // Añadir la arista al MST
            mst.Add(edge);

            // Añadir el nodo al conjunto de visitados
            visited.Add(edge.To);

            // Añadir nuevas aristas desde el nodo recién visitado
            edges.AddRange(graph.Edges.FindAll(e => e.From == edge.To && !visited.Contains(e.To)));
        }

        return mst;
    }

    public static int FindShortestPathInMST(List<Edge> mst, int startNode, int endNode)
    {
        var distances = new Dictionary<int, int>();
        var queue = new Queue<int>();

        foreach (var edge in mst)
        {
            if (!distances.ContainsKey(edge.From))
                distances[edge.From] = int.MaxValue;
            if (!distances.ContainsKey(edge.To))
                distances[edge.To] = int.MaxValue;
        }

        distances[startNode] = 0;
        queue.Enqueue(startNode);

        while (queue.Count > 0)
        {
            int node = queue.Dequeue();

            foreach (var edge in mst)
            {
                if (edge.From == node && distances[edge.To] > distances[node] + edge.Weight)
                {
                    distances[edge.To] = distances[node] + edge.Weight;
                    queue.Enqueue(edge.To);
                }
                else if (edge.To == node && distances[edge.From] > distances[node] + edge.Weight)
                {
                    distances[edge.From] = distances[node] + edge.Weight;
                    queue.Enqueue(edge.From);
                }
            }
        }

        return distances[endNode];
    }
}