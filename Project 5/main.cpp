/*
Project 5a
Main.cpp
Fouad Al-Rijleh, Rachel Rudolph
*/

#include <iostream>
#include <limits.h>
#include "d_except.h"
#include <list>
#include <fstream>
#include "d_matrix.h"
#include <queue>
#include <vector>
#include <stack>

#include <boost/graph/adjacency_list.hpp>
#include "heapV.h"
//#include "disjointSetV.h"

#define LargeValue 99999999

using namespace std;
using namespace boost;

int const NONE = -1;  // Used to represent a node that does not exist

struct VertexProperties;
struct EdgeProperties;

typedef adjacency_list<vecS, vecS, bidirectionalS, VertexProperties, EdgeProperties> Graph;

struct VertexProperties
{
	pair<int, int> cell; // maze cell (x,y) value
	Graph::vertex_descriptor pred;
	bool visited;
	bool marked;
	int weight;
};

// Create a struct to hold properties for each edge
struct EdgeProperties
{
	int weight;
	bool visited;
	bool marked;
};

//Output operator for the Graph class. Prints out all nodes and their properties, 
//and all edges and their properties.
ostream &operator<<(ostream &ostr, const Graph &g)
{
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{

		ostr << "Node " << *vItr;
		//ostr << " | Marked: " << g[*vItr].marked;
		ostr << " | Visited: " << g[*vItr].visited << endl;
	}
	cout << endl;

	pair<Graph::edge_iterator, Graph::edge_iterator> eItrRange = edges(g);
	for (Graph::edge_iterator eItr = eItrRange.first; eItr != eItrRange.second; ++eItr)
	{

		ostr << "Edge " << *eItr;
		//ostr << " | Marked: " << g[*eItr].marked;
		ostr << " | Visited: " << g[*eItr].visited << endl;
		//ostr << "Weight: " << g[*eItr].weight << endl << endl;
	}

	return ostr;
}

typedef adjacency_list<vecS, vecS, bidirectionalS, VertexProperties, EdgeProperties> Graph;

//Unvisits all nodes in g
void clearVisited(Graph &g)
{
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{
		g[*vItr].visited = false;
	}
}

//Looks for shortest path from start to goal using BFS
void findShortestPathBFS(Graph &g)
{
	clearVisited(g);
	queue<Graph::vertex_descriptor> queue;
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	Graph::vertex_iterator vItr = vItrRange.first;
	queue.push(*vItr);
	g[*vItr].visited = true;
	while (queue.size() != 0)
	{
		Graph::vertex_descriptor v = queue.front();
		pair<Graph::adjacency_iterator, Graph::adjacency_iterator> vItrAdjRange = adjacent_vertices(v, g);
		for (Graph::adjacency_iterator w = vItrAdjRange.first; w != vItrAdjRange.second; w++)
		{
			if (g[*w].visited == false)
			{
				g[*w].visited = true;
				queue.push(*w);
			}
		}
		queue.pop();
	}
}


//Looks for path from start to goal using DFS using a stack not recursion
void findPathDFSStack(Graph &g)
{
	clearVisited(g);
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	Graph::vertex_iterator vItr = vItrRange.first;
	stack<Graph::vertex_descriptor> stack;
	stack.push(*vItr);
	g[*vItr].visited = true;
	while (stack.size() != 0)
	{
		Graph::vertex_descriptor v = stack.top();
		pair<Graph::adjacency_iterator, Graph::adjacency_iterator> vItrAdjRange = adjacent_vertices(v, g);
		for (Graph::adjacency_iterator w = vItrAdjRange.first; w != vItrAdjRange.second; w++)
		{
			if (g[*w].visited == false)
			{
				g[*w].visited = true;
				stack.push(*w);
			}
			else
			{
				stack.pop();
			}
		}
	}
}

bool isConnected(Graph &g)
{
	if (num_vertices(g) == 0) return false;

	findShortestPathBFS(g);
	//findPathDFSStack(g);
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{
		if (g[*vItr].visited == false) return false;
	}
	return true;
}

//Modified from DFSStack - true if a sub_cycle is found
bool subCycle(Graph &g, pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange)
{
	queue<Graph::vertex_descriptor> queue;
	Graph::vertex_iterator vItr = vItrRange.first;
	queue.push(*vItr);
	g[*vItr].visited = true;
	while (queue.size() != 0)
	{
		Graph::vertex_descriptor v = queue.front();
		pair<Graph::adjacency_iterator, Graph::adjacency_iterator> vItrAdjRange = adjacent_vertices(v, g);
		for (Graph::adjacency_iterator w = vItrAdjRange.first; w != vItrAdjRange.second; w++)
		{
			if (g[*w].visited == false)
			{
				g[*w].visited = true;
				g[*w].pred = v;
				queue.push(*w);
			}
			else if (g[v].pred != *w && v != *vItrRange.first)
			{
				Graph::vertex_descriptor W = *w;
				Graph::vertex_descriptor start = *vItrRange.first;
				return true;
			}
		}
		queue.pop();
	}
	return false;
}

//true if graph contains a cycle
bool isCyclic(Graph &g)
{
	clearVisited(g);
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{
		if (g[*vItr].visited == false)
		{
			vItrRange.first = vItr;
			if (subCycle(g, vItrRange))
			{
				return true;
			}
		}
	}
	return false;
}

//creates a spanning tree of g in graph sf - called multiple times by findSpanningForest
void findSpanningTree(Graph &g, pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange, Graph &sf)
{
	queue<Graph::vertex_descriptor> queue;
	Graph::vertex_iterator vItr = vItrRange.first;
	queue.push(*vItr);
	g[*vItr].visited = true;
	while (queue.size() != 0)
	{
		Graph::vertex_descriptor v = queue.front();
		pair<Graph::adjacency_iterator, Graph::adjacency_iterator> vItrAdjRange = adjacent_vertices(v, g);
		for (Graph::adjacency_iterator w = vItrAdjRange.first; w != vItrAdjRange.second; w++)
		{
			if (g[*w].visited == false)
			{
				g[*w].visited = true;
				g[*w].pred = v;
				queue.push(*w);
				Graph::edge_descriptor e1 = add_edge(*w, v, sf).first;
				Graph::edge_descriptor e2 = add_edge(v, *w, sf).first;
			}
			else if (g[v].pred != *w && v != *vItrRange.first)
			{
				Graph::vertex_descriptor W = *w;
				Graph::vertex_descriptor start = *vItrRange.first;
				return;
			}
		}
		queue.pop();
	}
	return;
}

//creates a spanning forest of graph g in graph sf
void findSpanningForest(Graph &g, Graph &sf)
{
	clearVisited(g);
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{
		if (g[*vItr].visited == false)
		{
			vItrRange.first = vItr;
			findSpanningTree(g, vItrRange, sf);
		}
	}
	return;
}

int totalEdgeWeight(Graph &g)
{
	int cost = 0;
	pair<Graph::edge_iterator, Graph::edge_iterator> eItrRange = edges(g);
	for (Graph::edge_iterator eItr = eItrRange.first; eItr != eItrRange.second; eItr++)
	{
		cost += g[*eItr].weight;
	}
	return cost;
}

void initializeGraph(Graph &g,
	Graph::vertex_descriptor &start,
	Graph::vertex_descriptor &end, ifstream &fin)
	// Initialize g using data from fin.  Set start and end equal
	// to the start and end nodes.
{
	EdgeProperties e;

	int n, i, j;
	int startId, endId;
	fin >> n;
	fin >> startId >> endId;
	Graph::vertex_descriptor v;

	// Add nodes.
	for (int i = 0; i < n; i++)
	{
		v = add_vertex(g);
		if (i == startId)
			start = v;
		if (i == endId)
			end = v;
	}

	while (fin.peek() != '.')
	{
		fin >> i >> j >> e.weight;
		add_edge(i, j, e, g);
	}
}

void mstPrim(Graph &g, Graph &sf, Graph::vertex_descriptor start)
{
	priority_queue<Graph::edge_descriptor> queue;
	heapV<Graph::edge_descriptor, Graph> heap;
	heap.initializeMinHeap(g);
	g[start].visited = true;
	pair<Graph::edge_descriptor, bool> currentPair;
	Graph::edge_descriptor cheapestEdge, currentEdge;
	currentEdge = *edges(g).first; //initialize current edge to first edge -- doesnt matter which
	//cheapestEdge = *edges(g).first; //initialize cheapest edge to first edge -- doesnt matter which
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	queue.push(currentEdge);
	while (queue.size() != 0)
	{
		for (Graph::vertex_iterator u = vItrRange.first; u != vItrRange.second; ++u)
		{
			if (g[*u].visited == true)
				for (Graph::vertex_iterator v = vItrRange.first; v != vItrRange.second; v++)
				{
					if (g[*v].visited == false)
					{
						//current edge goes from marked u to unmarked v
						currentPair = edge(*u, *v, g);
						currentEdge = currentPair.first;
						queue.push(currentEdge);
						heap.minHeapDecreaseKey(*v, g);
						//if (currentPair.second == true //if valid edge
						//	&& g[currentEdge].weight < g[cheapestEdge].weight)
						//	cheapestEdge = currentEdge;
					}
				}
		}
		queue.pop();
		cheapestEdge = queue.top();
		Graph::vertex_descriptor i = source(cheapestEdge, g);
		Graph::vertex_descriptor j = target(cheapestEdge, g);
		if (edge(i, j, sf).second == false)
		{
			add_edge(i, j, sf);
			add_edge(j, i, sf);
			g[j].visited == true;
		}
	}

}

void msfPrim(Graph &g, Graph &sf)
{
	clearVisited(g);
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vItrRange = vertices(g);
	for (Graph::vertex_iterator vItr = vItrRange.first; vItr != vItrRange.second; ++vItr)
	{
		if (g[*vItr].visited == false)
		{
			mstPrim(g, sf, *vItr);
		}
	}
	return;
}

int main()
{
	char x;
	ifstream fin;
	stack <int> moves;
	string fileName;

	// Read the name of the graph from the keyboard or
	// hard code it here for testing.

	//fileName = "graph4.txt";

	cout << "Enter filename" << endl;
	cin >> fileName;

	fin.open(fileName.c_str());
	if (!fin)
	{
		cerr << "Cannot open " << fileName << endl;
		exit(1);
	}

	try

	{
		cout << "Reading graph" << endl;
		Graph g;

		Graph::vertex_descriptor start, end;

		initializeGraph(g, start, end, fin);
		cout << "Num nodes: " << num_vertices(g) << endl;
		cout << "Num edges: " << num_edges(g) << endl;
		cout << endl;

		//cout << g;

		bool connected;
		bool cyclic;

		cout << "Calling isCyclic" << endl;
		cyclic = isCyclic(g);

		if (cyclic)
			cout << "Graph contains a cycle" << endl;
		else
			cout << "Graph does not contain a cycle" << endl;

		cout << endl;

		cout << "Calling isConnected" << endl;
		connected = isConnected(g);

		if (connected)
			cout << "Graph is connected" << endl;
		else
			cout << "Graph is not connected" << endl;

		cout << endl;
		cout << "Finding spanning forest" << endl;

		// Initialize an empty graph to contain the spanning forest
		Graph sf(num_vertices(g));

		//findSpanningForest(g, sf);
		msfPrim(g, sf);

		cout << "Num nodes: " << num_vertices(sf) << endl;
		cout << "Num edges: " << num_edges(sf) / 2 << endl;
		cout << sf << endl;
		cout << endl;

		//cout << "Spanning forest weight: " << totalEdgeWeight(sf)/2 << endl;
		//cout << endl;

		cout << "Calling isConnected" << endl;
		connected = isConnected(sf);

		if (connected)
			cout << "Graph is connected" << endl;
		else
			cout << "Graph is not connected" << endl;
		cout << endl;

		cout << "Calling isCyclic" << endl;
		cyclic = isCyclic(sf);

		if (cyclic)
			cout << "Graph contains a cycle" << endl;
		else
			cout << "Graph does not contain a cycle" << endl;
		cout << endl;
	}
	catch (indexRangeError &ex)
	{
		cout << ex.what() << endl; exit(1);
	}
	catch (rangeError &ex)
	{
		cout << ex.what() << endl; exit(1);
	}

	system("pause");
}


