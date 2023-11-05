#ifndef GAME_DYNAMICS_H
#define GAME_DYNAMICS_H

#include "haphephobia/common.h"
#include "haphephobia/point.h"

// TODO create undirected graph 
// -
// #include <iostream>
// #include <list>
// using namespace std;

// class Graph {
//   int numVertices;
//   list<int> *adjLists;
//   bool *visited;

//    public:
//   Graph(int V);
//   void addEdge(int src, int dest);
//   void DFS(int vertex);
// };

// // Initialize graph
// Graph::Graph(int vertices) {
//   numVertices = vertices;
//   adjLists = new list<int>[vertices];
//   visited = new bool[vertices];
// }// Add edges
// void Graph::addEdge(int src, int dest) {
//   adjLists[src].push_front(dest);
// }

// -
// #include <iostream>
// #include <vector>
// #include <map>
// #include <string>

// using namespace std;

// struct vertex {
//     typedef pair<int, vertex*> ve;
//     vector<ve> adj; //cost of edge, destination vertex
//     string name;
//     vertex(string s) : name(s) {}
// };

// class graph
// {
// public:
//     typedef map<string, vertex *> vmap;
//     vmap work;
//     void addvertex(const string&);
//     void addedge(const string& from, const string& to, double cost);
// };

// void graph::addvertex(const string &name)
// {
//     vmap::iterator itr = work.find(name);
//     if (itr == work.end())
//     {
//         vertex *v;
//         v = new vertex(name);
//         work[name] = v;
//         return;
//     }
//     cout << "\nVertex already exists!";
// }

// void graph::addedge(const string& from, const string& to, double cost)
// {
//     vertex *f = (work.find(from)->second);
//     vertex *t = (work.find(to)->second);
//     pair<int, vertex *> edge = make_pair(cost, t);
//     f->adj.push_back(edge);
// }

// -
// using graph = std::map<int, std::vector<int>>;

// -
// struct edge {
//     int nodes[2];
//     float cost; // add more if you need it
// };

// using graph = std::map<int, std::vector<edge>>;

#endif // GAME_DYNAMICS_H
