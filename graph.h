//
// Created by juan-diego on 3/11/24.
//

#ifndef HOMEWORK_GRAPH_GRAPH_H
#define HOMEWORK_GRAPH_GRAPH_H


#include "window_manager.h"
#include "node.h"
#include "edge.h"


// *
// ---- Graph ----
// Esta clase contiene la estructura del grafo en si misma. Recordemos que un grafo G se define como G = (V, E),
// donde V es un conjunto de vertices y E un conjunto de aristas (a, b), donde a y b pertenecen a V.
//
// Variables miembro
//     - nodes         : Todos los nodos de nuestro grafo
//     - edges         : Todas las aristas de nuestro grafo
//     - window_manager: Se usa para que el grafo pueda dibujarse en el frame actual
//
// Funciones miembro
//     - parse_csv     : Lee las aristas y vértices desde los csv
//     - draw          : Dibuja las aristas y luego los vertices del grafo sobre la ventana
//     - reset         : Restaura los colores de vértices y aristas a sus colores por defecto
// *
struct Graph {
    WindowManager *window_manager;
    std::map<size_t, Node *> nodes;
    std::vector<Edge *> edges;

    explicit Graph(WindowManager* window_manager): window_manager(window_manager) {}

    void parse_csv(const std::string &nodes_path, const std::string &edges_path) {
        Node::parse_csv(nodes_path, this->nodes);
        Edge::parse_csv(edges_path, this->edges, this->nodes);

        for (Edge *edge: edges) {
            nodes[edge->src->id]->edges.push_back(edge);
            if (!edge->one_way) {
                nodes[edge->dest->id]->edges.push_back(edge);
            }
        }
    }

    void draw() {
        for (Edge *edge: edges) {
            edge->draw(window_manager->get_window());
        }
        for (auto &[_, node]: nodes) {
            node->draw(window_manager->get_window());
        }
    }
};



#endif //HOMEWORK_GRAPH_GRAPH_H
