//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar,
    BestFirst
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
    };

    void dijkstra(Graph& graph) {
        std::unordered_map<Node*, double> distance;
        std::unordered_map<Node*, Node*> parent;

        for (const auto& [id, node] : graph.nodes) {
            distance[node] = std::numeric_limits<double>::infinity();
        }

        distance[src] = 0.0;

        using PQItem = std::pair<double, Node*>;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<>> pq;
        pq.emplace(0.0, src);

        while (!pq.empty()) {
            auto [current_cost, current_node] = pq.top();
            pq.pop();

            if (current_node == dest) break;
            if (current_cost > distance[current_node]) continue;

            render();

            for (Edge* edge : current_node->edges) {
                Node* neighbor = (edge->src == current_node) ? edge->dest : edge->src;

                if (edge->one_way && edge->src != current_node) continue;

                double cost_through_current = distance[current_node] + edge->length;

                if (cost_through_current < distance[neighbor]) {
                    distance[neighbor] = cost_through_current;
                    parent[neighbor] = current_node;
                    pq.emplace(cost_through_current, neighbor);

                    visited_edges.emplace_back(
                            current_node->coord,
                            neighbor->coord,
                            sf::Color::Blue,
                            1.5f
                    );
                }
            }
        }

        set_final_path(parent);
    }


    double heuristic(Node* a, Node* b) {
        double dx = a->coord.x - b->coord.x;
        double dy = a->coord.y - b->coord.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    void a_star(Graph& graph) {
        std::unordered_map<Node*, double> gScore;
        std::unordered_map<Node*, double> fScore;
        std::unordered_map<Node*, Node*> parent;

        for (const auto& [id, node] : graph.nodes) {
            gScore[node] = std::numeric_limits<double>::infinity();
            fScore[node] = std::numeric_limits<double>::infinity();
        }

        gScore[src] = 0.0;
        fScore[src] = heuristic(src, dest);

        using QueueEntry = std::pair<double, Node*>;  // (fScore, Node*)
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> openSet;
        openSet.emplace(fScore[src], src);

        while (!openSet.empty()) {
            auto [current_f, current_node] = openSet.top();
            openSet.pop();

            if (current_node == dest) break;

            if (current_f > fScore[current_node]) continue;

            render();

            for (Edge* edge : current_node->edges) {
                Node* neighbor = (edge->src == current_node) ? edge->dest : edge->src;

                if (edge->one_way && edge->src != current_node) continue;

                double tentative_g = gScore[current_node] + edge->length;

                if (tentative_g < gScore[neighbor]) {
                    gScore[neighbor] = tentative_g;
                    fScore[neighbor] = tentative_g + heuristic(neighbor, dest);
                    parent[neighbor] = current_node;
                    openSet.emplace(fScore[neighbor], neighbor);

                    visited_edges.emplace_back(
                            current_node->coord,
                            neighbor->coord,
                            sf::Color::Green,
                            1.5f
                    );
                }
            }
        }

        set_final_path(parent);
    }


    void best_first(Graph& graph) {
        std::unordered_map<Node*, Node*> parent;
        std::unordered_map<Node*, double> bestHeuristic;

        using PQEntry = std::pair<double, Node*>;
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> openSet;

        bestHeuristic[src] = heuristic(src, dest);
        openSet.emplace(bestHeuristic[src], src);

        while (!openSet.empty()) {
            auto [current_h, current_node] = openSet.top();
            openSet.pop();

            if (current_h > bestHeuristic[current_node]) continue;

            render();

            if (current_node == dest) break;

            for (Edge* edge : current_node->edges) {
                Node* neighbor = (edge->src == current_node) ? edge->dest : edge->src;

                if (edge->one_way && edge->src != current_node) continue;

                double h = heuristic(neighbor, dest);

                if (!bestHeuristic.count(neighbor) || h < bestHeuristic[neighbor]) {
                    bestHeuristic[neighbor] = h;
                    parent[neighbor] = current_node;
                    openSet.emplace(h, neighbor);

                    visited_edges.emplace_back(
                            current_node->coord,
                            neighbor->coord,
                            sf::Color::Magenta,
                            1.5f
                    );
                }
            }
        }

        set_final_path(parent);
    }


    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(20));

        window_manager->clear();
        draw(true);
        window_manager->display();
    }



    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        path.clear();
        Node* current = dest;

        while (parent.find(current) != parent.end()) {
            Node* prev = parent[current];
            path.push_back(sfLine(current->coord, prev->coord, sf::Color::Red, 2.5f));
            current = prev;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) return;
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            case BestFirst:
                best_first(graph);
                break;
            default:
                break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
