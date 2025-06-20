//
// Created by juan-diego on 3/11/24.
//

#ifndef HOMEWORK_GRAPH_NODE_H
#define HOMEWORK_GRAPH_NODE_H

#include <SFML/Graphics.hpp>
#include <fstream>

// Color por defecto de un vertice (usado por SFML)
sf::Color default_node_color = sf::Color(150, 40, 50);
// Radio por defecto de un vertice (usado por SFML)
float default_radius = 0.4f;


struct Edge;


// *
// ---- Node ----
// Esta estructura contiene la informacion de un Nodo
//
// Variables miembro
//     - id            : Identificador de un vertice, debe ser irrepetible entre vertices
//     - coord         : La coordenada donde se encuentra el vertice (usado por SFML)
//     - edges         : El conjunto de aristas asociadas al vertice
//     - color         : Color del vertice (usado por SFML)
//     - radius        : Radio del vertice (usado por SFML)
//
// Funciones miembro
//     - parse_csv     : Lee los vertices desde el csv
//     - draw          : Dibuja el vertice instanciado
//     - reset         : Setea 'color' y 'radius' a sus valores por defecto
// *
struct Node {
    std::size_t id;
    sf::Vector2f coord;
    std::vector<Edge *> edges {};

    sf::Color color = default_node_color;
    float radius = default_radius;

    explicit Node(std::size_t id, float x, float y) : id(id), coord(x, y) {}

    static void parse_csv(const std::string &nodes_path, std::map<std::size_t, Node *> &nodes) {
        std::ifstream file(nodes_path);
        char *header = new char[40];
        header[39] = '\0';
        file.getline(header, 40, '\n');
        delete[] header;

        while (true) {
            char *id, *x, *y;
            id = new char[15];
            for (int i = 0; i < 15; ++i) id[i] = '\0';
            y = new char[15];
            for (int i = 0; i < 15; ++i) y[i] = '\0';
            x = new char[15];
            for (int i = 0; i < 15; ++i) x[i] = '\0';

            file.getline(id, 15, ',');
            file.getline(y, 15, ',');
            file.getline(x, 15, '\n');

            if (file.eof()) {
                break;
            }

            std::size_t identifier = static_cast<size_t>(std::stoll(id));
            nodes.insert({
                                 identifier,
                                 new Node(identifier, std::stof(y), std::stof(x))});

            delete[] id;
            delete[] y;
            delete[] x;
        }
    }

    void draw(sf::RenderWindow &window) const {
        sf::CircleShape point(radius);
        point.setPosition(coord);
        point.setFillColor(color);

        window.draw(point);
    }

    void reset() {
        color = default_node_color;
        radius = default_radius;
    }
};


#endif //HOMEWORK_GRAPH_NODE_H
