//
// Created by juan-diego on 3/11/24.
//

#ifndef HOMEWORK_GRAPH_EDGE_H
#define HOMEWORK_GRAPH_EDGE_H

#include <SFML/Graphics.hpp>
#include "node.h"
#include <cstring>
#include <fstream>
#include <cmath>

// Color por defecto de todas las aristas (usado por SFML)
sf::Color default_edge_color = sf::Color(255, 200, 100);
// Grosor por defecto de todas las aristas (usado por SFML)
float default_thickness = 0.8;


// No tocar esta clase
class sfLine : public sf::Drawable {
public:
    sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
    : thickness(thickness) {
        sf::Vector2f direction = point2 - point1;
        sf::Vector2f unitDirection = direction/std::sqrt(direction.x*direction.x+direction.y*direction.y);
        sf::Vector2f unitPerpendicular(-unitDirection.y,unitDirection.x);

        sf::Vector2f offset = (this->thickness/2.f)*unitPerpendicular;

        Vertices[0].position = point1 + offset;
        Vertices[1].position = point2 + offset;
        Vertices[2].position = point2 - offset;
        Vertices[3].position = point1 - offset;

        for (auto & vertex : Vertices) {
            vertex.color = color;
        }
    }

    void draw(sf::RenderTarget &target, sf::RenderStates states) const override {
        target.draw(Vertices,4,sf::Quads);
    }

private:
    sf::Vertex Vertices[4];
    float thickness;
    sf::Color color {};
};


// *
// ---- Edge ----
// Esta estructura contiene la informacion de una arista
//
// Variables miembro
//     - src           : Vértice inicial de la arista
//     - dest          : Vértice final de la arista
//     - max_speed     : Velocidad maxima en la que se puede ir de 'src' a 'dest'
//     - length        : Longitud de la trayectoria de 'src' a 'dest'
//     - one_way       : Si es falso, significa que tambien existe una arista en el grafo de 'dest' a 'src'.
//                       Caso contrario, significa que solo existe la arista actual de 'src' a 'dest'
//     - lanes         : La cantidad de carriles en el camino de 'src' a 'dest'
//     - color         : El color de la linea que conecta a 'src' y 'dest', es usado por SFML
//     - thickness     : El grosor de la linea que conecta a 'src' y 'dest', es usado por SFML
//
// Funciones miembro
//     - parse_csv     : Lee las aristas desde el csv
//     - draw          : Dibuja la arista instanciada
//     - reset         : Setea 'color' y 'thickness' a sus valores por defecto
// *
struct Edge {
    Node *src = nullptr;
    Node *dest = nullptr;
    int max_speed;
    double length;
    bool one_way;
    int lanes;

    sf::Color color = default_edge_color;
    float thickness = default_thickness;

    explicit Edge(Node *src, Node *dest, int max_speed, double length, bool one_way, int lanes) : max_speed(max_speed),
                                                                                                  length(length),
                                                                                                  one_way(one_way),
                                                                                                  lanes(lanes),
                                                                                                  src(src),
                                                                                                  dest(dest) {
    }

    static void
    parse_csv(const std::string &edges_path, std::vector<Edge *> &edges, std::map<std::size_t, Node *> &nodes) {
        edges.reserve(790'509);

        std::ifstream file(edges_path);
        char *header = new char[50];
        header[49] = '\0';
        file.getline(header, 50, '\n');
        delete[] header;

        while (true) {
            char *src, *dest, *max_speed, *length, *oneway, *lanes;

            src = new char[15];
            for (int i = 0; i < 15; ++i) src[i] = '\0';
            dest = new char[15];
            for (int i = 0; i < 15; ++i) dest[i] = '\0';
            max_speed = new char[5];
            for (int i = 0; i < 5; ++i) max_speed[i] = '\0';
            length = new char[20];
            for (int i = 0; i < 20; ++i) length[i] = '\0';
            oneway = new char[6];
            for (int i = 0; i < 6; ++i) oneway[i] = '\0';
            lanes = new char[3];
            for (int i = 0; i < 3; ++i) lanes[i] = '\0';

            file.getline(src, 15, ',');
            file.getline(dest, 15, ',');
            file.getline(max_speed, 5, ',');
            file.getline(length, 20, ',');
            file.getline(oneway, 6, ',');
            file.getline(lanes, 3, '\n');

            if (file.eof()) {
                break;
            }

            std::size_t src_id = static_cast<size_t>(std::stoll(src));
            std::size_t dest_id = static_cast<size_t>(std::stoll(dest));

            Node *src_node = nodes[src_id];
            Node *dest_node = nodes[dest_id];

            Edge *edge = new Edge(
                    src_node,
                    dest_node,
                    std::stoi(max_speed),
                    std::stod(length),
                    std::strcmp(oneway, "True") == 0,
                    std::stoi(lanes)
            );
            edges.push_back(edge);

            delete[] src;
            delete[] dest;
            delete[] oneway;
            delete[] length;
            delete[] lanes;
        }
    }

    void draw(sf::RenderWindow &window) const {
        sfLine line(src->coord, dest->coord, color, thickness);
        line.draw(window, sf::RenderStates::Default);
    }
};


#endif //HOMEWORK_GRAPH_EDGE_H
