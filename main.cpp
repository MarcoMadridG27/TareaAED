#include "gui.h"

int main() {
    GUI gui("nodes.csv", "edges.csv");
    gui.main_loop();
    return 0;
}
