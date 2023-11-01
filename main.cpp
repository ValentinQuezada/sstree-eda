#include <iostream>
#include <vector>
#include <random>
#include "SStree.h"
#include "indexing.cpp"

void tree() {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1000.0, 1000.0);

    std::vector<Point> points(500, Point(50));
    for (int i = 0; i < 500; i++) {
        for (int j = 0; j < 50; j++) {
            points[i][j] = dis(gen);
        }
    }

    // Insert the points into the SStree
    SsTree tree(0);
    for (auto& point : points) {
        tree.insert(point);
    }

    std::string filename = "sstree.dat";
    tree.saveToFile(filename);

    tree = SsTree(0);    // Clean the tree
    tree.loadFromFile(filename);

    tree.print();
}

/*
void indexing() {
    const std::string FILE_NAME("embedding.json");
    ImageData data = readEmbeddingsFromJson(FILE_NAME);

    SsTree tree(data.embeddings[0].dim());

    for (size_t i = 0; i < data.embeddings.size(); ++i) {
        tree.insert(data.embeddings[i], data.paths[i]);
    }

    std::string filename = "embbeding.dat";
    tree.saveToFile(filename);
}*/

int main() {
    tree();
    return 0;
}
