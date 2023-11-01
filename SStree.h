#ifndef SSTREE_H
#define SSTREE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <queue>
#include <limits>
#include <fstream>

#include "params.h"
#include "Point.h"

struct Pair {
    Point point;
    NType distance;

    Pair(const Point& p, NType d) : point(p), distance(d) {}
};
struct CompareSafe {
    bool operator()(const Pair& a, const Pair& b) const {
        return a.distance < b.distance; // max-heap basado en distancia
    }
};

class SsNode {
private:
    NType varianceAlongDirection(const std::vector<Point>& centroids, size_t direction) const;
    size_t minVarianceSplit(size_t coordinateIndex);
    
public:
    virtual ~SsNode() = default;

    Point centroid; 
    NType radius;
    SsNode* parent = nullptr;

    virtual bool isLeaf() const = 0;
    virtual std::vector<Point> getEntriesCentroids() const = 0;
    virtual void sortEntriesByCoordinate(size_t coordinateIndex) = 0;
    virtual std::pair<SsNode*, SsNode*> split() = 0;
    virtual bool intersectsPoint(const Point& point) const {
        return distance(this->centroid, point) <= this->radius;
    }

    virtual void updateBoundingEnvelope() = 0;
    size_t directionOfMaxVariance() const;
    size_t findSplitIndex();

    virtual std::pair<SsNode*, SsNode*> insert(const Point& point) = 0;

    bool test(bool isRoot = false) const;
    void print(size_t indent = 0) const;

    virtual void FNDFTrav(const Point &query, size_t k, std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> &res, NType& maxrad, std::vector<std::string>& paths) const = 0;

    virtual void saveToStream(std::ostream &out) const = 0;
    virtual void loadFromStream(std::istream &in) = 0;
};

class SsInnerNode : public SsNode {
private:
    std::vector<Point> getEntriesCentroids() const override;
    void sortEntriesByCoordinate(size_t coordinateIndex) override;

public:
    std::pair<SsNode*, SsNode*> split() override;
    std::vector<SsNode*> children;

    SsNode* findClosestChild(const Point& target) const;
    bool isLeaf() const override { return false; }
    void updateBoundingEnvelope() override;

    std::pair<SsNode*, SsNode*> insert(const Point& point) override;

    void FNDFTrav(const Point &query, size_t k, std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> &res, NType& maxrad, std::vector<std::string>& paths) const override{
        return;
    };

    virtual void saveToStream(std::ostream &out) const override;
    virtual void loadFromStream(std::istream &in) override;
};

class SsLeaf : public SsNode {
private:
    std::vector<std::string> paths;

    std::vector<Point> getEntriesCentroids() const override;
    void sortEntriesByCoordinate(size_t coordinateIndex) override;

public:
    std::pair<SsNode*, SsNode*> split() override;
    std::vector<Point> points;

    bool isLeaf() const override { return true; }
    void updateBoundingEnvelope() override;

    std::pair<SsNode*, SsNode*> insert(const Point& point) override;

    void FNDFTrav(const Point &query, size_t k, std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> &res, NType& maxrad, std::vector<std::string>& paths) const override {
        return;
    };

    virtual void saveToStream(std::ostream &out) const override;
    virtual void loadFromStream(std::istream &in) override;
};

class SsTree {
private:
    SsNode* search(SsNode* node, const Point& target);
    SsNode* searchParentLeaf(SsNode* node, const Point& target);
    size_t k;

public:
    SsNode* root;

    SsTree(size_t k) : root(nullptr), k(k) {}
    ~SsTree() {
        delete root;
    }
    
    void insert(const Point& point);
    void insert(const Point& point, const std::string& path);
    void build (const std::vector<Point>& points);
    std::vector<std::string> kNNQuery(const Point &center, size_t k);

    void print() const;
    void test() const;

    void saveToFile(const std::string &filename) const;
    void loadFromFile(const std::string &filename);
};

#endif // !SSTREE_H