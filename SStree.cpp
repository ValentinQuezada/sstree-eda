#include "SStree.h"

SsNode* SsTree::search(SsNode* node, const Point& target){
    if(node->isLeaf()){
        auto lnode = dynamic_cast<SsLeaf*>(node);
        for(auto &p : lnode->points){
            if(p == target) return node;
        }
    } else {
        auto inode = dynamic_cast<SsInnerNode*>(node);
        for(auto &c: inode->children){
            if(c->intersectsPoint(target)){
                auto result = search(c,target);
                if(result != nullptr) return result;
            }
        }
    } return nullptr;
}

SsNode* SsTree::searchParentLeaf(SsNode* node, const Point& target){
    if(node->isLeaf()) return node;
    else {
        auto inode = dynamic_cast<SsInnerNode*>(node);
        auto child = inode->findClosestChild(target);
        return searchParentLeaf(child,target);
    }
}

std::pair<SsNode*, SsNode*> SsLeaf::insert(const Point& point){
    if (find(this->points.begin(),this->points.end(),point) != this->points.end())
        return std::make_pair(nullptr,nullptr);
    this->points.push_back(point);
    this->updateBoundingEnvelope();
    if(this->points.size() <= Settings::M)
        return std::make_pair(nullptr,nullptr);
    return this->split();
}

std::pair<SsNode*, SsNode*> SsInnerNode::insert(const Point& point){
    auto closestChild = this->findClosestChild(point);
    auto childrenPair = closestChild->insert(point);
    SsNode* newChild1 = childrenPair.first;
    SsNode* newChild2 = childrenPair.second;
    if(newChild1 == nullptr){
        this->updateBoundingEnvelope();
        return std::make_pair(nullptr,nullptr);
    } else {
        this->children.erase(find(this->children.begin(),this->children.end(),closestChild));
        this->children.push_back(newChild1);
        this->children.push_back(newChild2);
        this->updateBoundingEnvelope();
        if(this->children.size() <= Settings::M){
            return std::make_pair(nullptr,nullptr);
        }
    }
    return this->split();
}

void SsTree::insert(const Point& point){
    if(this->root == nullptr){
        auto lnode = new SsLeaf();
        lnode->points.push_back(point);
        lnode->centroid = point;
        this->root = lnode;
        return;
    }
    auto newChildren = this->root->insert(point);
    SsNode *newChild1 = newChildren.first;
    SsNode *newChild2 = newChildren.second;
    if (newChild1 != nullptr) {
        auto inode = new SsInnerNode();
        inode->children.push_back(newChild1);
        inode->children.push_back(newChild2);
        inode->updateBoundingEnvelope();
        this->root = inode;
    }
}

void SsTree::insert(const Point& point, const std::string& path){
    this->insert(point);
}

/*
void SsLeaf::FNDFTrav(const Point &query, size_t k, std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> &res, NType& maxrad, std::vector<std::string>& paths) const {
    for(size_t i = 0; i < this->points.size(); ++i){
        auto dist = distance(query, points[i]);
        if(dist <= maxrad) {
            res.emplace(dist, this->paths[i]);
            if (res.size() > k) {
                res.pop();
                maxrad = res.top().first;
            }
        }
    }
    while (!res.empty()) {
        paths.push_back(res.top().second);
        res.pop();
    }
}

void SsInnerNode::FNDFTrav(const Point &query, size_t k, std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> &res, NType& maxrad, std::vector<std::string>& paths) const {
    for (auto &c : this->children) {
        auto dist = distance(query, c->centroid);
        if (dist <= maxrad + c->radius) {
            c->FNDFTrav(query, k, res, maxrad, paths);
        }
    }
    while (!res.empty()) {
        paths.push_back(res.top().second);
        res.pop();
    }
}

std::vector<std::string> SsTree::kNNQuery(const Point &center, size_t k)  {
    std::vector<std::string> res;
    std::priority_queue<std::pair<NType, std::string>, std::vector<std::pair<NType, std::string>>, CompareSafe> knnres;
    NType maxrad = this->root->radius;
    root->FNDFTrav(center, k, knnres, maxrad, res);
    return res;
}
*/

SsNode* SsInnerNode::findClosestChild(const Point& target) const {
    NType minDistance = NType::max_value();
    SsNode* result = nullptr;
    for (auto &c: this->children){
        auto dist = distance(c->centroid,target);
        if(dist < minDistance) {
            minDistance = dist;
            result = c;
        }
    }
    return result;
}

size_t SsNode::directionOfMaxVariance() const {
    NType maxVariance = 0;
    size_t directionIndex = 0;
    auto centroids = this->getEntriesCentroids();
    for(auto i=0; i<centroids[0].dim(); i++){
        if(varianceAlongDirection(centroids,i) > maxVariance){
            maxVariance = varianceAlongDirection(centroids,i);
            directionIndex = i;
        }
    }
    return directionIndex;
}

NType SsNode::varianceAlongDirection(const std::vector<Point> &centroids, size_t direction) const {
    NType mean = 0;
    for(auto &p: centroids){
        mean += p[direction];
    }
    mean /= centroids.size();
    NType var = 0;
    for(auto &p: centroids){
        var += (p[direction]-mean)*(p[direction]-mean);
    }
    var /= centroids.size();
    return var;
}

void SsLeaf::updateBoundingEnvelope() {
    auto centroids = this->getEntriesCentroids();
    this->centroid = centroids[0];
    for(auto i=0; i < centroids[0].dim(); i++){
        NType mean = 0;
        for(auto &p: centroids){
            mean += p[i];
        }
        mean /= centroids.size();
        this->centroid[i] = mean;
    }
    NType maxrad = 0;
    for(auto &p: centroids){
        maxrad = max(distance(this->centroid,p),maxrad);
    }
    this->radius = maxrad;
}

void SsInnerNode::updateBoundingEnvelope() {
    auto centroids = this->getEntriesCentroids();
    this->centroid = centroids[0];
    for(auto i=0; i < centroids[0].dim(); i++){
        NType mean = 0;
        for(auto &p: centroids){
            mean += p[i];
        }
        mean /= centroids.size();
        this->centroid[i] = mean;
    }
    NType maxrad = 0;
    for(auto &c: this->children){
        maxrad = max(distance(this->centroid,c->centroid)+c->radius,maxrad);
    }
    this->radius = maxrad;
}

std::pair<SsNode*,SsNode*> SsLeaf::split() {
    size_t splitIndex = this->findSplitIndex();
    auto newNode1 = new SsLeaf();
    auto newNode2 = new SsLeaf();
    for(auto i=0; i<this->points.size(); i++){
        if(i < splitIndex) newNode1->points.push_back(points[i]);
        else newNode2->points.push_back(points[i]);
    }
    newNode1->parent = this;
    newNode2->parent = this;
    newNode1->updateBoundingEnvelope();
    newNode2->updateBoundingEnvelope();
    return std::make_pair(newNode1,newNode2);
}

std::pair<SsNode*,SsNode*> SsInnerNode::split() {
    size_t splitIndex = this->findSplitIndex();
    auto newNode1 = new SsInnerNode();
    auto newNode2 = new SsInnerNode();
    for(auto i=0; i<this->children.size(); i++){
        if(i < splitIndex) newNode1->children.push_back(children[i]);
        else newNode2->children.push_back(children[i]);
    }
    newNode1->parent = this;
    newNode2->parent = this;
    newNode1->updateBoundingEnvelope();
    newNode2->updateBoundingEnvelope();
    return std::make_pair(newNode1,newNode2);
}

size_t SsNode::findSplitIndex() {
    size_t coordinateIndex = this->directionOfMaxVariance();
    this->sortEntriesByCoordinate(coordinateIndex);
    return minVarianceSplit(coordinateIndex);
}

std::vector<Point> SsLeaf::getEntriesCentroids() const {
    return this->points;
}

std::vector<Point> SsInnerNode::getEntriesCentroids() const {
    std::vector<Point> res;
    for(auto &c: this->children){
        res.push_back(c->centroid);
    }
    return res;
}

void SsLeaf::sortEntriesByCoordinate(size_t coordinateIndex) {
    std::sort(this->points.begin(), this->points.end(),
              [coordinateIndex](const Point &a, const Point &b) {
                  return a[coordinateIndex] < b[coordinateIndex];
              });
}

void SsInnerNode::sortEntriesByCoordinate(size_t coordinateIndex) {
    std::sort(this->children.begin(), this->children.end(),
              [coordinateIndex](const SsNode* a, const SsNode* b) {
                  return a->centroid[coordinateIndex] < b->centroid[coordinateIndex];
              });
}

size_t SsNode::minVarianceSplit(size_t coordinateIndex) {
    auto centroids = this->getEntriesCentroids();
    NType minVariance = NType::max_value();
    size_t splitIndex = Settings::m;
    for(auto i=Settings::m; i<=centroids.size()-Settings::m; i++){
        std::vector<Point> centroids1, centroids2;
        for(auto j=0; j<centroids.size(); j++){
            if(j < i) centroids1.push_back(centroids[j]);
            else centroids2.push_back(centroids[j]);
        }
        NType variance1 = this->varianceAlongDirection(centroids1,coordinateIndex);
        NType variance2 = this->varianceAlongDirection(centroids2,coordinateIndex);
        if(variance1+variance2 < minVariance){
            minVariance = variance1 + variance2;
            splitIndex = i;
        }
    }
    return splitIndex;
}

bool SsNode::test(bool isRoot) const {
    size_t count = 0;
    if (this->isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        count = leaf->points.size();

        // Verificar si los puntos están dentro del radio del nodo
        for (const Point& point : leaf->points) {
            if (distance(this->centroid, point) > this->radius) {
                std::cout << "Point outside node radius detected." << std::endl;
                return false;
            }
        }
    } else {
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        count = inner->children.size();

        // Verificar si los centroides de los hijos están dentro del radio del nodo padre
        for (const SsNode* child : inner->children) {
            if (distance(this->centroid, child->centroid) > this->radius) {
                std::cout << "Child centroid outside parent radius detected." << std::endl;
                return false;
            }
            // Verificar recursivamente cada hijo
            if (!child->test()) {
                return false;
            }
        }
    }

    // Comprobar la validez de la cantidad de hijos/puntos
    if (!isRoot && (count < Settings::m || count > Settings::M)) {
        std::cout << "Invalid number of children/points detected." << std::endl;
        return false;
    }

    // Comprobar punteros de parentezco, salvo para el nodo raíz
    if (!isRoot && !parent) {
        std::cout << "Node without parent detected." << std::endl;
        return false;
    }

    return true;
}

void SsTree::test() const {
    bool result = root->test();

    if (root->parent) {
        std::cout << "Root node parent pointer is not null!" << std::endl;
        result = false;
    }

    if (result) {
        std::cout << "SS-Tree is valid!" << std::endl;
    } else {
        std::cout << "SS-Tree has issues!" << std::endl;
    }
}


void SsNode::print(size_t indent) const {
    for (size_t i = 0; i < indent; ++i) {
        std::cout << "  ";
    }

    // Imprime información del nodo.
    std::cout << "Centroid: " << centroid << ", Radius: " << radius;
    if (isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        std::cout << ", Points: [ ";
        for (const Point& p : leaf->points) {
            std::cout << p << " ";
        }
        std::cout << "]";
    } else {
        std::cout << std::endl;
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        for (const SsNode* child : inner->children) {
            child->print(indent + 1); 
        }
    }
    std::cout << std::endl;
}
void SsTree::print() const {
    if (root) {
        root->print();
    } else {
        std::cout << "Empty tree." << std::endl;
    }
}







void SsLeaf::saveToStream(std::ostream &out) const {
    bool isLeaf = true;
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar centroid y radius
    out.write(reinterpret_cast<const char*>(&centroid), sizeof(centroid));
    out.write(reinterpret_cast<const char*>(&radius), sizeof(radius));

    // Guardar los puntos
    size_t numPoints = points.size();
    out.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));
    for (const auto& point : points) {
        out.write(reinterpret_cast<const char*>(&point), sizeof(point));
    }

    // Guardar las rutas (paths)
    size_t numPaths = paths.size();
    out.write(reinterpret_cast<const char*>(&numPaths), sizeof(numPaths));
    for (const auto& p : paths) {
        size_t pathLength = p.size();
        out.write(reinterpret_cast<const char*>(&pathLength), sizeof(pathLength));
        out.write(p.c_str(), pathLength);
    }
}

void SsLeaf::loadFromStream(std::istream &in) {
    // Leer centroid y radius
    in.read(reinterpret_cast<char*>(&centroid), sizeof(centroid));
    in.read(reinterpret_cast<char*>(&radius), sizeof(radius));

    // Leer puntos
    size_t numPoints;
    in.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));
    points.resize(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        in.read(reinterpret_cast<char*>(&points[i]), sizeof(points[i]));
    }

    // Leer rutas (paths)
    size_t numPaths;
    in.read(reinterpret_cast<char*>(&numPaths), sizeof(numPaths));
    paths.resize(numPaths);
    for (size_t i = 0; i < numPaths; ++i) {
        size_t pathLength;
        in.read(reinterpret_cast<char*>(&pathLength), sizeof(pathLength));
        char* buffer = new char[pathLength + 1];
        in.read(buffer, pathLength);
        buffer[pathLength] = '\0';
        paths[i] = std::string(buffer);
        delete[] buffer;
    }
}















void SsInnerNode::saveToStream(std::ostream &out) const {
    bool isLeaf = false;
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar centroid y radius
    out.write(reinterpret_cast<const char*>(&centroid), sizeof(centroid));
    out.write(reinterpret_cast<const char*>(&radius), sizeof(radius));

    // Guardar la cantidad de hijos para saber cuántos nodos leer después
    size_t numChildren = children.size();
    out.write(reinterpret_cast<const char*>(&numChildren), sizeof(numChildren));

    // Guardar los hijos
    for (const auto& child : children) {
        child->saveToStream(out);
    }
}

void SsInnerNode::loadFromStream(std::istream &in) {
    // Leer centroid y radius
    in.read(reinterpret_cast<char*>(&centroid), sizeof(centroid));
    in.read(reinterpret_cast<char*>(&radius), sizeof(radius));

    // Leer cantidad de hijos
    size_t numChildren;
    in.read(reinterpret_cast<char*>(&numChildren), sizeof(numChildren));

    // Leer hijos
    for (size_t i = 0; i < numChildren; ++i) {
        bool childIsLeaf;
        in.read(reinterpret_cast<char*>(&childIsLeaf), sizeof(childIsLeaf));
        
        SsNode* child = childIsLeaf ? static_cast<SsNode*>(new SsLeaf()) : static_cast<SsNode*>(new SsInnerNode());
        child->loadFromStream(in);
        children.push_back(child);
    }
}








void SsTree::saveToFile(const std::string &filename) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Cannot open file for writing");
    }
    root->saveToStream(out);
    out.close();
}

void SsTree::loadFromFile(const std::string &filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open file for reading");
    }
    if (root) {
        delete root;
        root = nullptr;
    }
    // Aquí se asume que el primer byte determina si es un nodo interno o una hoja
    bool isLeaf;
    in.read(reinterpret_cast<char*>(&isLeaf), sizeof(isLeaf));
    if (isLeaf) {
        root = new SsLeaf();
    } else {
        root = new SsInnerNode();
    }
    root->loadFromStream(in);
    in.close();
}


