# include "iostream"
# include "graphics.h"
# include "math.h"
# include "vector"
# include "limits"
# include "string"
# include "fstream"

using namespace std;


struct Point{
    double x;
    double y;
};


double distance(Point p1, Point p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}


// Screen
class Screen {
    private:
        int width;  // width of screen
        int height; // height of screen
    public:
        Screen(int inputWidth, int inputHeight);
        int getWidth() ;
        int getHeight();
        void initGraph();
        void clear();
        void drawLine(Point point1, Point point2);
        void drawCircle(Point point, int radius);
        void drawLineBetween2Circles(Point center1, Point center2, double radius);

};


Screen::Screen(int inputWidth, int inputHeight) {    
            /* Setup Screen  */
    width = inputWidth;
    height = inputHeight;
    }
int Screen::getWidth() {
    return width;
}
int Screen::getHeight() {
    return height;
}
void Screen::initGraph() {
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);
    // init graph (build GUI)
}
void Screen::clear() {
    // clear screen
    cleardevice();
}
void Screen::drawLine(Point point1, Point point2) {
    // draw line
   line(point1.x, point1.y, point2.x, point2.y);
}
void Screen::drawCircle(Point point, int radius) {
    // draw circle
    circle(point.x, point.y, radius);
}
void Screen::drawLineBetween2Circles(Point center1, Point center2, double radius) {
    // draw line between 2 circles
    
    Point point1, point2;
    // calculation
    if(center1.x  == center2.x){
        point1.x = center1.x ;
        point2.x =center1.x ;

        if (center1.y > center2.y)
        {
            point1.y = center1.y - radius;
            point2.y = center2.y + radius;

        }
        if (center2.y > center1.y)
        {
            point1.y = center1.y + radius;
            point2.y = center2.y - radius;
        }
        

    } else {
        float A, B, C, delta, a, b;
        
        a = (center2.y - center1.y) / (center2.x - center1.x);
        b = center1.y - a * center1.x;
        A = a * a + 1;

        // find probabilies for x point 1
        int x11, x12;
        
        
        B = 2*((a*b) - (a * center1.y) - center1.x );
        C = ((b * b) + (center1.y * center1.y)  - (2 * b * center1.y) + (center1.x * center1.x)) - (radius * radius);
        delta = B * B - 4 * A * C;
        
        x11 = (sqrt(delta) - B) / (2 * A);
        x12 = (-sqrt(delta) - B) / (2 * A);

        // find probabilies for x point 2
        
        int x21, x22;
        
        B = 2*((a*b) - (a * center2.y) - center2.x );
        C = ((b * b) + (center2.y * center2.y)  - (2 * b * center2.y) + (center2.x * center2.x)) - (radius * radius);

        delta = B * B - 4 * A * C;
        x21 = (sqrt(delta) - B) / (2 * A);
        x22 = (-sqrt(delta) - B) / (2 * A);


        /// find points
        if (center1.x > center2.x)

        {
            point1.x = x12;
            point1.y = a * x12 + b;

            point2.x = x21;
            point2.y = a * x21 + b;

        } else {
            point1.x = x11;
            point1.y = a * x11 + b;

            point2.x = x22;
            point2.y = a * x22 + b;
        }


    }
    //

    // draw line
    
    this->drawLine(point1, point2);
}



//


// Node

class Node {

    private:
        int id;
        Point point;
        Point position;

    public:

        static int radius;
        static int thickness;
        Node(int inputId);
        int getId();
        void setPoint(Point inputPoint);
        Point getPoint();
        void setPosition(Point inputPosition);
        Point getPosition();
        void visualize(Screen* screen);

};


int Node::radius = 9;
int Node::thickness = Node::radius / 10 + 1;

Node::Node(int inputId)
{
	id=inputId;
}
int Node::getId()
{
    return id;
}
void Node::setPoint(Point inputPoint)
{
    point = inputPoint;
}
Point Node::getPoint()
{
    return point;
}
void Node::setPosition(Point inputPosition)
{
    position = inputPosition;
}
Point Node::getPosition()
{
    return position;
}
void Node::visualize(Screen* screen)
{
    // draw node
    screen->drawCircle(position, Node::radius);
}

// 

class Edge {
    private:
        int id;
        Node* firstNode;
        Node* secondNode;
        double weight;
        Screen* screen;
    public:
        static int thickness;
        Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight = 1);
        void setWeight(double inputWeight);
        int getId();
        double getWeight();
        Node* getFirstNode();
        Node* getSecondNode();
        void visualize(Screen* screen);

};

int Edge::thickness = Node::thickness;


Edge::Edge(int inputId, Node* inputNode1, Node* inputNode2, double inputWeight)
{
	id = inputId;
	firstNode = inputNode1;
	secondNode = inputNode2;
	weight = inputWeight;
}       
void Edge::setWeight(double inputWeight)
{
    weight = inputWeight;
}
double Edge::getWeight()
{
	return weight;
}
int Edge::getId()
{
    return id;
}
Node* Edge::getFirstNode()
{
	return firstNode;
}
Node* Edge::getSecondNode()
{
	return secondNode;
}
void Edge::visualize(Screen* screen){
    // draw edge
    Point point1 = firstNode->getPosition();
    Point point2 = secondNode->getPosition();
    screen->drawLineBetween2Circles(point1, point2, Node::radius);
}





class Graph {
    vector<Edge*> edges;
    vector<Node*> nodes;
    double** adj;
    int edgesCount;
    int nodesCount;
    Screen* screen;
public:
    Graph();
    void setScreen(Screen* screen);
    void addEdge(Edge* inputEdge);
    void addNode(Node* inputNode);
    void removeEdge(Edge* inputEdge);
    void removeNode(Node* inputNode);
    void removeEdgeById(int id);
    void removeNodeById(int id);
    vector<Edge*> getEdges();
    vector<Node*> getNodes();
    int getNodesCount();
    int getEdgesCount();
    Point* getPoints();
    Point* getPositions();
    void makeAddjacencyMatrix();
    double** getAdjacencyMatrix();
    void scale();
    void visualizeNodes();
    void visualizeEdges();
    void visualize();

};




Graph::Graph()
{
    edgesCount = 0;
    nodesCount = 0;
}

void Graph::addEdge(Edge* inputEdge)
{
    edges.push_back(inputEdge);
    edgesCount ++;
    
}
void Graph::addNode(Node* inputNode)
{
    nodes.push_back(inputNode);
    nodesCount ++;
 
}
void Graph::removeEdge(Edge* inputEdge)
{
    for (int i = 0; i < edges.size(); i++)
    {
        if (edges[i] == inputEdge)
        {
            edges.erase(edges.begin() + i);
            edgesCount --;
            break;
        }
    }
}
void Graph::removeNode(Node* inputNode)
{
    for (int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i] == inputNode)
        {
            nodes.erase(nodes.begin() + i);
            nodesCount --;
            break;
        }
    }
}
void Graph::removeEdgeById(int id)
{
    for (int i = 0; i < edges.size(); i++)
    {
        if (edges[i]->getId() == id)
        {
            edges.erase(edges.begin() + i);
            edgesCount --;
            break;
        }
    }
}
void Graph::removeNodeById(int id)
{
    for (int i = 0; i < nodes.size(); i++)
    {
        if (nodes[i]->getId() == id)
        {
            nodes.erase(nodes.begin() + i);
            nodesCount --;
            break;
        }
    }
}
vector<Edge*> Graph::getEdges(){
    return edges;
}
vector<Node*> Graph::getNodes(){
    return nodes;
}
int Graph::getNodesCount(){
    return nodesCount;
}
int Graph::getEdgesCount(){
    return edgesCount;
}
Point* Graph::getPoints(){
    Point* points = new Point[nodesCount];
    for (int i = 0; i < nodesCount; i++)
    {
        points[i] = nodes[i]->getPoint();
    }
}
Point* Graph::getPositions(){
    Point* positions = new Point[nodesCount];
    for (int i = 0; i < nodesCount; i++)
    {
        positions[i] = nodes[i]->getPosition();
    }
}
void Graph::makeAddjacencyMatrix(){
    adj = new double*[nodesCount];
    for (int i = 0; i < nodesCount; i++)
    {
        adj[i] = new double[nodesCount];
    }
    for (int i = 0; i < nodesCount; i++)
    {
        for (int j = 0; j < nodesCount; j++)
        {
            adj[i][j] = numeric_limits<double>::max();
        }
    }
    for (int i = 0; i < edgesCount; i++)
    {
        Edge* edge = edges[i];
        Node* firstNode = edge->getFirstNode();
        Node* secondNode = edge->getSecondNode();
        int firstNodeId = firstNode->getId();
        int secondNodeId = secondNode->getId();
        adj[firstNodeId][secondNodeId] = edge->getWeight();
        
    }
}
double** Graph::getAdjacencyMatrix(){
    return adj;
}
void Graph::scale(){
    double maxX = 0;
    double maxY = 0;
    for (int i = 0; i < nodesCount; i++)
    {
        
        Node* node = nodes[i];
        Point point = node->getPosition();
        if (point.x > maxX)
        {
            maxX = point.x;
        }
        if (point.y > maxY)
        {
            maxY = point.y;
        }
    }
    double scaleX = (double)screen->getWidth() / maxX;
    double scaleY = (double)screen->getWidth() / maxY;
    for (int i = 0; i < nodesCount; i++)
    {
        Node* node = nodes[i];
        Point point = node->getPosition();
        point.x = point.x * scaleX;// + 30;
        point.y = point.y * scaleY;// + 30;
        node->setPosition(point);
    }
}
void Graph::visualizeNodes(){
    for (int i = 0; i < nodesCount; i++)
    {
        Node* node = nodes[i];
        node->visualize(screen);
    }
}
void Graph::visualizeEdges(){
    for (int i = 0; i < edgesCount; i++)
    {
        Edge* edge = edges[i];
        edge->visualize(screen);
    }
}
void Graph::visualize(){
    visualizeNodes();
    visualizeEdges();
}
void Graph::setScreen(Screen* screen){
    this->screen = screen;
}



class Problem {
    Point* points;
    Point* positions;
    int pointsCount;
    Graph* graph;

    public:
    Problem(Point* inputPoints, int inputPointsCount);
    Problem(string fileName);
    void setGraph();
    Graph* getGraph();
    int getPointsCount();
    Point* getPoints();
    void scale(Screen* screen);
    Point* getPositions();

};

Problem::Problem(Point* inputPoints, int inputPointsCount)
{
    points = inputPoints;
    pointsCount = inputPointsCount;
    positions = new Point[pointsCount];
    for (int i = 0; i < pointsCount; i++)
    {
        positions[i] = points[i];
    }
    
}
Problem::Problem(string fileName)
{
    ifstream file;
    file.open(fileName);
    if (!file.is_open())
    {
        cout << "File not found!" << endl;
        return;
    }
    file >> pointsCount;
    points = new Point[pointsCount];
    for (int i = 0; i < pointsCount; i++)
    {
        double x, y;
        file >> x >> y;
        Point point = Point{x, y};
        points[i] = point;
    }
    pointsCount = pointsCount;
    positions = new Point[pointsCount];
    for (int i = 0; i < pointsCount; i++)
    {
        positions[i] = points[i];
    }
}
void Problem::setGraph()
{
    graph = new Graph();
    for (int i = 0; i < pointsCount; i++)
    {
        Node* node = new Node(i);
        node->setPoint(points[i]);
        node->setPosition(points[i]);
        graph->addNode(node);

    }
    int edgeId = 0;
    vector<Node*> nodes = graph->getNodes();
    for (int i = 0; i < pointsCount; i++)
    {
        for (int j = i + 1; j < pointsCount; j++)
        {
            double d = distance(points[i], points[j]);
            Edge* edge = new Edge(edgeId, nodes[i], nodes[j], d);
            graph->addEdge(edge);
            edgeId ++;

            edge = new Edge(edgeId, nodes[j], nodes[i], d);
            graph->addEdge(edge);
            edgeId ++;
        }
    }
}
Graph* Problem::getGraph()
{
    return graph;
}
int Problem::getPointsCount()
{
    return pointsCount;
}
Point* Problem::getPoints()
{
    return points;
}
void Problem::scale(Screen* screen){
    Graph* newGraph = new Graph();
    for (int i = 0; i < pointsCount; i++)
    {
        Node* node = new Node(i);
        node->setPoint(points[i]);
        node->setPosition(positions[i]);
        newGraph->addNode(node);
    }
    newGraph->setScreen(screen);
    newGraph->scale();
    for (int i = 0; i < pointsCount; i++)
    {
        positions[i] = newGraph->getNodes()[i]->getPosition();
    }
}
Point* Problem::getPositions(){
    return positions;
}



class Path {
    int length;
    Point* points;
    Point* positions;
    
    int* order;
    Graph* graph;
    
    bool hasCosts;
    int** costs;

    

    public:
    Path(int inputLength, Point* inputPoints, Point* inputPositions);
    void setCosts(int** inputCosts);
    bool ifHasCosts();
    int getLength();
    Point* getPoints();
    Point* getPositions();
    void setOrder(int* inputOrder);
    int* getOrder();
    Point* getPointsByOrder();
    Point* getPositionsByOrder();
    double getCost();
    void setGraph();
    Graph* getGraph();
    void visualize(Screen* screen);
    bool isValid();
    void setRandomOrder();
    Path* copy();    

};

Path::Path(int inputLength, Point* inputPoints, Point* inputPositions)
{
    length = inputLength;
    points = inputPoints;
    positions = inputPositions;
    order = new int[length];
    for (int i = 0; i < length; i++)
    {
        order[i] = i;
    }
    hasCosts = false;
    costs = NULL;
}
void Path::setCosts(int** inputCosts)
{
    costs = inputCosts;
    hasCosts = true;
}
bool Path::ifHasCosts()
{
    return hasCosts;
}
int Path::getLength()
{
    return length;
}
Point* Path::getPoints()
{
    return points;
}
Point* Path::getPositions(){
    return positions;
}
void Path::setOrder(int* inputOrder)
{
    order = inputOrder;
}
int* Path::getOrder(){
    return order;
}
Point* Path::getPointsByOrder()
{
    Point* pointsByOrder = new Point[length];
    for (int i = 0; i < length; i++)
    {
        pointsByOrder[i] = points[order[i]];
    }
    return pointsByOrder;
}
Point* Path::getPositionsByOrder()
{
    Point* positionsByOrder = new Point[length];
    for (int i = 0; i < length; i++)
    {
        positionsByOrder[i] = positions[order[i]];
    }
    return positionsByOrder;
}
double Path::getCost()
{
    double cost = 0;
    if(hasCosts){
        for (int i = 0; i < length - 1; i++)
        {
            cost += costs[order[i]][order[i + 1]];
            
        }
        cost += costs[order[length - 1]][order[0]];
    } else {
        for (int i = 0; i < length - 1; i++)
        {
            cost += distance(points[order[i]], points[order[i + 1]]);
        }
        cost += distance(points[order[length - 1]], points[order[0]]);
    }
    return cost;
}
void Path::setGraph(){
    graph = new Graph();
    Point* sortedPoints = this->getPointsByOrder();
    Point* sortedPositions = this->getPositionsByOrder();

    for (int i = 0; i < length; i++)
    {
        Node* node = new Node(i);
        node->setPoint(sortedPoints[i]);
        node->setPosition(sortedPositions[i]);
        graph->addNode(node);
    }
    int edgeId = 0;
    vector<Node*> nodes = graph->getNodes();
    for (int i = 0; i < length - 1; i++)
    {
        Edge* edge = new Edge(edgeId, nodes[i], nodes[i + 1], distance(nodes[i]->getPosition(), nodes[i + 1]->getPosition()));
        
        graph->addEdge(edge);
        edgeId ++;
    }
    Edge* edge = new Edge(edgeId, nodes[length - 1], nodes[0], distance(sortedPoints[length - 1], sortedPoints[0]));
    graph->addEdge(edge);
}
Graph* Path::getGraph(){
    return graph;
}
void Path::visualize(Screen* screen){
    graph->setScreen(screen);
    graph->visualize();
}
bool Path::isValid(){
    if(length < 2){
        return false;
    }
    
    bool checked[length] = {false};
    for(int i = 0; i < length; i++){
        if(order[i] < 0 || order[i] >= length){
            return false;
        }
        if(checked[order[i]]){
            return false;
        }
        checked[order[i]] = true;

        
    }
    return true;
}
void Path::setRandomOrder(){
    for(int i = 0; i < length; i++){
        order[i] = i;
    }
    for(int i = 0; i < length; i++){
        int j = rand() % length;
        int temp = order[i];
        order[i] = order[j];
        order[j] = temp;
    }
}
Path* Path::copy(){
    Path* copyPath = new Path(length, points, positions);
    copyPath->setOrder(order);
    return copyPath;
}


class Algorithm{
    Path* path;
    double cost;
    Problem* problem;
    Screen* screen;
    void random();
    void localSearch();
    void antColony(double** adj, double** phermone, int ants, float evaporation, int n);
    public:
    Algorithm(Problem* inputProblem);
    void setScreen(Screen* inputScreen);
    void doRandom(int epochs, bool visualize = 0, int delay = 100);
    void doGreedy(bool visualize = 0);
    void doLocalSearch(int epochs, bool visualize = 0, int delay = 100);
    void doExhaustive(bool visualize = 0);
    void doAntColony(int ants, int epochs, float evaporation = 0.1, bool visualize = 0, int delay = 100);
};

Algorithm::Algorithm(Problem* inputProblem) {
    problem = inputProblem;
    path = new Path(problem->getPointsCount(), problem->getPoints(), problem->getPositions());
    path->setRandomOrder();
    cout << "Hi" << endl;
    cost = path->getCost();
    cout << "Bi" << endl;

}
void Algorithm::setScreen(Screen* inputScreen) {
    screen = inputScreen;
    problem->scale(screen);
}
void Algorithm::random() {
    path->setRandomOrder();
}
void Algorithm::doRandom(int epochs, bool visualize, int sleep) {
    Path* bestPath = path->copy();
    int bestCost = cost;
    int newCost;
    
    bestPath->setGraph();
    bestPath->visualize(screen);
    delay(sleep);
    for (int i = 0; i < epochs; i++)
    {
        this->random();
        
        newCost = path->getCost();
        if(newCost < bestCost){
            bestCost = newCost;
            bestPath->setOrder(path->getOrder());

            if(visualize){
                screen->clear();
                bestPath->setGraph();
                bestPath->visualize(screen);
                delay(sleep);
            }
        }   
        cout << "Minimum cost: " << cost << "\tCurrent cost: " << newCost << endl;
    }
    path = bestPath->copy();
    cost = bestCost;
}
void Algorithm::localSearch() {
    double changesValue[problem->getPointsCount()][problem->getPointsCount()];
    double chances[problem->getPointsCount()][problem->getPointsCount()];
    double cummulative[problem->getPointsCount()][problem->getPointsCount()];
    Path* newPath = path->copy();
    int* newOrder = newPath->getOrder();
    double sum = 0;
    for (int i = 0; i < problem->getPointsCount(); i++)
    {
        for (int j = 0; j < problem->getPointsCount(); j++)
        {
            swap(newOrder[i], newOrder[j]);
            newPath->setOrder(newOrder);
            double c = 100 / newPath->getCost();
            changesValue[i][j] = c;
            swap(newOrder[i], newOrder[j]);
            newPath->setOrder(newOrder);

            sum += c;

        }
    }
    
    for (int i = 0; i < problem->getPointsCount(); i++)
    {
        for (int j = 0; j < problem->getPointsCount(); j++)
        {
            chances[i][j] = changesValue[i][j] / sum;
        }
    }

    double s = 0;
    for (int i = 0; i < problem->getPointsCount(); i++)
    {
        for (int j = 0; j < problem->getPointsCount(); j++)
        {
            cummulative[i][j] = chances[i][j] + s;
            s = cummulative[i][j];
            //cout << s << endl;
        }
    }
 
    double r = ((double) rand() / (RAND_MAX));
    int selectedI, selectedJ;
    bool isSelected = false;

    for (int i = 0; i < problem->getPointsCount(); i++)
    {
        for (int j = 0; j < problem->getPointsCount(); j++)
        {
            if (r <= cummulative[i][j])
            {
                selectedI = i;
                selectedJ = j;
                isSelected = true;
                break;
            }
        }
        if (isSelected)
        {
            break;
        }

    }
    swap(newOrder[selectedI], newOrder[selectedJ]);
    path->setOrder(newOrder);



}
void Algorithm::doLocalSearch(int epochs, bool visualize, int sleep) {
    Path* bestPath = path->copy();
    int bestCost = cost;
    int newCost;
    

    bestPath->setGraph();
    bestPath->visualize(screen);
    delay(sleep);

    for (int i = 0; i < epochs; i++)
    {
        this->localSearch();
        
        newCost = path->getCost();
        if(newCost < bestCost){
            bestCost = newCost;
            bestPath->setOrder(path->getOrder());

            if(visualize){
                screen->clear();
                bestPath->setGraph();
                bestPath->visualize(screen);
                
                delay(sleep);
            }
        }   
         cout << "Minimum cost: " << bestCost << "\tCurrent cost: " << newCost << endl;
       
    }
    path = bestPath->copy();
    cost = bestCost;
    cout << path->isValid() << endl;
}
void Algorithm::doGreedy(bool visualize) {
    int* order = new int[problem->getPointsCount()];
    int visited[problem->getPointsCount()] = {0};
    int current = 0;
    order[0] = 0;
    visited[0] = 1;
    Point* points = problem->getPoints();

    for(int i = 1; i < problem->getPointsCount(); i++){
        double minDistance = numeric_limits<int>::max();
        int minIndex = -1;
        for(int j = 0; j < problem->getPointsCount(); j++){
            if(visited[j] == 0){
                double d = distance(points[current], points[j]);
                if(d < minDistance){
                    minDistance = d;
                    minIndex = j;
                }
            }
        }
        order[i] = minIndex;
        visited[minIndex] = 1;
        current = minIndex;
    }
    path->setOrder(order);

    if(visualize){
        screen->clear();
        path->setGraph();
        path->visualize(screen);
    }


    
}




int chooseByChance(vector<double> chances, int size){
    double r = ((double) rand() / (RAND_MAX));
    int selected = -1;
    double s = 0;
    for (int i = 0; i < size; i++)
    {
        if(r <= chances[i] + s){
            selected = i;
            break;
        }
        s += chances[i];
    }
    return selected;
}

vector<double> getChancesByValues(vector<double> values, int size){
    vector<double> chances;
    double sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += values[i];
    }
    for (int i = 0; i < size; i++)
    {
        chances.push_back(values[i] / sum);
    }
    return chances;
}

int* chooseOrderByPhermone(double** phermone, int size){
    int start  = 0;
    int* path = new int[size];
    
    path[0] = start;
    bool is_visited[size] = {false};
    is_visited[start] = true;
    
    for(int tmp = 0; tmp < size - 1; tmp++){
        int selected = -1;
        double selected_meph = -1;
        for(int i = 0; i < size ; i++){
            if(is_visited[i])
                continue;
            if(selected_meph < phermone[path[tmp]][i]){
                selected = i;
                selected_meph = phermone[path[tmp]][i]; }
        } 
        path[tmp + 1] = selected;
        is_visited[selected] = true;
    }
    return path;
}

void Algorithm::antColony(double** adj, double** phermone, int ants, float evaporation, int n){
    
    int current_node[ants] =  {0};

    bool** is_visited = new bool*[ants];
    for (int i = 0; i < ants; i++)
    {
        is_visited[i] = new bool[n];
        is_visited[i][0] = true;
        for (int j = 1; j < n; j++)
        {
            is_visited[i][j] = false;
        }
    }

    

    int lenght_visited[ants] = {1};
    
    vector<vector<int>> next_edges_for_mephrone;
    vector<int> choices;
    vector<double> choices_reward;

    for(int tmp = 0; tmp < n; tmp++){
        
        
        next_edges_for_mephrone.clear();
        int lgg = 0;

        for (int ant = 0; ant < ants; ant ++) {
            if(lenght_visited[ant] == n){
                
                vector<int> next_edges;
                next_edges.push_back(current_node[ant]);
                next_edges.push_back(0);
                
                next_edges_for_mephrone.push_back(next_edges);
                lgg ++;
                
                
                current_node[ant] = 0;
                
                bool** is_visited = new bool*[ants];
                
                is_visited[ant] = new bool[n];
                is_visited[ant][0] = true;
                for (int j = 1; j < n; j++)
                {
                    is_visited[ant][j] = false;
                }

                lenght_visited[ant] = 1;
        
                continue;
            }

            
            choices.clear();
            choices_reward.clear();
            int lg = 0;

            for(int v = 0; v < n; v++){
                if(is_visited[ant][v] || v == current_node[ant])
                    continue;
                choices.push_back(v);
                choices_reward.push_back(phermone[current_node[ant]][v]);
                lg ++;
                
            }

            is_visited[ant][current_node[ant]] = true;
            lenght_visited[ant] += 1;

            double s = 0;
         
            
            

            vector<double> chances = getChancesByValues(choices_reward, lg);
            int next_v = choices[chooseByChance(chances, lg)];
            vector<int> next_edges;
            next_edges.push_back(current_node[ant]);
            next_edges.push_back(next_v);
            next_edges_for_mephrone.push_back(next_edges);
            lgg ++;

            is_visited[ant][next_v] = true;
            current_node[ant] = next_v;
            lenght_visited[ant] ++;

        }

        for(int i =0; i<lgg; i++){
            vector<int> edge = next_edges_for_mephrone[i];
            phermone[edge[0]][edge[1]] = (phermone[edge[0]][edge[1]] + 1 / adj[edge[0]][edge[1]]) * (1 - evaporation);
        }
            
    }
}



void Algorithm::doAntColony(int ants, int epochs, float evaporation, bool visualize, int sleep){
    problem->setGraph();
    Graph* g = problem->getGraph();
    g->makeAddjacencyMatrix();
    double** adj = g->getAdjacencyMatrix();
    double** phermone = new double*[g->getNodesCount()];
    for (int i = 0; i < g->getNodesCount(); i++)
    {
        phermone[i] = new double[g->getNodesCount()];
        for (int j = 0; j < g->getNodesCount(); j++)
        {
            phermone[i][j] = 1;
        }
    }
    
    for(int epoch = 0; epoch < epochs; epoch++){
        this->antColony(adj, phermone, ants, evaporation, problem->getPointsCount());
        int* order = chooseOrderByPhermone(phermone, problem->getPointsCount());
        path->setOrder(order);
        if(visualize){
            screen->clear();
            path->setGraph();
            path->visualize(screen);
            delay(sleep);
        }
        cout << path->getCost() << endl;
    }
    
}





int main() {

    Screen screen(400, 500);
    screen.initGraph();
    Problem p = Problem("cases/tsp_76_1");
    
    Algorithm algorithm = Algorithm(&p);
    algorithm.setScreen(&screen);
    algorithm.doAntColony(15, 100, 0.2, 1, 10);
    //algorithm.doLocalSearch(1000, 1, 100);
    getch();
}