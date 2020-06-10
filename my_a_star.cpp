#include <iostream>
#include <queue> //priority_queue
#include <vector>
#include <cmath>

#define EPSILON 0.001

using namespace std;

class Action {
public:
    //Assume dx, dy is either 1, 0, -1
    Action(const int dx, const int dy, const float gCost)
    : dx(dx), dy(dy), gCost(gCost) {}

    const int dx, dy; // amount of grid spaces traversed
    const float gCost;
};

class Point {
public:
    Point(const float x, const float y) 
    : x(x), y(y) {}

    const float x, y;
};

class Rectangle {
public:
    Rectangle(Point bottomLeft, Point topRight)
    : bottomLeft(bottomLeft), topRight(topRight) {}

    Point bottomLeft, topRight;
};

enum State {
    OBSTACLE = 'X', 
    IN_CLOSED = 'C', 
    IN_OPEN = 'O', 
    EMPTY = 'E',
    PATH = 'P'
};

class Node {
public:
    Node(const int x, const int y) 
    : x(x), y(y), state(EMPTY), gCost(INFINITY), hCost(INFINITY), parent(NULL) {}

    float getFCost() const {
        return gCost + hCost;
    }

    float distTo(Node* node) const {
        return sqrt(pow(x-node->x, 2) + pow(y-node->y, 2));
    }

    State state;
    const int x, y;
    float gCost, hCost;
    Node* parent;
};

class nodeGreaterThan {
public:
    bool operator () (const Node* lhs, const Node* rhs) {
        float fCostDiff = lhs->getFCost() - rhs->getFCost();
        if (fabs(fCostDiff) < EPSILON) //equal fCost
            return lhs->hCost > rhs->hCost;
        else
            return fCostDiff > 0.0;
    }
};

class Grid {
    int getXIndex(Point point, bool topRight=false) {
        int index = (point.x - rect.bottomLeft.x) / nodeSize;
        if (topRight)
            index--;
        return index;
    }

    int getYIndex(Point point, bool topRight=false) {
        int index = (point.y - rect.bottomLeft.y) / nodeSize;
        if (topRight)
            index--;
        return index;
    }

public:
    Grid(Rectangle rect, vector<Rectangle> obstacles, float nodeSize=0.05) :
    nodeSize(nodeSize), 
    rect(rect), 
    numNodesX((rect.topRight.x - rect.bottomLeft.x) / nodeSize),
    numNodesY((rect.topRight.y - rect.bottomLeft.y)  / nodeSize),
    nodes(numNodesX) {
        for (int i = 0; i < numNodesX; i++) {
            nodes[i].reserve(numNodesY);
            for (int j = 0; j < numNodesY; j++) {
                nodes[i].push_back(Node(i, j));
            }
        }
        
        for (Rectangle obstacle : obstacles) {
            int minX = getXIndex(obstacle.bottomLeft);
            int minY = getYIndex(obstacle.bottomLeft);
            int maxX = getXIndex(obstacle.topRight, true);
            int maxY = getYIndex(obstacle.topRight, true);

            for (int i = minX; i <= maxX; i++) {
                for (int j = minY; j <= maxY; j++)
                    nodes[i][j].state = OBSTACLE;
            }
        }
    }

    Node* getNode(Point pos) {
        return &nodes[getXIndex(pos)][getYIndex(pos)];
    }

    bool inBounds(int x, int y) {
        return x >= 0 && x < numNodesX && y >= 0 && y < numNodesY;
    }

    void print() {
        for (int j = numNodesY-1; j >= 0; j--) {
            for (int i = 0; i < numNodesX; i++) {
                cout << (char) nodes[i][j].state << " ";
            }
            cout << endl;
        }
        cout << endl;
    }

    const int numNodesX, numNodesY;
    const float nodeSize;
    Rectangle rect;
    vector<vector<Node>> nodes;
};

class AStar {
    vector<Action> actions;

public:
    AStar(Grid grid, vector<Action> actions)
    : grid(grid), actions(actions) {}

    void findPath(Point start, Point end) {
        Node* startNode = grid.getNode(start);
        Node* endNode = grid.getNode(end);
        startNode->state = IN_OPEN;
        startNode->gCost = 0.0;
        startNode->hCost = startNode->distTo(endNode);

        priority_queue<Node*, vector<Node*>, nodeGreaterThan> openSet;
        openSet.push(startNode);

        while (!openSet.empty()) {
            Node* currNode = openSet.top();

            if (currNode == endNode) {
                setPath(endNode);
                return;
            }
                
            openSet.pop();
            currNode->state = IN_CLOSED;

            for (Action action : actions) {
                int adjX = currNode->x + action.dx;
                int adjY = currNode->y + action.dy;

                if (!grid.inBounds(adjX, adjY))
                    continue;

                Node* adjNode = &grid.nodes[adjX][adjY];

                if (adjNode->state == OBSTACLE || adjNode->state == IN_CLOSED)
                    continue;

                float newGCost = currNode->gCost + action.gCost;
                if (newGCost < adjNode->gCost) {
                    adjNode->gCost = newGCost;
                    adjNode->parent = currNode;
                }

                if (adjNode->state == EMPTY) {
                    adjNode->hCost = adjNode->distTo(endNode);
                    adjNode->state = IN_OPEN;
                    openSet.push(adjNode);
                }
            }
        }
    }

    void setPath(Node* node) {
        while (node != NULL) {
            node->state = PATH;
            node = node->parent;
        }
    }

    Grid grid;
};

int main() {
    Rectangle rect(Point(0.0, 0.0), Point(11.0, 11.0));
    vector<Rectangle> obstacles {
        Rectangle(Point(3.0, 0.0), Point(4.0, 7.0)),
        Rectangle(Point(7.0, 4.0), Point(8.0, 11.0))
    };
    Grid grid(rect, obstacles);
    vector<Action> actions {
        Action(1, 0, 1.0),
        Action(0, 1, 1.0),
        Action(-1, 0, 1.0),
        Action(0, -1, 1.0),
        Action(-1, -1, sqrt(2)),
        Action(-1, 1, sqrt(2)),
        Action(1, -1, sqrt(2)),
        Action(1, 1, sqrt(2))
    };

    AStar aStar(grid, actions);
    aStar.findPath(Point(1.0, 1.0), Point(9.0, 9.0));
    aStar.grid.print();
}