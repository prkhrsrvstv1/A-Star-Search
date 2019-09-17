#include <bits/stdc++.h>
using namespace std;

/******************************** Declarations ********************************/

class Node {
 private:
 public:
  Node *parent;
  int g_score, h_score, f_score, z_r, z_c;
  char state[3][3];
  Node(char[3][3], int);
  vector<Node *> get_neighbors();
  void print();
};

inline string state_to_string(char[3][3]);
bool goal_test(Node *);
int manhattan_heuristic(char[3][3]);
int misplaced_heuristic(char[3][3]);
vector<Node *> build_solution(Node *);
vector<Node *> a_star(char[3][3]);

struct compare_nodes {
  bool operator()(Node *, Node *);
};

class Frontier {
 public:
  set<Node *, compare_nodes> hashtable;
  unordered_set<string> hashlist;

 public:
  void insert(Node *);
  Node *pop();
  bool contains(Node *);
  bool empty();
};

class Explored {
 private:
  unordered_set<string> hashlist;

 public:
  void insert(Node *);
  bool contains(Node *);
};
/******************************************************************************/

/********************************* Definitions ********************************/

inline string state_to_string(char state[3][3]) {
  return string(state[0]) + string(state[1]) + string(state[2]);
}

bool goal_test(Node *node) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (node->state[i][j] - '0' != 3 * i + j) return false;
  return true;
};

int manhattan_heuristic(char state[3][3]) {
  int score = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      int tile = state[i][j] - '0';
      score += abs(i - tile / 3) + abs(j - tile % 3);
    }
  return score;
}

int misplaced_heuristic(char state[3][3]) {
  int score = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (state[i][j] - '0' != 3 * i + j) ++score;
  return score;
}

vector<Node *> build_solution(Node *node) {
  stack<Node *> temp_stack;
  while (node) {
    temp_stack.push(node);
    node = node->parent;
  }
  vector<Node *> path;
  while (!temp_stack.empty()) {
    path.push_back(temp_stack.top());
    temp_stack.pop();
  }
  return path;
}

bool compare_nodes::operator()(Node *n1, Node *n2) {
  if (n1->f_score == n2->f_score) return n1 > n2;
  return n1->f_score < n2->f_score;
}

Node::Node(char state[3][3], int g_score) {
  this->parent = nullptr;
  this->g_score = g_score;
  this->h_score = manhattan_heuristic(state);
  this->f_score = this->g_score + this->h_score;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      this->state[i][j] = state[i][j];
      if (state[i][j] == '0') this->z_r = i, this->z_c = j;
    }
}

vector<Node *> Node::get_neighbors() {
  vector<Node *> neighbors;
  if (this->z_r != 0) {
    Node *temp = new Node(this->state, this->g_score + 1);
    temp->state[this->z_r][z_c] = temp->state[this->z_r - 1][this->z_c];
    temp->state[this->z_r - 1][this->z_c] = '0';
    temp->z_r -= 1;
    temp->h_score = manhattan_heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_c != 0) {
    Node *temp = new Node(this->state, this->g_score + 1);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r][temp->z_c - 1];
    temp->state[temp->z_r][temp->z_c - 1] = '0';
    temp->z_c -= 1;
    temp->h_score = manhattan_heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_c != 2) {
    Node *temp = new Node(this->state, this->g_score + 1);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r][temp->z_c + 1];
    temp->state[temp->z_r][temp->z_c + 1] = '0';
    temp->z_c += 1;
    temp->h_score = manhattan_heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_r != 2) {
    Node *temp = new Node(this->state, this->g_score + 1);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r + 1][temp->z_c];
    temp->state[temp->z_r + 1][temp->z_c] = '0';
    temp->z_r += 1;
    temp->h_score = manhattan_heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  return neighbors;
}

void Node::print() {
  cout << "\n";
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) cout << this->state[i][j] << " ";
    cout << "\n";
  }
}

void Frontier::insert(Node *node) {
  this->hashlist.insert(state_to_string(node->state));
  this->hashtable.insert(node);
}

Node *Frontier::pop() {
  Node *popped = *(this->hashtable.begin());
  this->hashtable.erase(popped);
  this->hashlist.erase(state_to_string(popped->state));
  return popped;
}

bool Frontier::contains(Node *node) {
  return this->hashlist.find(state_to_string(node->state)) !=
         this->hashlist.end();
}

bool Frontier::empty() { return this->hashlist.empty(); }

void Explored::insert(Node *node) {
  this->hashlist.insert(state_to_string(node->state));
}

bool Explored::contains(Node *node) {
  return this->hashlist.find(state_to_string(node->state)) !=
         this->hashlist.end();
}

vector<Node *> a_star(char initial_state[3][3]) {
  Node *node = new Node(initial_state, 0);
  Frontier *frontier = new Frontier();
  frontier->insert(node);
  Explored *explored = new Explored();
  while (!frontier->empty()) {
    node = frontier->pop();
    if (goal_test(node)) return build_solution(node);
    explored->insert(node);
    for (Node *neighbor : node->get_neighbors()) {
      if (!explored->contains(neighbor) && !frontier->contains(neighbor)) {
        neighbor->parent = node;
        neighbor->f_score = node->g_score + 1 + neighbor->h_score;
        frontier->insert(neighbor);
      } else if (frontier->contains(neighbor) &&
                 neighbor->f_score > node->g_score + 1 + neighbor->h_score) {
        neighbor->f_score = node->g_score + 1 + neighbor->h_score;
        neighbor->parent = node;
      }
    }
  }
  return vector<Node *>();
}

/***************************************************************

void print_frontier(Frontier *frontier) {
  cout << "Frontier begins\n";
  for (Node *node : frontier->hashtable) node->print();
  cout << "Frontier ends\n";
}

/***************************************************************/

/******************************************************************************/

/********************************** Execution *********************************/
int main() {
  /**************************** Testing a_star ********************************/
  char initial_state[3][3] = {
      {'2', '8', '1'}, {'3', '4', '0'}, {'5', '6', '7'}};
  vector<Node *> solution = a_star(initial_state);
  if (solution.empty())
    cout << "Unsolvable starting state.\n";
  else {
    cout << "Found solution with " << solution.size() - 1 << " steps.\n";
    for (Node *node : solution) node->print();
  }
  /****************************************************************************/

  return 0;
}