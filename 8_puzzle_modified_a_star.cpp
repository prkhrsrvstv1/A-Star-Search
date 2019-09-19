#include <bits/stdc++.h>
#define NUM_TRIALS 10
using namespace std;

/******************************** Declarations ********************************/

typedef function<int(char[3][3])> heuristic_t;

int nodes_generated, max_fringe_length;

// This class' objects represent a node in the state graph.
class Node {
 private:
 public:
  // Link to the best possible way to get to this state.
  Node *parent;
  // To keep the g(node), h(node), f(node) stored.
  int g_score, h_score, f_score;
  // To store the location of the blank tile in this state.
  int z_r, z_c;
  // To represent the state as a 2D matrix.
  char state[3][3];

  // Parametrized constructor
  Node(char[3][3], int, heuristic_t, Node *);

  // Return a list of all possible states that are 1 action away from this
  // state.
  vector<Node *> get_neighbors(heuristic_t);

  // Return a list of level-order traversal from this state upto fixed depth.
  vector<Node *> modified_expand(int, heuristic_t);

  // Display this state as 2D matrix;
  void print();
};

// Convert 2D state to 1D string for hashing purpose.
inline string state_to_string(char[3][3]);

// Test if the given state is the Goal State.
bool goal_test(Node *);

// Returns the Manhattan Distance heuristic for the given node.
int manhattan_heuristic(char[3][3]);

// Returns the Mismatch Tiles heuristic for the given node.
int mismatch_heuristic(char[3][3]);

// Build the solution path from the given goal state using Node::parent.
vector<Node *> build_solution(Node *);

// A* path finder that uses BFS upto a given depth to expand the frontier at
// each step.
vector<Node *> modified_a_star(char[3][3], heuristic_t, int);

// Check if a given state is unsolvable by checking parity of inversion count.
bool unsolvable_state(vector<int>);

// Provides custom compare function to order the Frontier::hastable
struct compare_nodes {
  // Overrides the () operator so that compare_nodes() can be called.
  bool operator()(Node *, Node *);
};

// This class defines a custom data structure Frontier, that combines benefits
// of priority queue and hashtable.
class Frontier {
 private:
  // To facilitate priority-based popping, like priority queue.
  set<Node *, compare_nodes> hashtable;
  // To facilitate constant-time membership checking.
  unordered_set<string> hashlist;

 public:
  // Insert a new node in the frontier set.
  void insert(Node *);

  // Pop out the minimum node from the frontier set.
  Node *pop();

  // Check if given node is present in the frontier set.
  bool contains(Node *);

  // Return the present length of the frontier set.
  int length();

  // Check if the frontier set is empty.
  bool empty();
};

// This class defines a custom data structure Explored, which is wrapper around
// C++'s std::unordered_set.
class Explored {
 private:
  // To facilitate constant-time membership checking.
  unordered_set<string> hashlist;

 public:
  // Insert a new node in the explored set.
  void insert(Node *);

  // Check if given node is present in the explored set.
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

int mismatch_heuristic(char state[3][3]) {
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

bool unsolvable_state(vector<int> state) {
  int inversion_count = 0;
  for (int i = 0; i < 8; ++i)
    for (int j = i + 1; j < 9; ++j)
      if (state[i] && state[j] && state[i] > state[j]) ++inversion_count;
  return inversion_count % 2;
}

Node::Node(char state[3][3], int g_score, heuristic_t heuristic,
           Node *parent = nullptr) {
  this->parent = parent;
  this->g_score = g_score;
  this->h_score = heuristic(state);
  this->f_score = this->g_score + this->h_score;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      this->state[i][j] = state[i][j];
      if (state[i][j] == '0') this->z_r = i, this->z_c = j;
    }
}

vector<Node *> Node::get_neighbors(heuristic_t heuristic) {
  vector<Node *> neighbors;
  if (this->z_r != 0) {
    Node *temp = new Node(this->state, this->g_score + 1, heuristic, this);
    temp->state[this->z_r][z_c] = temp->state[this->z_r - 1][this->z_c];
    temp->state[this->z_r - 1][this->z_c] = '0';
    temp->z_r -= 1;
    temp->h_score = heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_c != 0) {
    Node *temp = new Node(this->state, this->g_score + 1, heuristic, this);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r][temp->z_c - 1];
    temp->state[temp->z_r][temp->z_c - 1] = '0';
    temp->z_c -= 1;
    temp->h_score = heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_c != 2) {
    Node *temp = new Node(this->state, this->g_score + 1, heuristic, this);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r][temp->z_c + 1];
    temp->state[temp->z_r][temp->z_c + 1] = '0';
    temp->z_c += 1;
    temp->h_score = heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  if (this->z_r != 2) {
    Node *temp = new Node(this->state, this->g_score + 1, heuristic, this);
    temp->state[temp->z_r][temp->z_c] = temp->state[temp->z_r + 1][temp->z_c];
    temp->state[temp->z_r + 1][temp->z_c] = '0';
    temp->z_r += 1;
    temp->h_score = heuristic(temp->state);
    temp->f_score = temp->g_score + temp->h_score;
    neighbors.push_back(temp);
  }
  return neighbors;
}

vector<Node *> Node::modified_expand(int depth, heuristic_t heuristic) {
  vector<Node *> temp;
  queue<Node *> q1, q2;
  q1.push(this);
  Node *curr_node;
  while (depth--) {
    swap(q2, q1);
    while (!q2.empty()) {
      curr_node = q2.front();
      q2.pop();
      for (Node *neighbor : curr_node->get_neighbors(heuristic)) {
        q1.push(neighbor);
        temp.push_back(neighbor);
      }
    }
  }
  ::nodes_generated += temp.size();
  return temp;
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

int Frontier::length() { return this->hashtable.size(); }

bool Frontier::empty() { return this->hashlist.empty(); }

void Explored::insert(Node *node) {
  this->hashlist.insert(state_to_string(node->state));
}

bool Explored::contains(Node *node) {
  return this->hashlist.find(state_to_string(node->state)) !=
         this->hashlist.end();
}

vector<Node *> modified_a_star(char initial_state[3][3], heuristic_t heuristic,
                               int depth = 1) {
  Node *node = new Node(initial_state, 0, heuristic);
  Frontier *frontier = new Frontier();
  frontier->insert(node);
  max_fringe_length = max(max_fringe_length, frontier->length());
  Explored *explored = new Explored();
  while (!frontier->empty()) {
    node = frontier->pop();
    if (goal_test(node)) return build_solution(node);
    explored->insert(node);
    for (Node *neighbor : node->modified_expand(depth, heuristic)) {
      if (!explored->contains(neighbor) && !frontier->contains(neighbor)) {
        neighbor->f_score = node->g_score + 1 + neighbor->h_score;
        frontier->insert(neighbor);
        max_fringe_length = max(max_fringe_length, frontier->length());
      } else if (frontier->contains(neighbor) &&
                 neighbor->f_score > node->g_score + 1 + neighbor->h_score) {
        neighbor->f_score = node->g_score + 1 + neighbor->h_score;
        neighbor->parent = node;
      }
    }
  }
  return vector<Node *>();
}

/******************************************************************************/

/********************************** Execution *********************************/
int main() {
  // Controls the prompt at the end of program.
  bool show_opt_path = false;
  // Seed the random number generator with the current system time.
  srand(time(NULL));

  // This vector is used to generate the random states.
  vector<int> base_state({0, 1, 2, 3, 4, 5, 6, 7, 8});
  // This vector will hold NUM_TRIALS solvable initial states to be solved.
  vector<char[3][3]> random_states(NUM_TRIALS);

  // Populate random_states NUM_TRIALS random states
  for (int x = 0; x < NUM_TRIALS; ++x) {
    do {
      // Keep shuffling the current state,
      random_shuffle(base_state.begin(), base_state.end());
      // till it is unsolvable.
    } while (unsolvable_state(base_state));

    // Now copy this solvable state into random_states[x].
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        random_states[x][i][j] = '0' + base_state[3 * i + j];
  }

  // List of depths to test for.
  vector<int> depths({1, 5});
  // List of heuristic functions that will be used for testing.
  vector<heuristic_t> heuristics;
  // Push in both the heuristic functions previously defined.
  heuristics.push_back(manhattan_heuristic);
  heuristics.push_back(mismatch_heuristic);
  // Vector to hold the solution.
  vector<Node *> solution;
  // To keep track of the start and end times for each trial.
  clock_t start_cycles, end_cycles;

  for (int h = 0; h < heuristics.size(); ++h) {
    cout << "Heuristic : " << (h == 0 ? "Manhattan Distance" : "Mismatch")
         << "\n\n";
    for (int depth : depths) {
      cout << "\tDepth : " << depth << "\n";
      for (int x = 0; x < NUM_TRIALS; ++x) {
        // Print details of the trial.

        cout << "\t\tTrial : " << x + 1 << "\n";
        cout << "\t\tStarting state:\n\t\t";
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) cout << random_states[x][i][j] << " ";
          cout << "\n\t\t";
        }

        // Reset the global variables.
        // ::nodes_generated = 1 will include the initial_node too.
        ::nodes_generated = 1, ::max_fringe_length = 0;

        // Store the #(CPU cycles) before call to modified_a_star()
        start_cycles = clock();
        // Make the call to modified_a_star() with appropriate parameters.
        solution = modified_a_star(random_states[x], heuristics[h], depth);
        // Store the #(CPU cycles) after modified_a_star() has returned.
        end_cycles = clock();

        // We are only sending solvable states, so this is just a redundant
        // sanity check.
        if (solution.empty()) cout << "\t\tUnsolvable starting state.\n\n";
        // If a solution was found,
        else {
          // Print result stats.
          cout << "Time taken : "
               << double(end_cycles - start_cycles) / CLOCKS_PER_SEC
               << " sec.\n";
          cout << "\t\tNodes generated : " << ::nodes_generated << "\n";
          cout << "\t\tMaximum fringe length : " << ::max_fringe_length << "\n";
          cout << "\t\tLength of optimal path : " << solution.size() - 1
               << "\n";
          /**** (Un)comment to (not)see the solution paths ****/

          // show_opt_path = true;
          // cout << "\t\tSolution steps :";
          // for (Node *node : solution) node->print();

          /****************************************************/
          cout << "\n\n";
        }
      }
    }
  }

  if (!show_opt_path)
    // Prompt user to make the code print the solution paths as well.
    cout << "\nTo also see the actual optimal paths, user can uncomment the "
            "corresponding section (as indicated in main()).\n\n";

  return 0;
}
