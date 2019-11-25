#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

using namespace std;

#ifdef TESTING
#include "../red/test_runner.h"
#else
template <class T>
ostream& operator<< (ostream& os, const vector<T>& s) {
  os << "{";
  bool first = true;
  for (const auto& x : s) {
    if (!first) {
      os << ", ";
    }
    first = false;
    os << x;
  }
  return os << "}";
}
#endif //testing

float trun(float val) {
  return trunc(val * 100000) / 100000;
}

template <typename T>
struct Vec {
  Vec(T x, T y) : x(x), y(y) {}

  Vec<int> toInt() const {
    return Vec<int>(static_cast<int>(x), static_cast<int>(y));
  }

  Vec<float> toFloat() const {
    return Vec<float>(static_cast<float>(x), static_cast<float>(y));
  }

  Vec<T> operator+(const Vec<T>& a) const {
    return Vec(x + a.x, y + a.y);
  }

  Vec<T>& operator+=(const Vec<T>& a) {
    x += a.x;
    y += a.y;
    return *this;
  }

  Vec<T> operator-(const Vec<T>& a) const {
    return Vec(x - a.x, y - a.y);
  }

  Vec<T>& operator-=(const Vec<T>& a) {
    x -= a.x;
    y -= a.y;
    return *this;
  }

  Vec<T> operator*(T val) const {
    return Vec(x * val, y * val);
  }

  friend Vec<T> operator*(T val, const Vec<T>& vec) {
    return vec * val;
  }

  Vec<T>& operator*=(T val) {
    x *= val;
    y *= val;
    return *this;
  }

  float operator*(const Vec<T>& a) const {
    return x * a.x + y * a.y;
  }

  Vec<float> operator/(T val) const {
    float v = static_cast<float>(val);
    return Vec<float>(x / v, y / v);
  }

  bool operator==(const Vec<int> a) const {
    return x == a.x && y == a.y;
  }

  bool operator==(const Vec<float> a) const {
    return trun(x) == trun(a.x) && trun(y) == trun(a.y);
  }

  friend ostream& operator<<(ostream& os, Vec<T> a) {
    os << "(" << a.x << ", " << a.y << ")";
    return os;
  }

  void rotate(int angleDeg) {
    float angle = angleDeg * 2 * M_PI / 360;
    float newx = *this * Vec<float>(cos(angle), sin(-angle));
    float newy = *this * Vec<float>(sin(angle), cos(angle));
    x = newx; y = newy;
  }

  // orthodoxal counterclockwise
  float angle(const Vec<T> a) const {
    return atan2<float, float>(this->outerZ(a), *this * a);
  }

  T L2Sq() const {
    return x * x + y * y;
  }

  float L2Norm() const {
    return sqrt(L2Sq());
  }

  T outerZ(const Vec<T>& a) const {
    return x * a.y - y * a.x;
  }

  T x;
  T y;
};

template <typename T>
T Sq(T x) {
  return x * x;
}

struct Control {
  static const int SHIELD = -1;
  static const int BOOST = -2;
  int angle;
  int power;

  friend ostream& operator<<(ostream& os, Control c) {
    os << "{" << c.angle << ", " << c.power << "}";
    return os;
  }
};

struct Thing {
  Vec<int> r;
  Vec<int> v;
  const int R;

  Thing (int R, int x, int y, int vx, int vy) : R(R), r(x, y), v(vx, vy) {}
};

struct Checkpoint : public Thing {
  Checkpoint(int x, int y) : Thing(200, x, y, 0, 0) {}
};

struct Pod : public Thing {
  Pod(int x, int y, int vx, int vy) : Thing(400, x, y, vx, vy), facing(1, 0) {} // debug

  Pod(int x, int y, int vx, int vy, int angle, bool* hasBoost) : 
    Thing(400, x, y, vx, vy), facing(1, 0), hasBoost(hasBoost) 
  {
    rotate(angle);
  }

  //debug codinGames: strange problems with pointers
  Pod(int x, int y, int vx, int vy, int angle) : Thing(400, x, y, vx, vy), facing(1, 0) 
  {
    rotate(angle);
  }

  static const int MAX_ANGLE_CHANGE = 18;
  static const int MAX_POWER = 100;
  static const int BOOST = 650;

  // postive direction is clockwise
  void rotate(int angleDeg) {
    facing.rotate(angleDeg);
    debugCounter++;
  }

  void setAngle(int angleDeg) {
    angle = angleDeg;
    facing = Vec<float>(1, 0);
    rotate(angleDeg);
  }

  int angle;
  int nextCheckpoint = 1;
  int checkpointsPassed = 1;
  bool* hasBoost;
  int shieldCd = 0;
  Vec<float> facing;
  int debugCounter = 0;
};

class World {
public:
  World(vector<Checkpoint>& checkpoints, vector<Pod>& pods) : 
      checkpoints(checkpoints), pods(pods) {}

  static constexpr float blinkTime = 1.2;
  static float collision (const Thing& a, const Thing& b) {    
    const auto& relv = a.v - b.v   ; // realtive speed of pod a          // |  (b)
    const auto& relbr = b.r - a.r;                                       // |/
    float relvNormSq = relv.L2Sq();                                        // (a)___
    
    int innerProd = relv * relbr;

    if (innerProd < 0)
      return -1;

    // dist from b's center to a's movement line
    float roSq = Sq(relv.outerZ(relbr)) / relvNormSq;

    if (roSq > Sq(a.R + b.R))
      return -1;


    float x = innerProd / relvNormSq - sqrt((Sq(a.R + b.R) - roSq) / relvNormSq);
    return innerProd / relvNormSq - sqrt((Sq(a.R + b.R) - roSq) / relvNormSq);
  }

  // to avoid doublecounting collisions consider only collisions in (0, T]
  void blink(const vector<Control>& controls) {
    for (int i = 0; i < pods.size(); i++) {
      if (pods[i].shieldCd) pods[i].shieldCd--;
      pods[i].v = (pods[i].v.toFloat() * 0.85).toInt();
      pods[i].rotate(controls[i].angle);
      int power = controls[i].power;
      if (power == Control::SHIELD && !pods[i].shieldCd)
        pods[i].shieldCd = 3;

      if (pods[i].shieldCd)
        continue;
      else if (power > 0) {
        pods[i].v += (pods[i].facing * power).toInt();
      }

      else if (power == Control::BOOST && *pods[i].hasBoost)
        pods[i].v += (pods[i].facing * Pod::BOOST).toInt();
    }

    // cout << "p.v : " << pods[0].v << endl;
    // cout << "p.f : " << pods[0].facing << endl;

    float timeRemain = World::blinkTime;
    while (timeRemain > 0) {
      float timeToNearestCollision;
      int aIndex, bIndex;
      tie(timeToNearestCollision, aIndex, bIndex) = nearestCollission();
      // cout << "timeToNearestCollision : " << timeToNearestCollision << endl;


      // TODO: simultaneous collisions
      if (timeToNearestCollision <= timeRemain) {
        move(timeToNearestCollision);
        updateNextCheckpoints(timeToNearestCollision);
        kiss(pods[aIndex], pods[bIndex], timeToNearestCollision);

        timeRemain -= timeToNearestCollision;
      } else {
        move(timeRemain);
        updateNextCheckpoints(timeRemain);
        timeRemain = 0;
      }
    }
  }

  vector<Pod> pods;
  vector<Checkpoint>& checkpoints;


private:
  tuple<float, int, int> nearestCollission() {
    float timeToNearestCollision = numeric_limits<float>::max();
    int aIndex, bIndex;
    for (int i = 0; i < pods.size(); i++) {
      for (int j = 0; j < i; j++) {
        Pod& a = pods[i];
        Pod& b = pods[j];
        float t = collision(a, b);
        if (t > 0 && t <= timeToNearestCollision) {
          timeToNearestCollision = t;
          aIndex = i; bIndex = j;
        }
      }
    }
    if (timeToNearestCollision < 0)
      timeToNearestCollision = numeric_limits<float>::max();
    return make_tuple(timeToNearestCollision, aIndex, bIndex);
  }

  void updateNextCheckpoints(float timeToNearestCollision) {
    for (auto& pod : pods) {
      // don't need assuming only one checkpoint coild be passed in a blink. perfomance costly(?)
      // vector<int> indexes(checkpoints.size());
      // iota(indexes.begin(), indexes.end(), 0);
      // sort(indexes.begin(), indexes.end(), [checkpoints](int x, int y) 
      //   {return dist(p, checkpoints[x]) < dist(p, checkpoints[y]);})
      for (int i = 0; i < checkpoints.size(); i++) {
        if (i != pod.nextCheckpoint)
          continue;
        float t = collision(pod, checkpoints[i]);
        if (t > 0 && t <= timeToNearestCollision) {
            pod.nextCheckpoint = (pod.nextCheckpoint + 1) % checkpoints.size();
            pod.checkpointsPassed++;
        }
      }
    }
  }

  void kiss(Pod& a, Pod& b, float t) {
    // cout << "!! COLLISION !!" << endl;
    a.r += t * a.v;
    b.r += t * b.v;
    int ma = a.shieldCd == 3 ? 10 : 1;
    int mb = b.shieldCd == 3 ? 10 : 1;

    const auto& vMassCenter = (ma * a.v + mb * b.v) / (ma + mb);
    auto pImpactA = -2 * ma * (a.v.toFloat() - vMassCenter);
    pImpactA *= max(1.0f, 120 / pImpactA.L2Norm());
    a.v += (pImpactA / ma).toInt();
    b.v -= (pImpactA / mb).toInt();

  }

  void move(float time) {
    for (auto& pod : pods)
      pod.r += (pod.v.toFloat() * time).toInt();
  }

};

struct Node;

struct Node {
  Node(Node* parent, World world) : parent(parent), world(world) {}
  float score;
  Node* parent;
  vector<Node> children;
  vector<Control> controls; // {forMyPod1, forMyPod2, forEnemyPod1, forEnemyPod2}
  World world;
};

class Brain {
public:
  static const vector<int> angles; 
  static const vector<int> powers;
  static const int MAX_DEPTH = 2; // root has depth = 0, must be even to maximize score

  static vector<Control> optimalControl(const World& world) {
    //return {{18, 100}, {18, 100}};
    Node root(nullptr, world);
    //root.score = vanillaMinimax(root, 0);
    root.score = alphaBeta(root, -numeric_limits<float>::max(), numeric_limits<float>::max(), 0);
    vector<Node> optimalNodes;
    for (const auto& child : root.children) {
      if (child.score == root.score) {
        optimalNodes.push_back(child);
        cerr << child.controls << endl;
      }
    }
    cerr << " score: " << root.score << endl;
    return selectRandom(optimalNodes).controls;
  }
private:
  // paranoid
  static float vanillaMinimax(Node& node, int depth) {
    if (depth == Brain::MAX_DEPTH) {
      return estimate(node);
    }
    generateChildren(node, depth);
    float score = -numeric_limits<float>::max();
    for (Node& child : node.children) {
      child.score = -vanillaMinimax(child, depth + 1);
      if (child.score > score)
        score = child.score;
    }
    return score;
  }
  static float alphaBeta(Node& node, float alpha, float beta, int depth) {
    if (depth == Brain::MAX_DEPTH) {
      return estimate(node);
    }

    generateChildren(node, depth);
    float score;
    if (!depth % 2) {
      score = -numeric_limits<float>::max();
      for (Node& child : node.children) {
        child.score = alphaBeta(child, alpha, beta, depth + 1);
        score = max(score, child.score);
        alpha = max(alpha, score);
        if (alpha >= beta)
          return score;
      }
    } else {
      score = numeric_limits<float>::max();
      for (Node& child : node.children) {
        child.score = alphaBeta(child, alpha, beta, depth + 1);
        score = min(score, child.score);
        beta = min(beta, score);
        if (alpha >= beta)
          return score;
      }
    }
    return score;
  }

  static void generateChildren(Node& node, int depth) {
    for (int angle1 : Brain::angles) {
      for (int power1 : Brain::powers) {
        for (int angle2 : Brain::angles) {
          for (int power2 : Brain::powers) {
            Node child(&node, node.world);
            //if (node.parent != nullptr)
              //cout << "1: " << node.world.pods[0].r << " " << child.world.pods[0].r << endl;
            if (!(depth % 2)) // my turn
              child.controls = {{angle1, power1}, {angle2, power2}};
            else {
              child.controls = node.controls;
              child.controls.push_back({angle1, power1});
              child.controls.push_back({angle2, power2});
              child.world.blink(child.controls);
              //cout << "2: " << child.world.pods[0].r << endl;
            }
            node.children.push_back(child);
          }
        }
      }
    }
  }

  static float reward2(const Pod& p1, const Pod& p2, const vector<Checkpoint>& cps) {
    const float leaderReward = 40000;
    const float followerReward = 30000;
    const float speedCoef = 0.5;
    const float angleCoef = 500;
    const float shieldPenalty = 50;
    float reward = 0;
    reward += max(p1.checkpointsPassed, p2.checkpointsPassed)*leaderReward;
    reward += min(p1.checkpointsPassed, p2.checkpointsPassed)*followerReward;
    for (const auto& p: {p1, p2}) {
      reward -= (cps[p.nextCheckpoint].r - p.r).L2Norm();
      reward += p.v.L2Norm() * speedCoef;
      reward -= abs(p.facing.angle((cps[p.nextCheckpoint].r - p.r).toFloat())) * angleCoef;
      reward -= (p.shieldCd > 0)*shieldPenalty;
    }
    return reward;
  }

  static float estimate(const Node& node) {
    const auto& pods = node.world.pods;
    const auto& checkpoints = node.world.checkpoints;
    const auto& myPod1 = pods[0];
    const auto& myPod2 = pods[1];
    const auto& enemyPod1 = pods[2];
    const auto& enemyPod2 = pods[3];
    float myReward = reward2(myPod1, myPod2, checkpoints);
    float enemyReward = reward2(enemyPod1, enemyPod2, checkpoints);
  
    // cerr << node.controls << " " << myReward - enemyReward << " " ;
    // cerr << myPod1.r << " " << myPod2.r << "\n";
    //cout << " passed: " << Vec<int>(enemyPod1.checkpointsPassed, enemyPod2.checkpointsPassed) << endl;

    return myReward - enemyReward;
  }

  template <typename T>
  static T selectRandom(const vector<T>& v) {
      static std::random_device rd;
      static std::mt19937 gen(rd());
      uniform_int_distribution<> distribution(0, v.size() - 1);
      return v[distribution(gen)];
  }

};

const vector<int> Brain::angles {-18, 0, 18};
const vector<int> Brain::powers {0, 100, Control::SHIELD};//, Control::SHIELD, Control::BOOST};

#ifdef TESTING

void TestCollision() {
  Pod p1 (10, 10, 30, 40);
  Checkpoint c1(6010 + 120*3, 8010 + 120*4);
  float t = World::collision(p1, c1);
  ASSERT_EQUAL(t, 200);

  t = World::collision(c1, p1);
  ASSERT_EQUAL(round(t), 200);

  Pod p2 (10, 10, -30, 40);
  t = World::collision(p2, c1);
  ASSERT_EQUAL(t, -1);

  Pod p3 (-1200, -100, 50, 228);
  Pod p4 (1600, -100, -50, 228);
  t = World::collision(p3, p4);
  ASSERT_EQUAL(t, 20.0);
}

// coordinates are extraordinary, Oy looks south
void TestRotate() {
  Pod p(0, 0, 0, 0);
  p.rotate(-90);
  ASSERT_EQUAL(Vec<float>(0, -1), p.facing)
  p.rotate(45);
  ASSERT_EQUAL(Vec<float>(sqrt(2) / 2, -sqrt(2) / 2), p.facing)
}

void TestAngle() {
  Vec<float> a (1, 1);
  Vec<float> b (1, 0);

  ASSERT_EQUAL(trun(a.angle(b)), trun(-M_PI / 4));
  ASSERT_EQUAL(trun(b.angle(a)), trun(M_PI / 4));
  Vec<float> c (0, 1);
  ASSERT_EQUAL(trun(b.angle(c)), trun(M_PI / 2));
  Vec<float> d (-1, 0);
  ASSERT_EQUAL(trun(b.angle(d)), trun(M_PI));
}

void TestBlink() {
  bool hasBoost = true;
  vector<Pod> pv {{0,0,0,0, 0,&hasBoost}, {850,0,0,0, 0,&hasBoost}};
  Checkpoint cp(15000, 9000);
  vector<Checkpoint> cv {{0, 0}, cp};
  World world(cv, pv);
  Pod& p1 = world.pods[0];
  Pod& p2 = world.pods[1];
  world.blink({{0, 100}, {-180, 100}});

  ASSERT_EQUAL(Vec<int>(-0.7 * 100, 0), p1.r)
  ASSERT_EQUAL(Vec<int>(850 + 0.7 * 100, 0), p2.r)

  ASSERT_EQUAL(Vec<int>(-100, 0), p1.v);
  ASSERT_EQUAL(Vec<int>(100, 0), p2.v);
  int i = 0;
  for (; i < 100 && p1.checkpointsPassed == 1; i++) {

    float angle = p1.facing.angle((cp.r - p1.r).toFloat());
    int angleInt = angle / (2 * M_PI) * 360;
    if (angleInt < -Pod::MAX_ANGLE_CHANGE)
      angleInt = -Pod::MAX_ANGLE_CHANGE;
    else if (angleInt > Pod::MAX_ANGLE_CHANGE)
      angleInt = Pod::MAX_ANGLE_CHANGE;
    world.blink({{angleInt, 100}, {-5, 100}});
  }
  ASSERT(i < 50);
  ASSERT((p1.r - cp.r).L2Norm() < 1500);
}

void TestDecision() {
  Pod mp1(0, 0, 0, 0);
  Pod mp2(5000, 5000, 0, 0);
  mp2.rotate(180);
  Pod ep1(5900, 5500, 0, 0);
  ep1.rotate(180);
  Pod ep2(10000, 8000, 0, 0);
  ep2.rotate(180);
  Checkpoint cp1(1500, 8000);
  Checkpoint cp2(3000, 5000);
  vector<Pod> pods {mp1, mp2, ep1, ep2};
  vector<Checkpoint> cps {cp1, cp2};
  World world(cps, pods);
  cout << Brain::optimalControl(world) << endl;
}


int main() {
  TestRunner tr;
  RUN_TEST(tr, TestCollision);
  RUN_TEST(tr, TestRotate);
  RUN_TEST(tr, TestBlink);
  RUN_TEST(tr, TestAngle);
  RUN_TEST(tr, TestDecision);
}

#else //TESTING
int main() {
  int laps;
  cin >> laps; cin.ignore();
  int checkpointCount;
  cin >> checkpointCount; cin.ignore();
  vector<Checkpoint> checkpoints;
  for (int i = 0; i < checkpointCount; i++) {
      int checkpointX;
      int checkpointY;
      cin >> checkpointX >> checkpointY; cin.ignore();
      checkpoints.emplace_back(checkpointX, checkpointY);
  }
  // bool* hasBoost;
  // *hasBoost = false;
  vector<Pod> pods(4, {0,0,0,0,0});
  int round = 0;
  while (1) {
    for (int i = 0; i < 4; i++) {
        int x; 
        int y; 
        int vx; 
        int vy; 
        int angle;
        int nextCp;
        cin >> pods[i].r.x >> pods[i].r.y >> pods[i].v.x >> pods[i].v.y >> angle >> nextCp; 
        cin.ignore();
        // cerr << "angle: " << angle << " ";
        if (!round) {
          angle = Vec<float>(1, 0).angle((checkpoints[1].r - pods[i].r).toFloat()) * 180 / M_PI;
          cerr << " AAAAAA ngle " << angle;
        }
        pods[i].setAngle(angle);

        // TODO: remove
        if (pods[i].nextCheckpoint != nextCp) {
          int oldNc = pods[i].nextCheckpoint;
          pods[i].nextCheckpoint = nextCp;
          pods[i].checkpointsPassed += 
            (nextCp > oldNc ? nextCp - oldNc : nextCp + (checkpoints.size() - oldNc));
        }
    }
    cerr << "coord: " << pods[0].r << endl;
    World world(checkpoints, pods);
    auto controls = Brain::optimalControl(world);
    for (int i = 0; i < 2; i++) {
      Vec<float> unit(1, 0);
      unit.rotate(pods[i].angle + controls[i].angle);
      cerr << pods[i].angle << " " << controls[i].angle << " " << unit << endl;
      Vec<int> desired = pods[i].r + (unit * 10000).toInt();
      cout << desired.x << " " << desired.y << " ";
      int power = controls[i].power;
      if (power >= 0)
        cout << power << endl;
      else if (power == Control::SHIELD)
        cout << "SHIELD" << endl;
    }
    round++;
  }
}

#endif //TESTING
