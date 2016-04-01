/**
 * Simple bouncing ball simulation
 *
 * Author: Ted Li
 */

#include "dart/dart.h"
#include <random>
#include <chrono>
#include <fstream>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::collision;
using namespace dart::gui;

int training_count = 0;

float ballRadius      = 0.5;
float gravityCoeff    = -9.81;
float restitutionWall = 1;
float frictionWall    = 0.075;
float restitutionBall = 0.8;
float frictionBall    = 0.1;

std::vector<Eigen::Vector6d> velocity;
std::vector<Eigen::Vector6d> position;
std::vector<float> toGround;
std::vector<float> toWall1;
std::vector<float> toWall2;

std::vector<Eigen::Vector6d> acceleration;

typedef struct Rand {
private:
  std::mt19937_64 rng;
  uint64_t timeSeed;
  std::seed_seq *ss;
  std::uniform_real_distribution<double> unif;

public:
  Rand() {
    std::mt19937_64 rng;
    timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    ss = new std::seed_seq{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(*ss);
    unif = std::uniform_real_distribution<double>(-10, 0);
  }

  float rand() {
    return unif(rng);
  }
} Rand;

Rand rng;


class BouncingBallWindow : public SimWindow
{
public:
  BouncingBallWindow(const WorldPtr& world)
  {
    setWorld(world);
    ballSkeleton = world->getSkeleton("ballSkeleton");
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    SimWindow::keyboard(key, x, y);
  }

  void noteState(FreeJoint *joint)
  {
    float posX = joint->getPosition(3);
    float posY = joint->getPosition(4);
    float posZ = joint->getPosition(5);

    velocity.push_back(joint->getVelocities());
    position.push_back(joint->getPositions());
    toGround.push_back(2.5 + posY);
    toWall1.push_back(2.5 + posX);
    toWall2.push_back(2.5 + posZ);

    acceleration.push_back(joint->getAccelerations());
  }

  void reportState()
  {
    std::ofstream output;
    std::ostringstream file_path_stream;
    // TODO: get rid of this magic path
    file_path_stream << "/Users/lwxted/Documents/Projects/contact_dynamics/training_examples/";
    file_path_stream << "ex" << (training_count++) << ".txt";
    output.open(file_path_stream.str());
    for (int i = 0; i < acceleration.size(); ++i) {
      output << velocity[i][0] << ", "
             << velocity[i][1] << ", "
             << velocity[i][2] << ", "
             << velocity[i][3] << ", "
             << velocity[i][4] << ", "
             << velocity[i][5] << ", "
             << position[i][0] << ", "
             << position[i][1] << ", "
             << position[i][2] << ", "
             << position[i][3] << ", "
             << position[i][4] << ", "
             << position[i][5] << ", "
             << toGround[i] << ", "
             << toWall1[i] << ", "
             << toWall2[i] << ", "
             << ballRadius         << ", "
             << gravityCoeff       << ", "
             << restitutionWall    << ", "
             << frictionWall       << ", "
             << restitutionBall    << ", "
             << frictionBall       << ", "
             << acceleration[i][0] << ", "
             << acceleration[i][1] << ", "
             << acceleration[i][2] << ", "
             << acceleration[i][3] << ", "
             << acceleration[i][4] << ", "
             << acceleration[i][5] << ", " << std::endl;
    }
    output.close();
    velocity.clear();
    position.clear();
    toGround.clear();
    toWall1.clear();
    toWall2.clear();
    acceleration.clear();
  }

  void timeStepping() override
  {
    int numFrame = (int) (mWorld->getTime() * 1000);
    FreeJoint *joint = (FreeJoint *) ballSkeleton->getJoint("free_joint");

    if (numFrame % 3000 == 0) {
      float vx = rng.rand();
      float vy = rng.rand();
      reportState();
      mWorld->reset();
      mWorld->setGravity(Eigen::Vector3d(0.0, gravityCoeff, 0.0));
      Eigen::Vector6d resetPosition = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      Eigen::Vector6d resetVelocity = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(vx, 0, vy));
      joint->setPositions(resetPosition);
      joint->setVelocities(resetVelocity);
    } else {
      noteState(joint);
    }

    SimWindow::timeStepping();
  }

protected:
  SkeletonPtr ballSkeleton;
};

int main(int argc, char *argv[])
{
  // Create ball skeleton, body and joint
  SkeletonPtr ballSkeleton = Skeleton::create("ballSkeleton");
  FreeJoint::Properties ballJointProperties;
  ballJointProperties.mName = "free_joint";
  BodyNodePtr ballNode = ballSkeleton->createJointAndBodyNodePair
    <FreeJoint>(nullptr, ballJointProperties,
    BodyNode::Properties(std::string("ball"))).second;

  // Define the collision + visualization shape of the ball
  std::shared_ptr<EllipsoidShape> ballShape(
    new EllipsoidShape(Eigen::Vector3d(ballRadius, ballRadius, ballRadius)));
  ballShape->setColor(dart::Color::Orange());
  ballShape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.0));
  ballNode->addVisualizationShape(ballShape);
  ballNode->addCollisionShape(ballShape);
  ballNode->setRestitutionCoeff(restitutionBall);
  ballNode->setFrictionCoeff(frictionBall);

  // Create ground skeleton, body and joint
  SkeletonPtr groundSkeleton = Skeleton::create("groundSkeleton");
  WeldJoint::Properties groundJointProperties;
  groundJointProperties.mName = "ground_joint";
  BodyNodePtr groundNode = groundSkeleton->
    createJointAndBodyNodePair<WeldJoint>(
      nullptr, groundJointProperties,
      BodyNode::Properties(std::string("ground"))).second;
  groundNode->setRestitutionCoeff(restitutionWall);
  groundNode->setFrictionCoeff(frictionWall);

  // Define the collision + visualization shape of the ground + walls
  // Ground
  std::shared_ptr<BoxShape> boxShape(
    new BoxShape(Eigen::Vector3d(200.0, 0.05, 200.0)));
  boxShape->setOffset(Eigen::Vector3d(0, -2.5, 0));
  groundNode->addVisualizationShape(boxShape);
  groundNode->addCollisionShape(boxShape);

  // Wall 1
  std::shared_ptr<BoxShape> wallShape1(
    new BoxShape(Eigen::Vector3d(0.05, 200.0, 200.0)));
  wallShape1->setOffset(Eigen::Vector3d(-2.5, 0, 0));
  wallShape1->setColor(dart::Color::Gray(0.4));
  groundNode->addVisualizationShape(wallShape1);
  groundNode->addCollisionShape(wallShape1);

  // Wall 2
  std::shared_ptr<BoxShape> wallShape2(
    new BoxShape(Eigen::Vector3d(200.0, 200.0, 0.05)));
  wallShape2->setOffset(Eigen::Vector3d(0, 0, -2.5));
  wallShape2->setColor(dart::Color::Gray(0.4));
  groundNode->addVisualizationShape(wallShape2);
  groundNode->addCollisionShape(wallShape2);

  WorldPtr world(new World);
  world->addSkeleton(ballSkeleton);
  world->addSkeleton(groundSkeleton);

  // Gravity
  Eigen::Vector3d gravity(0.0, gravityCoeff, 0.0);
  world->setGravity(gravity);

  // Set initial conditions
  ballShape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.0));
  Eigen::Vector6d initial = Eigen::compose(
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
  ballSkeleton->getJoint("free_joint")->setVelocities(initial);

  BouncingBallWindow window(world);

  // Initialize glut, the window and begin the glut event loop.
  glutInit(&argc, argv);
  window.initWindow(1280, 1024, "Bouncing Ball Simulation");
  glutMainLoop();
}
