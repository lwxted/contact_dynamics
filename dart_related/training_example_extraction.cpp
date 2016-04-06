/**
 * Simple bouncing ball simulation
 *
 * Author: Ted Li
 */

#include "dart/dart.h"
#include "util/Reporter.h"

#include <random>
#include <chrono>
#include <fstream>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::collision;
using namespace dart::gui;

static const std::string training_examples_path = "/Users/lwxted/Documents/Projects/contact_dynamics/training_examples/";

int training_count = 0;

float ballRadius      = 0.5;
float gravityCoeff    = -9.81;
float restitutionWall = 1;
float frictionWall    = 0.075;
float restitutionBall = 0.8;
float frictionBall    = 0.1;

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

typedef enum {
  kFreeFall,
  kHitContact,
  kBreakContact,
  kMovingUpwards
} ContactMode;


class BouncingBallWindow : public SimWindow
{
private:
  int _contact_mode;
  Reporter<float> _reporter;

public:
  BouncingBallWindow(const WorldPtr& world)
  {
    setWorld(world);
    ballSkeleton = world->getSkeleton("ballSkeleton");
    _contact_mode = 0;
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    SimWindow::keyboard(key, x, y);
  }

  void noteState(FreeJoint *joint)
  {
    std::vector<float> data_point;

    // Input features
    data_point.push_back(joint->getVelocity(0));  // angVelocityD0
    data_point.push_back(joint->getVelocity(1));  // angVelocityD1
    data_point.push_back(joint->getVelocity(2));  // angVelocityD1
    data_point.push_back(joint->getVelocity(3));  // vX
    data_point.push_back(joint->getVelocity(4));  // vY
    data_point.push_back(joint->getVelocity(5));  // vZ
    data_point.push_back(joint->getPosition(0));  // angPosD0
    data_point.push_back(joint->getPosition(1));  // angPosD1
    data_point.push_back(joint->getPosition(2));  // angPosD2
    data_point.push_back(joint->getPosition(3));  // posX
    data_point.push_back(joint->getPosition(4));  // posY
    data_point.push_back(joint->getPosition(5));  // posZ
    data_point.push_back(joint->getPosition(4) + 2.5 - ballRadius / 2);  // distanceToGround
    data_point.push_back(joint->getPosition(3) + 2.5 - ballRadius / 2);  // distanceToWall1
    data_point.push_back(joint->getPosition(5) + 2.5 - ballRadius / 2);  // distanceToWall2
    data_point.push_back(ballRadius);                   // ballRadius
    data_point.push_back(gravityCoeff);                 // gravityCoeff
    data_point.push_back(restitutionWall);              // restitutionWall
    data_point.push_back(frictionWall);                 // frictionWall
    data_point.push_back(restitutionBall);              // restitutionBall
    data_point.push_back(frictionBall);                 // frictionBall

    // Output labels
    data_point.push_back(joint->getAcceleration(0));  // angAccD0
    data_point.push_back(joint->getAcceleration(1));  // angAccD1
    data_point.push_back(joint->getAcceleration(2));  // angAccD2
    data_point.push_back(joint->getAcceleration(3));  // AccX
    data_point.push_back(joint->getAcceleration(4));  // AccY
    data_point.push_back(joint->getAcceleration(5));  // AccZ

    // Store data to reporter
    _reporter.add_data(data_point);
  }

  void reportState()
  {
    std::ostringstream file_path_stream;
    file_path_stream << training_examples_path;
    file_path_stream << "ex" << (training_count++) << ".txt";
    _reporter.dump_to_file_at(file_path_stream.str());
    _reporter.clear_data();
  }

  void timeStepping() override
  {
    int numFrame = (int) (mWorld->getTime() * 1000);
    FreeJoint *joint = (FreeJoint *) ballSkeleton->getJoint("free_joint");

    if (numFrame % 3000 == 2999) {
      float vx = rng.rand();
      float vz = rng.rand();
      reportState();
      mWorld->reset();
      mWorld->setGravity(Eigen::Vector3d(0.0, gravityCoeff, 0.0));
      Eigen::Vector6d resetPosition = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      Eigen::Vector6d resetVelocity = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(vx, 0, vz));
      Eigen::Vector6d resetAcceleration = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      joint->setPositions(resetPosition);
      joint->setVelocities(resetVelocity);
      joint->setAccelerations(resetAcceleration);
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
