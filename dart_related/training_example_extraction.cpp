/**
 * Simple bouncing ball simulation
 *
 * @author Ted Li
 */

#include "dart/dart.h"
#include "util/Reporter.h"
#include "util/Rand.h"
#include "util/ContactModeAnnotator.h"

#include <algorithm>
#include <fstream>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::collision;
using namespace dart::gui;

static std::string output_path;

int training_count = 1000;

// Initial conditions
float ballRadius      = 0.5;
float gravityCoeff    = -9.81;

float restitutionWall = 0.35;
float restitutionBall = 0.35;

float frictionWall    = 0.1;
float frictionBall    = 0.1;

struct Rand rng;
ContactModeAnnotator cma;

class BouncingBallWindow : public SimWindow
{
private:
  Reporter<float> _reporter;

public:
  BouncingBallWindow(const WorldPtr& world)
  {
    setWorld(world);
    ballSkeleton = world->getSkeleton("ballSkeleton");
    wallSkeleton = world->getSkeleton("wallSkeleton");
    world->setTimeStep(world->getTimeStep() / 5.0);
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

    float distToGround = joint->getPosition(4) + 2.25;
    float distToWall1 = joint->getPosition(3) + 2.25;
    float distToWall2 = joint->getPosition(5) + 2.25;

    data_point.push_back(distToGround);  // distanceToGround
    data_point.push_back(distToWall1);  // distanceToWall1
    data_point.push_back(distToWall2);  // distanceToWall2

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
    data_point.push_back(joint->getAcceleration(3));  // angAccD0
    data_point.push_back(joint->getAcceleration(4));  // angAccD1
    data_point.push_back(joint->getAcceleration(5));  // angAccD2


    ContactMode mode = cma.annotate(
      joint->getPosition(3), joint->getPosition(4), joint->getPosition(5),
      joint->getVelocity(4),
      joint->getAcceleration(3),
      joint->getAcceleration(4),
      joint->getAcceleration(5),
      distToGround, distToWall1, distToWall2);

    data_point.push_back(mode);

    // Store data to reporter
    _reporter.add_data(data_point);
  }

  void reportState()
  {
    std::ostringstream file_path_stream;
    file_path_stream << output_path;
    file_path_stream << "/";
    file_path_stream << "ex" << (training_count++) << ".txt";
    _reporter.dump_to_file_at(file_path_stream.str());
    _reporter.clear_data();
    cma = ContactModeAnnotator();
  }

  void timeStepping() override
  {
    int numFrame = (int) (mWorld->getTime() * 1000);
    FreeJoint *ballJoint = (FreeJoint *) ballSkeleton->getJoint("ballJoint");

    if (numFrame % 3000 == 2999) {
      float vx = rng.rand(-6, 0);
      float vz = rng.rand(-6, 0);
      float restitutionBall = rng.rand(0.35, 0.9);
      float restitutionWall = rng.rand(0.35, 0.9);
      float frictionWall = rng.rand(0.05, 0.9);
      float frictionBall = rng.rand(0.05, 0.9);

      reportState();

      // Reset world
      mWorld->reset();

      // Reset gravity
      mWorld->setGravity(Eigen::Vector3d(0.0, gravityCoeff, 0.0));

      // Reset position, velocity and acceleration
      Eigen::Vector6d resetPosition = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      Eigen::Vector6d resetVelocity = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(vx, 0, vz));
      Eigen::Vector6d resetAcceleration = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      ballJoint->setPositions(resetPosition);
      ballJoint->setVelocities(resetVelocity);
      ballJoint->setAccelerations(resetAcceleration);

      BodyNode *ballNode = ballSkeleton->getBodyNode("ballNode");
      BodyNode *wallNode = wallSkeleton->getBodyNode("groundNode");

      // Reset restitution, friction
      ballNode->setRestitutionCoeff(restitutionBall);
      ballNode->setFrictionCoeff(frictionBall);
      wallNode->setRestitutionCoeff(restitutionWall);
      wallNode->setFrictionCoeff(frictionWall);

    } else {
      noteState(ballJoint);
    }

    SimWindow::timeStepping();
  }

protected:
  SkeletonPtr ballSkeleton;
  SkeletonPtr wallSkeleton;
};

int main(int argc, char *argv[])
{
  if (argc != 2) {
    printf("usage: training_example_extraction <example_output_path>\n");
    return 1;
  }

  output_path = std::string(argv[1], argv[1] + strlen(argv[1]));

  // Create ball skeleton, body and joint
  SkeletonPtr ballSkeleton = Skeleton::create("ballSkeleton");
  FreeJoint::Properties ballJointProperties;
  ballJointProperties.mName = "ballJoint";
  BodyNodePtr ballNode = ballSkeleton->createJointAndBodyNodePair
    <FreeJoint>(nullptr, ballJointProperties,
    BodyNode::Properties(std::string("ballNode"))).second;

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
  SkeletonPtr groundSkeleton = Skeleton::create("wallSkeleton");
  WeldJoint::Properties groundJointProperties;
  groundJointProperties.mName = "ground_joint";
  BodyNodePtr groundNode = groundSkeleton->
    createJointAndBodyNodePair<WeldJoint>(
      nullptr, groundJointProperties,
      BodyNode::Properties(std::string("groundNode"))).second;
  groundNode->setRestitutionCoeff(restitutionWall);
  groundNode->setFrictionCoeff(frictionWall);

  // Define the collision + visualization shape of the ground + walls
  // Ground
  std::shared_ptr<BoxShape> boxShape(
    new BoxShape(Eigen::Vector3d(50.0, 0.01, 50.0)));
  boxShape->setOffset(Eigen::Vector3d(0, -2.5, 0));
  groundNode->addVisualizationShape(boxShape);
  groundNode->addCollisionShape(boxShape);

  // Wall 1
  std::shared_ptr<BoxShape> wallShape1(
    new BoxShape(Eigen::Vector3d(0.01, 50.0, 50.0)));
  wallShape1->setOffset(Eigen::Vector3d(-2.5, 0, 0));
  wallShape1->setColor(dart::Color::Gray(0.4));
  groundNode->addVisualizationShape(wallShape1);
  groundNode->addCollisionShape(wallShape1);

  // Wall 2
  std::shared_ptr<BoxShape> wallShape2(
    new BoxShape(Eigen::Vector3d(50.0, 50.0, 0.01)));
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
  ballSkeleton->getJoint("ballJoint")->setVelocities(initial);

  BouncingBallWindow window(world);

  // Initialize glut, the window and begin the glut event loop.
  glutInit(&argc, argv);
  window.initWindow(1280, 1024, "Bouncing Ball Simulation");
  glutMainLoop();
}
