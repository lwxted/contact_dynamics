/**
 * Ball path visualization
 *
 * Author: Ted Li
 */

#include "dart/dart.h"
#include "util/CSVParser.h"

#include "common/ContactMode.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::collision;
using namespace dart::gui;


static const float ballRadius = 0.5;
static float timeStepFactor = 1.0;
static int posX_col, posY_col, posZ_col, mode_col = -1;
static int collision_vis_counter = 0;

static CSVParser<float> parser;


class BallPathWindow : public SimWindow
{
private:
  SkeletonPtr ballSkeleton;

public:
  BallPathWindow(const WorldPtr& world)
  {
    setWorld(world);
    ballSkeleton = world->getSkeleton("ballSkeleton");
  }

  void timeStepping() override
  {
    int numFrame = (int) (mWorld->getTime() * 1000);
    FreeJoint *ballJoint = (FreeJoint *) ballSkeleton->getJoint("ballJoint");

    if (numFrame == parser.data().size() / timeStepFactor) {
      // Reset world
      mWorld->reset();
      collision_vis_counter = 0;
    } else {
      // Update ball position
      BodyNode *ballNode = ballSkeleton->getBodyNode("ballNode");
      Eigen::Vector6d position = Eigen::compose(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(
          parser.data()[numFrame % parser.data().size() * timeStepFactor][posX_col],
          parser.data()[numFrame % parser.data().size() * timeStepFactor][posY_col],
          parser.data()[numFrame % parser.data().size() * timeStepFactor][posZ_col]
      ));
      ballJoint->setPositions(position);

      if (mode_col != -1) {
        if (collision_vis_counter > 0) {
          ballNode->getVisualizationShape(0)->setColor(dart::Color::Red());
          --collision_vis_counter;
        } else {
          switch ((ContactMode) (int) parser.data()[numFrame % parser.data().size() * timeStepFactor][mode_col]) {
            case kUndefined:
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Black());
              break;

            case kFreeFall:
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Orange(1));
              break;

            case kHitContactGround:
              std::cout << "Collision with ground at t=" << ' ' << numFrame << std::endl;
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Red());
              collision_vis_counter = 50;
              break;

            case kHitContactWall1:
              std::cout << "Collision with wall 1 at t=" << ' ' << numFrame << std::endl;
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Red());
              collision_vis_counter = 50;
              break;

            case kHitContactWall2:
              std::cout << "Collision with wall 2 at t=" << ' ' << numFrame << std::endl;
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Red());
              collision_vis_counter = 50;
              break;

            case kBreakContact:
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Red());
              break;

            case kMovingUpwards:
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Orange(0.25));
              break;

            case kProbablyStatic:
              ballNode->getVisualizationShape(0)->setColor(dart::Color::Gray());
              break;
          }
        }
      }
    }

    SimWindow::timeStepping();
  }
};


int main(int argc, char *argv[])
{
  if (argc < 6) {
    printf("usage: ball_path_visualization <training_data> "
      "<time_step_factor> <posX_col> <posY_col> <posZ_col>\n");
    return 1;
  }

  if (argc == 7) {
    mode_col = atoi(argv[6]);
  }

  // Read in data
  std::string data_path(argv[1], argv[1] + strlen(argv[1]));

  parser.load_from_file_at(
    data_path,
    [](const std::string& s) {
      return std::stof(s);
    }
  );

  timeStepFactor = atof(argv[2]);
  posX_col = atoi(argv[3]);
  posY_col = atoi(argv[4]);
  posZ_col = atoi(argv[5]);

  // Construct world
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

  // Create ground skeleton, body and joint
  SkeletonPtr groundSkeleton = Skeleton::create("wallSkeleton");
  WeldJoint::Properties groundJointProperties;
  groundJointProperties.mName = "ground_joint";
  BodyNodePtr groundNode = groundSkeleton->
    createJointAndBodyNodePair<WeldJoint>(
      nullptr, groundJointProperties,
      BodyNode::Properties(std::string("groundNode"))).second;

  // Define the collision + visualization shape of the ground + walls
  // Ground
  std::shared_ptr<BoxShape> boxShape(
    new BoxShape(Eigen::Vector3d(5.0, 0.05, 5.0)));
  boxShape->setOffset(Eigen::Vector3d(0, -2.5, 0));
  groundNode->addVisualizationShape(boxShape);
  groundNode->addCollisionShape(boxShape);

  // Wall 1
  std::shared_ptr<BoxShape> wallShape1(
    new BoxShape(Eigen::Vector3d(0.05, 5.0, 5.0)));
  wallShape1->setOffset(Eigen::Vector3d(-2.5, 0, 0));
  wallShape1->setColor(dart::Color::Gray(0.4));
  groundNode->addVisualizationShape(wallShape1);
  groundNode->addCollisionShape(wallShape1);

  // Wall 2
  std::shared_ptr<BoxShape> wallShape2(
    new BoxShape(Eigen::Vector3d(5.0, 5.0, 0.05)));
  wallShape2->setOffset(Eigen::Vector3d(0, 0, -2.5));
  wallShape2->setColor(dart::Color::Gray(0.4));
  groundNode->addVisualizationShape(wallShape2);

  WorldPtr world(new World);
  world->addSkeleton(ballSkeleton);
  world->addSkeleton(groundSkeleton);

  // No gravity
  Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  world->setGravity(gravity);

  // Set initial conditions
  ballShape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.0));
  Eigen::Vector6d initial = Eigen::compose(
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
  ballSkeleton->getJoint("ballJoint")->setVelocities(initial);

  BallPathWindow window(world);

  // Initialize glut, the window and begin the glut event loop.
  glutInit(&argc, argv);
  window.initWindow(1280, 1024, "Ball Path Visualizer");
  glutMainLoop();
}
