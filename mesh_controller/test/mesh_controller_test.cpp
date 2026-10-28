#include <gtest/gtest.h>
#include <mbf_msgs/action/exe_path.hpp>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/mesh_map.h>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace ::testing;

struct MeshControllerTest : public Test
{
protected:

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    controller_.reset();
    mesh_map_ptr_.reset();
    tf_buffer_ptr_.reset();
    node_ptr_.reset();
    rclcpp::shutdown();
  }

  void initController(const std::function<float(float)> & height, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    const auto dir = std::filesystem::temp_directory_path() / ("mesh_controller_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const auto mesh_file = dir / "grid.ply";
    const size_t columns = 31;
    const size_t rows = 11;
    {
      std::ofstream ply(mesh_file);
      ply << "ply\nformat ascii 1.0\nelement vertex " << columns * rows
          << "\nproperty float x\nproperty float y\nproperty float z\nelement face " << 2 * (columns - 1) * (rows - 1)
          << "\nproperty list uchar int vertex_indices\nend_header\n";
      for (size_t row = 0; row < rows; row++) {
        for (size_t column = 0; column < columns; column++) {
          ply << column * 0.1 << " " << row * 0.1 << " " << height(column * 0.1) << "\n";
        }
      }
      for (size_t row = 0; row + 1 < rows; row++) {
        for (size_t column = 0; column + 1 < columns; column++) {
          const size_t a = row * columns + column;
          const size_t b = a + 1;
          const size_t c = b + columns;
          const size_t d = a + columns;
          ply << "3 " << a << " " << b << " " << c << "\n3 " << a << " " << c << " " << d << "\n";
        }
      }
    }

    node_ptr_ = std::make_shared<rclcpp::Node>("mesh_controller", "test", rclcpp::NodeOptions(options)
      .append_parameter_override("mesh_map.mesh_file", mesh_file.string())
      .append_parameter_override("mesh_map.mesh_part", "/")
      .append_parameter_override("mesh_map.mesh_working_file", (dir / "grid.h5").string())
      .append_parameter_override("mesh_map.mesh_working_part", "mesh")
      .append_parameter_override("mesh_map.layers", std::vector<std::string>{"height_diff", "inflation"})
      .append_parameter_override("mesh_map.default_layer", "inflation")
      .append_parameter_override("mesh_map.height_diff.type", "mesh_layers/HeightDiffLayer")
      .append_parameter_override("mesh_map.height_diff.threshold", 0.1)
      .append_parameter_override("mesh_map.height_diff.radius", 0.4)
      .append_parameter_override("mesh_map.inflation.type", "mesh_layers/InflationLayer")
      .append_parameter_override("mesh_map.inflation.inputs", std::vector<std::string>{"height_diff"})
      .append_parameter_override("mesh_map.inflation.inscribed_radius", 0.1)
      .append_parameter_override("mesh_map.inflation.inflation_radius", 0.3)
      .append_parameter_override("mesh_map.inflation.lethal_value", std::numeric_limits<double>::infinity()));
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    mesh_map_ptr_ = std::make_shared<mesh_map::MeshMap>(*tf_buffer_ptr_, node_ptr_);
    ASSERT_TRUE(mesh_map_ptr_->readMap());
    std::filesystem::remove_all(dir);

    const auto mesh = mesh_map_ptr_->mesh();
    lvr2::DenseVertexMap<mesh_map::Vector> vector_map(mesh->nextVertexIndex(), mesh_map::Vector(1, 0, 0));
    mesh_map_ptr_->setVectorMap(vector_map);

    controller_ = std::make_shared<mesh_controller::MeshController>();
    ASSERT_TRUE(controller_->initialize("mesh_controller", tf_buffer_ptr_, mesh_map_ptr_, node_ptr_));
  }

  static geometry_msgs::msg::PoseStamped pose(const double x, const double y, const double z, const double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    return pose;
  }

  bool cycle(const geometry_msgs::msg::PoseStamped & robot_pose, geometry_msgs::msg::TwistStamped & cmd_vel)
  {
    std::string message;
    for (int i = 0; i < 2; i++) {
      if (controller_->isGoalReached(dist_tolerance_, angle_tolerance_)) {
        return true;
      }
      EXPECT_EQ(
        controller_->computeVelocityCommands(robot_pose, geometry_msgs::msg::TwistStamped(), cmd_vel, message),
        mbf_msgs::action::ExePath::Result::SUCCESS);
    }
    return controller_->isGoalReached(dist_tolerance_, angle_tolerance_);
  }

  const double dist_tolerance_ = 0.2;
  const double angle_tolerance_ = 0.1;
  std::shared_ptr<mesh_controller::MeshController> controller_;
  std::shared_ptr<mesh_map::MeshMap> mesh_map_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(MeshControllerTest, turnsInPlaceToTheGoalHeading)
{
  initController([](float) {return 0.0f;});
  ASSERT_TRUE(controller_->setPlan({pose(0.5, 0.5, 0, 0), pose(1.5, 0.5, 0, M_PI_2)}));
  geometry_msgs::msg::TwistStamped cmd_vel;

  EXPECT_FALSE(cycle(pose(0.8, 0.5, 0, 0), cmd_vel));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);

  EXPECT_FALSE(cycle(pose(1.45, 0.5, 0, 0), cmd_vel));
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GT(cmd_vel.twist.angular.z, 0.0);

  EXPECT_FALSE(cycle(pose(1.45, 0.5, 0, M_PI), cmd_vel));
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_LT(cmd_vel.twist.angular.z, 0.0);

  EXPECT_FALSE(cycle(pose(1.45, 0.5, 0, -M_PI_4), cmd_vel));
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GT(cmd_vel.twist.angular.z, 0.0);

  EXPECT_TRUE(cycle(pose(1.45, 0.5, 0, M_PI_2 - 0.05), cmd_vel));
}

TEST_F(MeshControllerTest, reachesTheGoalHeadingOnASlope)
{
  const float slope = 0.3;
  initController([slope](float x) {return slope * x;});
  ASSERT_TRUE(controller_->setPlan({pose(0.5, 0.5, 0.15, 0), pose(1.5, 0.5, 0.45, 0)}));
  geometry_msgs::msg::TwistStamped cmd_vel;

  auto robot_pose = pose(1.45, 0.5, 0.435, 0);
  const double pitch = -std::atan(slope);
  robot_pose.pose.orientation.x = 0;
  robot_pose.pose.orientation.y = std::sin(pitch / 2);
  robot_pose.pose.orientation.z = 0;
  robot_pose.pose.orientation.w = std::cos(pitch / 2);
  EXPECT_TRUE(cycle(robot_pose, cmd_vel));
}

TEST_F(MeshControllerTest, stopsBeforeLethalVerticesAhead)
{
  initController([](float x) {return x >= 2.0f && x <= 2.2f ? 0.3f : 0.0f;});
  float lethal_x = std::numeric_limits<float>::infinity();
  const auto mesh = mesh_map_ptr_->mesh();
  for (const auto vH : mesh->vertices()) {
    const auto & position = mesh->getVertexPosition(vH);
    if (std::fabs(position.y - 0.5) < 0.01 && !(mesh_map_ptr_->vertexCosts()[vH] < 1.0)) {
      lethal_x = std::min(lethal_x, position.x);
    }
  }
  ASSERT_LT(lethal_x, 2.0);
  ASSERT_GT(lethal_x, 1.5);
  ASSERT_TRUE(controller_->setPlan({pose(0.3, 0.5, 0, 0), pose(2.8, 0.5, 0, 0)}));
  geometry_msgs::msg::TwistStamped cmd_vel;

  EXPECT_FALSE(cycle(pose(lethal_x - 1.2, 0.5, 0, 0), cmd_vel));
  const double free_velocity = cmd_vel.twist.linear.x;
  EXPECT_GT(free_velocity, 0.0);

  EXPECT_FALSE(cycle(pose(lethal_x - 0.65, 0.5, 0, 0), cmd_vel));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_LT(cmd_vel.twist.linear.x, free_velocity);

  EXPECT_FALSE(cycle(pose(lethal_x - 0.45, 0.5, 0, 0), cmd_vel));
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
}

TEST_F(MeshControllerTest, fadesTheLinearVelocityNearTheGoal)
{
  initController([](float) {return 0.0f;});
  ASSERT_TRUE(controller_->setPlan({pose(0.3, 0.5, 0, 0), pose(2.0, 0.5, 0, 0)}));
  geometry_msgs::msg::TwistStamped cmd_vel;

  EXPECT_FALSE(cycle(pose(1.0, 0.5, 0, 0), cmd_vel));
  const double free_velocity = cmd_vel.twist.linear.x;
  EXPECT_GT(free_velocity, 0.0);

  EXPECT_FALSE(cycle(pose(1.75, 0.5, 0, 0), cmd_vel));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_LT(cmd_vel.twist.linear.x, free_velocity);
}

TEST_F(MeshControllerTest, capsTheAngularVelocityInBothDirections)
{
  initController([](float) {return 0.0f;}, rclcpp::NodeOptions()
    .append_parameter_override("mesh_controller.ang_vel_factor", 4.0)
    .append_parameter_override("mesh_controller.max_ang_velocity", 0.5));
  ASSERT_TRUE(controller_->setPlan({pose(0.3, 0.5, 0, 0), pose(2.8, 0.5, 0, 0)}));
  geometry_msgs::msg::TwistStamped cmd_vel;

  EXPECT_FALSE(cycle(pose(1.0, 0.5, 0, 3 * M_PI_4), cmd_vel));
  EXPECT_LT(cmd_vel.twist.angular.z, 0.0);
  EXPECT_GE(cmd_vel.twist.angular.z, -0.5);

  EXPECT_FALSE(cycle(pose(1.0, 0.5, 0, -3 * M_PI_4), cmd_vel));
  EXPECT_GT(cmd_vel.twist.angular.z, 0.0);
  EXPECT_LE(cmd_vel.twist.angular.z, 0.5);
}
