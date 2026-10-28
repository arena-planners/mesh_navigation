#include <gtest/gtest.h>
#include <dijkstra_mesh_planner/dijkstra_mesh_planner.h>
#include <mbf_msgs/action/get_path.hpp>
#include <mesh_map/mesh_map.h>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ::testing;

struct DijkstraMeshPlannerTest : public Test
{
protected:

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    planner_.reset();
    mesh_map_ptr_.reset();
    tf_buffer_ptr_.reset();
    node_ptr_.reset();
    rclcpp::shutdown();
  }

  void initPlanner()
  {
    const auto dir = std::filesystem::temp_directory_path() / ("dijkstra_mesh_planner_test_" + std::to_string(getpid()));
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
          ply << column * 0.1 << " " << row * 0.1 << " 0\n";
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

    node_ptr_ = std::make_shared<rclcpp::Node>("dijkstra_mesh_planner", "test", rclcpp::NodeOptions()
      .append_parameter_override("mesh_map.mesh_file", mesh_file.string())
      .append_parameter_override("mesh_map.mesh_part", "/")
      .append_parameter_override("mesh_map.mesh_working_file", (dir / "grid.h5").string())
      .append_parameter_override("mesh_map.mesh_working_part", "mesh")
      .append_parameter_override("mesh_map.layers", std::vector<std::string>{"height_diff"})
      .append_parameter_override("mesh_map.default_layer", "height_diff")
      .append_parameter_override("mesh_map.height_diff.type", "mesh_layers/HeightDiffLayer"));
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    mesh_map_ptr_ = std::make_shared<mesh_map::MeshMap>(*tf_buffer_ptr_, node_ptr_);
    ASSERT_TRUE(mesh_map_ptr_->readMap());
    std::filesystem::remove_all(dir);

    planner_ = std::make_shared<dijkstra_mesh_planner::DijkstraMeshPlanner>();
    ASSERT_TRUE(planner_->initialize("dijkstra", mesh_map_ptr_, node_ptr_));
  }

  geometry_msgs::msg::PoseStamped pose(const double x, const double y, const double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = mesh_map_ptr_->mapFrame();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    return pose;
  }

  void expectEndsAtTheGoal(const double start_x, const double goal_x)
  {
    const auto goal = pose(goal_x, 0.5, -M_PI_2);
    std::vector<geometry_msgs::msg::PoseStamped> plan;
    double cost;
    std::string message;
    ASSERT_EQ(
      planner_->makePlan(pose(start_x, 0.5, 0), goal, 0.0, plan, cost, message),
      mbf_msgs::action::GetPath::Result::SUCCESS) << message;
    ASSERT_FALSE(plan.empty()) << "start " << start_x << ", goal " << goal_x;
    EXPECT_EQ(plan.back().pose, goal.pose) << "start " << start_x << ", goal " << goal_x;
  }

  std::shared_ptr<dijkstra_mesh_planner::DijkstraMeshPlanner> planner_;
  std::shared_ptr<mesh_map::MeshMap> mesh_map_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(DijkstraMeshPlannerTest, endsThePlanWithTheGoalPose)
{
  initPlanner();
  expectEndsAtTheGoal(0.25, 2.62);
}

TEST_F(DijkstraMeshPlannerTest, plansToAGoalNearTheStart)
{
  initPlanner();
  expectEndsAtTheGoal(2.58, 2.62);
  expectEndsAtTheGoal(2.5, 2.62);
  expectEndsAtTheGoal(2.35, 2.62);
}
