#include <gtest/gtest.h>
#include <cvp_mesh_planner/cvp_mesh_planner.h>
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

struct CVPMeshPlannerTest : public Test
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

  void initPlanner(const double step_width)
  {
    const auto dir = std::filesystem::temp_directory_path() / ("cvp_mesh_planner_test_" + std::to_string(getpid()));
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

    node_ptr_ = std::make_shared<rclcpp::Node>("cvp_mesh_planner", "test", rclcpp::NodeOptions()
      .append_parameter_override("mesh_map.mesh_file", mesh_file.string())
      .append_parameter_override("mesh_map.mesh_part", "/")
      .append_parameter_override("mesh_map.mesh_working_file", (dir / "grid.h5").string())
      .append_parameter_override("mesh_map.mesh_working_part", "mesh")
      .append_parameter_override("mesh_map.layers", std::vector<std::string>{"height_diff"})
      .append_parameter_override("mesh_map.default_layer", "height_diff")
      .append_parameter_override("mesh_map.height_diff.type", "mesh_layers/HeightDiffLayer")
      .append_parameter_override("cvp.step_width", step_width));
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    mesh_map_ptr_ = std::make_shared<mesh_map::MeshMap>(*tf_buffer_ptr_, node_ptr_);
    ASSERT_TRUE(mesh_map_ptr_->readMap());
    std::filesystem::remove_all(dir);

    planner_ = std::make_shared<cvp_mesh_planner::CVPMeshPlanner>();
    ASSERT_TRUE(planner_->initialize("cvp", mesh_map_ptr_, node_ptr_));
  }

  geometry_msgs::msg::PoseStamped pose(const double x, const double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = mesh_map_ptr_->mapFrame();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  std::shared_ptr<cvp_mesh_planner::CVPMeshPlanner> planner_;
  std::shared_ptr<mesh_map::MeshMap> mesh_map_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(CVPMeshPlannerTest, backTracksToTheGoalWithinTheStepWidth)
{
  const double step_width = 0.05;
  initPlanner(step_width);
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  double cost;
  std::string message;
  ASSERT_EQ(
    planner_->makePlan(pose(0.25, 0.5), pose(2.65, 0.5), 0.0, plan, cost, message),
    mbf_msgs::action::GetPath::Result::SUCCESS) << message;
  ASSERT_GE(plan.size(), 2u);
  const auto & last = plan[plan.size() - 2].pose.position;
  const auto & goal = plan.back().pose.position;
  EXPECT_LE(std::hypot(goal.x - last.x, goal.y - last.y, goal.z - last.z), step_width + 1e-3);
  for (size_t i = 1; i < plan.size(); i++) {
    const auto & a = plan[i - 1].pose.position;
    const auto & b = plan[i].pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y, b.z - a.z), step_width + 1e-3) << "pose " << i;
  }
}
