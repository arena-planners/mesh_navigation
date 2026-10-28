#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <mesh_map/mesh_map.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ::testing;

struct MeshMapTest : public Test
{
protected:

  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  // Call this manually at the beginning of each test.
  // Allows setting parameter overrides via NodeOptions (mirrors behavior of how parameters are loaded from yaml via launch file for example)
  void initNodeAndMeshMap(const rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions()) {
    node_ptr_ = std::make_shared<rclcpp::Node>("mesh_map", "test", nodeOptions);
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
    mesh_map_ptr_ = std::make_shared<mesh_map::MeshMap>(
      *tf_buffer_ptr_, node_ptr_);
  }

  void TearDown() override {
    rclcpp::shutdown();
    mesh_map_ptr_.reset();
    tf_buffer_ptr_.reset();
    node_ptr_.reset();
  }

  void loadStairs(const std::vector<float> & heights)
  {
    const auto dir = std::filesystem::temp_directory_path() / ("mesh_map_test_" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const auto mesh_file = dir / "stairs.ply";
    const size_t columns = heights.size();
    const size_t rows = 3;
    {
      std::ofstream ply(mesh_file);
      ply << "ply\nformat ascii 1.0\nelement vertex " << columns * rows
          << "\nproperty float x\nproperty float y\nproperty float z\nelement face " << 2 * (columns - 1) * (rows - 1)
          << "\nproperty list uchar int vertex_indices\nend_header\n";
      for (size_t row = 0; row < rows; row++) {
        for (size_t column = 0; column < columns; column++) {
          ply << column * spacing_ << " " << row * spacing_ << " " << heights[column] << "\n";
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

    const std::vector<std::string> layer_names{"test_layer"};
    initNodeAndMeshMap(rclcpp::NodeOptions()
      .append_parameter_override("mesh_map.mesh_file", mesh_file.string())
      .append_parameter_override("mesh_map.mesh_part", "/")
      .append_parameter_override("mesh_map.mesh_working_file", (dir / "stairs.h5").string())
      .append_parameter_override("mesh_map.mesh_working_part", "mesh")
      .append_parameter_override("mesh_map.layers", layer_names)
      .append_parameter_override("mesh_map.default_layer", "test_layer")
      .append_parameter_override("mesh_map.test_layer.type", "mesh_map/TestLayer")
    );
    ASSERT_TRUE(mesh_map_ptr_->readMap());
    std::filesystem::remove_all(dir);

    const auto mesh = mesh_map_ptr_->mesh();
    lvr2::DenseVertexMap<mesh_map::Vector> vector_map(mesh->nextVertexIndex(), mesh_map::Vector());
    for (const auto vH : mesh->vertices()) {
      const auto & position = mesh->getVertexPosition(vH);
      const size_t column = std::min(static_cast<size_t>(std::lround(position.x / spacing_)), columns - 2);
      vector_map[vH] = mesh_map::Vector(spacing_, 0, heights[column + 1] - heights[column]).normalized();
    }
    mesh_map_ptr_->setVectorMap(vector_map);
  }

  mesh_map::Vector followVectorField(const std::vector<float> & heights, const float step_width)
  {
    mesh_map::Vector position(spacing_ / 2, spacing_, heights.front());
    auto face = mesh_map_ptr_->getContainingFace(position, 0.1);
    EXPECT_TRUE(face);
    if (!face) {
      return position;
    }
    lvr2::FaceHandle current_face = face.unwrap();
    const float end = (heights.size() - 2) * spacing_;
    for (int step = 0; step < 1000 && position.x < end; step++) {
      if (!mesh_map_ptr_->meshAhead(position, current_face, step_width)) {
        ADD_FAILURE() << "meshAhead failed at (" << position.x << ", " << position.y << ", " << position.z << ")";
        break;
      }
    }
    return position;
  }

  const float spacing_ = 0.1;
  std::shared_ptr<mesh_map::MeshMap> mesh_map_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  rclcpp::Node::SharedPtr node_ptr_;
};

TEST_F(MeshMapTest, loadsSinglePlugin)
{
  const std::vector<std::string> layer_names{"test_layer"};
  initNodeAndMeshMap(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.default_layer", "test_layer")
    .append_parameter_override("mesh_map.test_layer.type", "mesh_map/TestLayer")
  );
  // The MeshMap requires that a map file is loaded to initialize, so we test the layer manager directly
  mesh_map::LayerManager manager(*mesh_map_ptr_, node_ptr_);

  EXPECT_NO_THROW(manager.read_configured_layers());
  EXPECT_TRUE(manager.load_layer_plugins(node_ptr_->get_logger()));
  // This calls back to the mesh map and segfaults because the MeshMap has its own internal LayerManager :(
  // EXPECT_TRUE(manager.initialize_layer_plugins(node_ptr_, mesh_map_ptr_));

  EXPECT_NE(manager.get_layer("test_layer"), nullptr);
}

TEST_F(MeshMapTest, loadsMultiplePlugins)
{
  const std::vector<std::string> layer_names{"t3", "t1", "t2"};
  initNodeAndMeshMap(rclcpp::NodeOptions()
    .append_parameter_override("mesh_map.layers", layer_names)
    .append_parameter_override("mesh_map.default_layer", "t3")
    .append_parameter_override("mesh_map.t1.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t2.type", "mesh_map/TestLayer")
    .append_parameter_override("mesh_map.t3.type", "mesh_map/TestLayer")
  );
  // The MeshMap requires that a map file is loaded to initialize, so we test the layer manager directly
  mesh_map::LayerManager manager(*mesh_map_ptr_, node_ptr_);

  EXPECT_NO_THROW(manager.read_configured_layers());
  EXPECT_TRUE(manager.load_layer_plugins(node_ptr_->get_logger()));

  EXPECT_NE(manager.get_layer("t1"), nullptr);
  EXPECT_NE(manager.get_layer("t2"), nullptr);
  EXPECT_NE(manager.get_layer("t3"), nullptr);
}

TEST_F(MeshMapTest, meshAheadClimbsStairs)
{
  const std::vector<float> heights{0, 0, 0, 0, 0, 0, 0.167, 0.167, 0.333, 0.333, 0.333, 0.5, 0.5, 0.667, 0.667, 0.667};
  loadStairs(heights);
  for (const float step_width : {0.03f, 0.05f}) {
    const auto position = followVectorField(heights, step_width);
    EXPECT_GE(position.x, (heights.size() - 2) * spacing_) << "step width " << step_width;
    EXPECT_NEAR(position.z, heights.back(), 0.01) << "step width " << step_width;
  }
}

TEST_F(MeshMapTest, meshAheadDescendsStairs)
{
  const std::vector<float> heights{0.667, 0.667, 0.667, 0.5, 0.5, 0.333, 0.333, 0.333, 0.167, 0.167, 0, 0, 0, 0, 0, 0};
  loadStairs(heights);
  for (const float step_width : {0.03f, 0.05f}) {
    const auto position = followVectorField(heights, step_width);
    EXPECT_GE(position.x, (heights.size() - 2) * spacing_) << "step width " << step_width;
    EXPECT_NEAR(position.z, heights.back(), 0.01) << "step width " << step_width;
  }
}
