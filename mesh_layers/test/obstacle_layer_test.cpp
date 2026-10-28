#include <gtest/gtest.h>
#include <optional>
#include <rclcpp/qos.hpp>
#include <string>

namespace mesh_layers
{
std::optional<rclcpp::QoS> get_qos_profile_from_string(const std::string& str);
}

TEST(ObstacleLayer, test_qos_profile_from_string)
{
    const auto reliable = mesh_layers::get_qos_profile_from_string("Reliable");
    ASSERT_TRUE(reliable);
    EXPECT_EQ(reliable->reliability(), rclcpp::ReliabilityPolicy::Reliable);

    const auto best_effort = mesh_layers::get_qos_profile_from_string("BestEffort");
    ASSERT_TRUE(best_effort);
    EXPECT_EQ(best_effort->reliability(), rclcpp::ReliabilityPolicy::BestEffort);

    EXPECT_FALSE(mesh_layers::get_qos_profile_from_string("Unknown"));
}
