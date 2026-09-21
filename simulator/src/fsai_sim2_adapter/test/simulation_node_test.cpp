#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <thread>
#include <vector>
#include <numbers>
#include <json/json.h>

#include <gtest/gtest.h>

#include "fsai_sim2_adapter/simulation_node.hpp"

using namespace std::chrono_literals;

namespace {
const auto kBringup = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
  "fsai_bringup";

class PhysicalNodeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.parameter_overrides({
      {"core_type", "fsai"},
      {"core_params", (kBringup / "vehicles/reference_bicycle").string()},
      {"track", (kBringup / "tracks/skidpad_small").string()},
      {"scenario", (kBringup / "scenarios/manual.yaml").string()},
      {"enable_timer", false}, {"shutdown_on_finish", false},
    });
    sim = std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>(options);
    sim->InitialisePlugins();
    peer = std::make_shared<rclcpp::Node>("physical_test_client", rclcpp::NodeOptions().use_intra_process_comms(true));
    sub = peer->create_subscription<fsai_interfaces::msg::ActuatorState>(
      "/fsai/actuator_state", 100,
      [this](const fsai_interfaces::msg::ActuatorState &message) { states.push_back(message); });
    state_sub = peer->create_subscription<std_msgs::msg::String>(
      "/sim/state/as_state", 10,
      [this](const std_msgs::msg::String &message) { safety = message.data; });
    odom_sub = peer->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 100,
      [this](const nav_msgs::msg::Odometry &message) { odometry = message; });
    command = peer->create_publisher<fsai_interfaces::msg::ActuationCommand>("/fsai/actuation_command", 10);
    executor.add_node(sim);
    executor.add_node(peer);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (command->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    ASSERT_GT(command->get_subscription_count(), 0u);
    Step();
    ASSERT_FALSE(states.empty());
  }

  void TearDown() override {
    if (sim) { executor.remove_node(sim); }
    executor.remove_node(peer);
  }

  void Step(int count = 1) {
    for (int i = 0; i < count; ++i) {
      sim->AdvanceOneStep();
      executor.spin_some();
    }
  }

  bool Trigger(const std::string &name) {
    auto client = peer->create_client<std_srvs::srv::Trigger>(name);
    if (!client->wait_for_service(2s)) { return false; }
    auto result = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (executor.spin_until_future_complete(result, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    return result.get()->success;
  }

  void Arm() {
    auto client = peer->create_client<eufs_msgs::srv::SetMission>("/set_mission");
    ASSERT_TRUE(client->wait_for_service(2s));
    auto request = std::make_shared<eufs_msgs::srv::SetMission::Request>();
    request->mission = request->MANUAL_DRIVING;
    auto result = client->async_send_request(request);
    ASSERT_EQ(executor.spin_until_future_complete(result, 2s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(result.get()->success);
    EXPECT_FALSE(Trigger("/go"));
    Step(1000);
    ASSERT_TRUE(Trigger("/go"));
  }

  void Send(std::uint64_t sequence, double torque = 80.0, double brake = 0.0,
    std::int32_t seconds_offset = 0, double steering = 0.0) {
    fsai_interfaces::msg::ActuationCommand message;
    message.stamp = states.back().stamp;
    message.stamp.sec += seconds_offset;
    message.sequence = sequence;
    message.rear_axle_torque_nm = torque;
    message.friction_brake_ratio = brake;
    message.steering_angle_rad = steering;
    command->publish(message);
    executor.spin_some();
  }

  std::shared_ptr<fsai::sim2_adapter::FsaiSimulationNode> sim;
  rclcpp::Node::SharedPtr peer;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Subscription<fsai_interfaces::msg::ActuatorState>::SharedPtr sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<fsai_interfaces::msg::ActuationCommand>::SharedPtr command;
  std::vector<fsai_interfaces::msg::ActuatorState> states;
  nav_msgs::msg::Odometry odometry;
  std::string safety;
};
}  // namespace

TEST_F(PhysicalNodeTest, OffReadyDrivingWatchdogAndEbsAreConnected) {
  Send(0);
  Step();
  EXPECT_DOUBLE_EQ(states.back().u_mps, 0.0);
  EXPECT_EQ(safety, "OFF");
  Arm();
  Send(1);
  Step();
  EXPECT_EQ(safety, "DRIVING");
  EXPECT_GT(states.back().u_mps, 0.0);
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 80.0);
  Step(25);
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(states.back().friction_brake_ratio, 0.5);
  ASSERT_TRUE(Trigger("/ebs"));
  Send(2);
  Step();
  EXPECT_EQ(safety, "EMERGENCY_BRAKE");
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(states.back().friction_brake_ratio, 1.0);
  EXPECT_FALSE(Trigger("/go"));
}

TEST_F(PhysicalNodeTest, OldFutureAndDuplicateCommandsDoNotReplaceFreshBrake) {
  Arm();
  Send(10, 0.0, 0.75);
  Step();
  Send(9);
  Step();
  Send(10);
  Step();
  Send(11, 80.0, 0.0, -1);
  Step();
  Send(12, 80.0, 0.0, 1);
  Step();
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(states.back().friction_brake_ratio, 0.75);
  Send(11);
  Step();
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 80.0);
}

TEST_F(PhysicalNodeTest, ResetRestoresPoseTimeAndRequiresRearming) {
  sensor_msgs::msg::JointState joints;
  auto joint_sub = peer->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&joints](const sensor_msgs::msg::JointState &message) { joints = message; });
  eufs_msgs::msg::ConeArrayWithCovariance cones;
  auto cone_sub = peer->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    "/sensors/camera/cones", 10,
    [&cones](const eufs_msgs::msg::ConeArrayWithCovariance &message) { cones = message; });
  ASSERT_TRUE(Trigger("/reset"));
  executor.spin_some();
  const auto initial_cones = cones;
  Arm();
  Send(1, 80.0, 0.0, 0, 0.1);
  Step(10);
  ASSERT_EQ(joints.name.size(), 6u);
  EXPECT_EQ(joints.name[0], "steer_fl");
  EXPECT_EQ(joints.name[5], "spin_rr");
  EXPECT_GT(joints.position[0], joints.position[1]);
  EXPECT_GT(joints.position[1], 0.0);
  EXPECT_GT(joints.position[4], 0.0);
  ASSERT_TRUE(Trigger("/ebs"));
  Step();
  ASSERT_TRUE(Trigger("/reset"));
  executor.spin_some();
  EXPECT_EQ(states.back().stamp.sec, 0);
  EXPECT_EQ(states.back().stamp.nanosec, 0u);
  EXPECT_DOUBLE_EQ(states.back().u_mps, 0.0);
  EXPECT_EQ(safety, "OFF");
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, 0.0);
  EXPECT_NEAR(odometry.pose.pose.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_EQ(cones, initial_cones);
  ASSERT_EQ(joints.position.size(), 6u);
  for (const double angle : joints.position) { EXPECT_DOUBLE_EQ(angle, 0.0); }
  Send(0);
  Step();
  EXPECT_DOUBLE_EQ(states.back().rear_axle_torque_nm, 0.0);
}

namespace {
class ReferenceRuntimeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() { int argc = 0; rclcpp::init(argc, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    directory = std::filesystem::temp_directory_path() /
      ("fsai-reference-runtime-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ASSERT_TRUE(std::filesystem::create_directory(directory));
    std::filesystem::copy_file(kBringup / "tracks/skidpad_small/cones.csv", directory / "cones.csv");
    std::ofstream(directory / "track.yaml") <<
      "schema_version: 1\nname: algorithm_circle_fixture\nframe_id: map\n"
      "vehicle_start: {x_m: 12.0, y_m: 0.0, yaw_rad: 1.5707963267948966}\n"
      "start_gate: {x_m: 12.0, y_m: 0.0, yaw_rad: 1.5707963267948966, width_m: 3.5}\n"
      "finish_gate: {x_m: 12.0, y_m: 0.0, yaw_rad: 1.5707963267948966, width_m: 3.5}\n";
    std::ofstream route(directory / "control_centerline.csv");
    route << "x_m,y_m,width_m\n" << std::setprecision(17);
    for (int i = 0; i < 240; ++i) {
      const double angle = i * 2.0 * std::numbers::pi / 240.0;
      route << 12 * std::cos(angle) << ',' << 12 * std::sin(angle) << ",3.5\n";
    }
    WriteScenario(400);
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  void WriteScenario(int seconds) {
    std::ofstream(directory / "scenario.yaml") <<
      "schema_version: 1\nname: algorithm_ten_laps\nmission: track_drive\nvehicle_profile: reference_bicycle\n"
      "track_bundle: algorithm_circle_fixture\nseed: 1\nmode: as_fast_as_possible\n"
      "plant_step_ms: 1\nouter_step_ms: 5\nauto_start: true\nreference_driver: true\n"
      "target_laps: 10\ncruise_speed_mps: 2.5\nduration_limit_s: " << seconds << '\n';
  }
  rclcpp::NodeOptions Options() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"core_params", (kBringup / "vehicles/reference_bicycle").string()},
      {"track", directory.string()}, {"scenario", (directory / "scenario.yaml").string()},
      {"report_path", (directory / "result.json").string()},
      {"enable_timer", false}, {"shutdown_on_finish", false},
    });
    return options;
  }
  Json::Value Report() {
    std::ifstream input(directory / "result.json");
    Json::Value report;
    input >> report;
    return report;
  }
  std::filesystem::path directory;
};
}

TEST_F(ReferenceRuntimeTest, TenLapsFinishOnlyAfterStoppingAndWriteExclusiveReport) {
  auto sim = std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>(Options());
  for (int i = 0; i < 80000 && !std::filesystem::exists(directory / "result.json"); ++i) {
    ASSERT_NO_THROW(sim->AdvanceOneStep());
  }
  ASSERT_TRUE(std::filesystem::exists(directory / "result.json"));
  const auto report = Report();
  EXPECT_EQ(report["outcome"].asString(), "complete");
  EXPECT_EQ(report["completed_laps"].asUInt(), 10u);
  EXPECT_EQ(report["target_laps"].asUInt(), 10u);
  EXPECT_EQ(report["control_centerline_sha256"].asString().size(), 64u);
  EXPECT_GT(report["min_clearance_m"].asDouble(), 0.0);
  EXPECT_NEAR(report["final_speed_mps"].asDouble(), 0.0, 0.02);
  EXPECT_GT(report["simulation_time_s"].asDouble(), 300.0);
  EXPECT_THROW(std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>(Options()), std::exception);
  EXPECT_EQ(Report(), report) << "An existing report must not be overwritten";
}

TEST_F(ReferenceRuntimeTest, TimeoutWritesFailedReportAndThrowsInsteadOfSuccessfulFinish) {
  WriteScenario(6);
  auto sim = std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>(Options());
  bool failed = false;
  for (int i = 0; i < 2400; ++i) {
    try { sim->AdvanceOneStep(); }
    catch (const std::runtime_error &) { failed = true; break; }
  }
  EXPECT_TRUE(failed);
  ASSERT_TRUE(std::filesystem::exists(directory / "result.json"));
  const auto report = Report();
  EXPECT_EQ(report["outcome"].asString(), "failed");
  EXPECT_LT(report["completed_laps"].asUInt(), 10u);
  EXPECT_NEAR(report["final_speed_mps"].asDouble(), 0.0, 0.02);
}

TEST_F(PhysicalNodeTest, RealtimeAndFastModesProduceTheSameSeededScenario) {
  // Release the fixture's manual scenario before exercising both real scenario
  // configurations with identical deterministic stepping through the ROS node.
  executor.remove_node(sim);
  sim.reset();
  auto run = [&](const std::string &mode) {
    // Each experiment owns its subscriptions/topics. Reusing the previous DDS
    // reader admits its in-flight final sample into the next experiment.
    std::vector<fsai_interfaces::msg::ActuatorState> trace;
    std::vector<eufs_msgs::msg::ConeArrayWithCovariance> cones;
    const auto state_topic = "/trace/" + mode + "/actuators";
    const auto cone_topic = "/trace/" + mode + "/cones";
    auto trace_sub = peer->create_subscription<fsai_interfaces::msg::ActuatorState>(
      state_topic, 100,
      [&trace](const fsai_interfaces::msg::ActuatorState &message) {
        // Constructor-time t=0 publication precedes graph discovery; compare
        // every accepted integration step, not that optional initial delivery.
        if (message.stamp.sec != 0 || message.stamp.nanosec != 0) { trace.push_back(message); }
      });
    auto cone_sub = peer->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      cone_topic, 100,
      [&cones](const eufs_msgs::msg::ConeArrayWithCovariance &message) {
        if (message.header.stamp.sec != 0 || message.header.stamp.nanosec != 0) {
          cones.push_back(message);
        }
      });
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments({"--ros-args", "-r", "/fsai/actuator_state:=" + state_topic,
      "-r", "/sensors/camera/cones:=" + cone_topic});
    options.parameter_overrides({
      {"core_type", "fsai"},
      {"core_params", (kBringup / "vehicles/ads_dv").string()},
      {"track", (kBringup / "tracks/skidpad_small").string()},
      {"scenario", (kBringup / "scenarios/straight_acceleration.yaml").string()},
      {"enable_timer", false}, {"shutdown_on_finish", false}, {"run_mode", mode},
    });
    sim = std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>(options);
    executor.add_node(sim);
    Step(4000);
    // spin_some processes the ready set it collected at entry. Drain the
    // final published samples before taking the trace or destroying its writer.
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ((trace.size() < 4000 || cones.size() < 400 || safety != "FINISHED") &&
        std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    const auto result = std::make_pair(trace, cones);
    executor.remove_node(sim);
    sim.reset();
    return result;
  };
  const auto realtime = run("realtime");
  const auto fast = run("as_fast_as_possible");
  ASSERT_EQ(realtime.first.size(), 4000u);
  ASSERT_EQ(fast.first.size(), realtime.first.size());
  for (std::size_t i = 0; i < fast.first.size(); ++i) {
    EXPECT_EQ(realtime.first[i].stamp.sec * 1000000000LL + realtime.first[i].stamp.nanosec,
      static_cast<std::int64_t>(i + 1) * 5000000LL);
    ASSERT_EQ(realtime.first[i], fast.first[i]) << "actuator sample " << i;
  }
  ASSERT_EQ(realtime.second.size(), 400u);
  ASSERT_EQ(fast.second.size(), realtime.second.size());
  for (std::size_t i = 0; i < fast.second.size(); ++i) {
    ASSERT_EQ(realtime.second[i], fast.second[i]) << "camera sample " << i;
  }
  EXPECT_EQ(fast.first.back().stamp.sec, 20);
  EXPECT_EQ(fast.first.back().stamp.nanosec, 0u);
  EXPECT_EQ(safety, "FINISHED");
  EXPECT_GT(odometry.pose.pose.position.y, 1.0);
}
