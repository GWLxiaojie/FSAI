#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "fsai_sim2_adapter/vehicle_profile_loader.hpp"

namespace {
namespace fs = std::filesystem;
const fs::path profiles = fs::path(__FILE__).parent_path().parent_path().parent_path() /
  "fsai_bringup/vehicles";

class VehicleProfileLoader : public ::testing::Test {
 protected:
  fs::path directory;

  void SetUp() override {
    auto pattern = (fs::temp_directory_path() / "fsai-profile-test-XXXXXX").string();
    const auto created = mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    directory = created;
    for (const auto &entry : fs::directory_iterator(profiles / "ads_dv")) {
      fs::copy_file(entry.path(), directory / entry.path().filename());
    }
  }

  void TearDown() override {
    if (!directory.empty()) fs::remove_all(directory);
  }

  void Save(const char *file, const YAML::Node &node) {
    std::ofstream stream(directory / file);
    stream << node;
    ASSERT_TRUE(stream.good());
  }

  auto Load() { return fsai::sim2_adapter::LoadVehicleProfile(directory); }
};

TEST_F(VehicleProfileLoader, RealProfilesLoadAndInterfaceTimeoutIsHonored) {
  for (const auto *name : {"ads_dv", "reference_bicycle"}) {
    const auto p = fsai::sim2_adapter::LoadVehicleProfile(profiles / name);
    EXPECT_DOUBLE_EQ(p.mass_kg, 300.0);
    EXPECT_DOUBLE_EQ(p.wheelbase_m, 1.53);
  }
  auto interfaces = YAML::LoadFile((directory / "interfaces.yaml").string());
  interfaces["command_timeout_s"] = .25;
  Save("interfaces.yaml", interfaces);
  EXPECT_EQ(Load().command_timeout, std::chrono::milliseconds(250));
}

TEST_F(VehicleProfileLoader, RejectsScalarEvenWhenNumeric) {
  auto root = YAML::LoadFile((directory / "vehicle.yaml").string());
  root["mass_kg"] = 300.;
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
}

TEST_F(VehicleProfileLoader, RejectsInvalidValueUnitAndMissingProvenance) {
  const auto original = YAML::LoadFile((directory / "vehicle.yaml").string());
  for (const double value : {-1., 900., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    auto root = YAML::Clone(original);
    root["mass_kg"]["value"] = value;
    Save("vehicle.yaml", root);
    EXPECT_THROW(Load(), std::exception);
  }
  for (const auto *key : {"value", "unit", "source", "calibration_status"}) {
    auto root = YAML::Clone(original);
    root["mass_kg"].remove(key);
    Save("vehicle.yaml", root);
    EXPECT_THROW(Load(), std::exception) << key;
  }
  auto root = YAML::Clone(original);
  root["mass_kg"]["unit"] = "lb";
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
}

TEST_F(VehicleProfileLoader, RejectsNonFiniteOrReversedBounds) {
  const auto original = YAML::LoadFile((directory / "vehicle.yaml").string());
  for (const auto *key : {"valid_min", "valid_max"}) {
    for (double value : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
      auto root = YAML::Clone(original);
      root["mass_kg"][key] = value;
      Save("vehicle.yaml", root);
      EXPECT_THROW(Load(), std::exception);
    }
  }
  auto root = YAML::Clone(original);
  root["mass_kg"]["valid_min"] = 800;
  root["mass_kg"]["valid_max"] = 50;
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
}

TEST_F(VehicleProfileLoader, RequiresKnownSchemaAndRejectsUnknownKeys) {
  const auto original = YAML::LoadFile((directory / "vehicle.yaml").string());
  auto root = YAML::Clone(original);
  root.remove("schema_version");
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
  root = YAML::Clone(original);
  root["schema_version"] = 2;
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
  root = YAML::Clone(original);
  root["mass_kgg"] = root["mass_kg"];
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
  root = YAML::Clone(original);
  root["mass_kg"]["valid_minn"] = 50.;
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
}

TEST_F(VehicleProfileLoader, RejectsMissingFilesFieldsAndDuplicateKeys) {
  auto root = YAML::LoadFile((directory / "vehicle.yaml").string());
  root.remove("mass_kg");
  Save("vehicle.yaml", root);
  EXPECT_THROW(Load(), std::exception);
  fs::copy_file(profiles / "ads_dv/vehicle.yaml", directory / "vehicle.yaml",
    fs::copy_options::overwrite_existing);
  {
    std::ofstream stream(directory / "vehicle.yaml", std::ios::app);
    stream << "\nschema_version: 1\n";
  }
  EXPECT_THROW(Load(), std::exception);
  fs::remove(directory / "aero.yaml");
  EXPECT_THROW(Load(), std::exception);
}
}  // namespace
