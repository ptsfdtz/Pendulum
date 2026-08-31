#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/config/Config.h"
#include "pendulum/safety/SafetyManager.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testDefaultConfig(const std::filesystem::path& path) {
    const auto config = pendulum::config::AppConfig::load(path);
    require(config.schemaVersion == 1, "schema version mismatch");
    require(config.ni6602.device == "Dev1", "NI device mismatch");
    require(config.pci1723.deviceDescription == "PCI-1723,BID#15",
            "Advantech device mismatch");
    require(config.pci1723.positiveVoltageCartDirection == "LEFT",
            "positive voltage direction mismatch");
    require(config.pci1723.negativeVoltageCartDirection == "RIGHT",
            "negative voltage direction mismatch");

    bool rejected = false;
    try {
        config.validateForSafeOutputTest();
    } catch (const std::exception&) {
        rejected = true;
    }
    require(rejected, "default config must reject safe output test");
    config.validateForManualConsole();
}

void testSafetyOrderAndFaultContainment() {
    std::vector<std::string> calls;
    pendulum::safety::SafetyManager manager;
    auto servo = manager.registerAction(10, "servo", [&] { calls.emplace_back("servo"); });
    auto broken = manager.registerAction(5, "broken", [] { throw std::runtime_error("test"); });
    auto analog = manager.registerAction(0, "analog", [&] { calls.emplace_back("analog"); });

    manager.emergencyStop("unit test");
    require(manager.stopRequested(), "emergency stop flag was not set");
    require(calls.size() == 2, "not all non-throwing safety actions ran");
    require(calls[0] == "analog" && calls[1] == "servo", "safety priority order is wrong");
}

void testEncoderRolloverAndCalibration() {
    using pendulum::calibration::MotorEncoderCalibration;
    require(MotorEncoderCalibration::deltaWithRollover(0xFFFFFFFAU, 10U) == 16,
            "positive rollover delta is wrong");
    require(MotorEncoderCalibration::deltaWithRollover(10U, 0xFFFFFFFAU) == -16,
            "negative rollover delta is wrong");
    require(MotorEncoderCalibration::deltaWithRollover(100U, 350U) == 250,
            "ordinary positive delta is wrong");

    const std::vector<std::uint32_t> rolloverSamples{
        0xFFFFFFFEU, 0xFFFFFFFFU, 0U, 1U, 0U};
    require(MotorEncoderCalibration::stableRepresentative(rolloverSamples, 3) == 0U,
            "rollover-safe representative is wrong");

    const auto result = MotorEncoderCalibration::calculate(1000U, 81000U, 100.0);
    require(result.signedDeltaCounts == 80000, "calibration delta is wrong");
    require(result.countsPerMillimeter == 800.0, "counts_per_mm calculation is wrong");
}

void testAtomicCalibrationPersistence(const std::filesystem::path& sourceConfig) {
    const auto unique = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto directory =
        std::filesystem::temp_directory_path() / ("PendulumLabConfigTest_" + unique);
    std::filesystem::create_directories(directory);
    const auto testConfig = directory / "config.json";
    std::filesystem::copy_file(sourceConfig, testConfig);

    try {
        pendulum::config::AppConfig::saveMotorEncoderCalibration(
            testConfig,
            pendulum::config::MotorEncoderCalibrationRecord{100U, 8100U, 8000, 100.0, 80.0});
        const auto loaded = pendulum::config::AppConfig::load(testConfig);
        require(loaded.ni6602.motorEncoderCountsPerMillimeter.has_value(),
                "saved counts_per_mm is missing");
        require(*loaded.ni6602.motorEncoderCountsPerMillimeter == 80.0,
                "saved counts_per_mm is wrong");

        bool backupFound = false;
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (entry.path().filename().string().starts_with("config.json.backup_")) {
                backupFound = true;
            }
        }
        require(backupFound, "atomic config update did not retain a backup");
    } catch (...) {
        std::filesystem::remove_all(directory);
        throw;
    }
    std::filesystem::remove_all(directory);
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Expected config path\n";
        return 2;
    }
    try {
        testDefaultConfig(argv[1]);
        testSafetyOrderAndFaultContainment();
        testEncoderRolloverAndCalibration();
        testAtomicCalibrationPersistence(argv[1]);
        std::cout << "Phase 1 tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
}
