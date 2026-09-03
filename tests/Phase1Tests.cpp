#include "pendulum/calibration/HomeCenterController.h"
#include "pendulum/calibration/MotorEncoderCalibration.h"
#include "pendulum/calibration/MotorZeroCalibrator.h"
#include "pendulum/config/Config.h"
#include "pendulum/control/ReferenceLqrVelocityController.h"
#include "pendulum/safety/SafetyManager.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
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
    require(config.ni6602.pendulumCounter == "Dev1/ctr1",
            "pendulum counter mismatch");
    require(config.ni6602.pendulumEncoderDecoding == "X4",
            "pendulum encoder decoding mismatch");
    require(config.ni6602.pendulumEncoderFilterMinPulseWidthMicroseconds == 10.0,
            "pendulum encoder digital filter mismatch");
    require(config.balanceControl.downwardZeroCaptureSeconds == 1.5 &&
                config.balanceControl.downwardZeroSettleTimeoutSeconds == 20.0 &&
                config.balanceControl.downwardZeroMaximumSpanCounts == 8,
            "stable downward-zero capture settings mismatch");
    require(config.ni6602.motorEncoderFilterMinPulseWidthMicroseconds == 10.0,
            "motor encoder digital filter mismatch");
    require(config.pci1723.deviceDescription == "PCI-1723,BID#15",
            "Advantech device mismatch");
    require(config.pci1723.positiveVoltageCartDirection == "RIGHT",
            "positive voltage direction mismatch");
    require(config.pci1723.negativeVoltageCartDirection == "LEFT",
            "negative voltage direction mismatch");
    require(config.manualConsole.dashboardRefreshMilliseconds == 100,
            "dashboard refresh interval mismatch");
    require(config.homeCenter.escapeCounts == 200,
            "home-center escape distance mismatch");
    require(config.homeCenter.maximumTravelDisagreementFraction == 0.02,
            "home-center travel agreement threshold mismatch");
    require(config.homeCenter.searchVoltage == 0.20 &&
                config.homeCenter.travelVoltage == 0.20,
            "home-center search/traversal speed mismatch");
    require(config.homeCenter.centerFastVoltage == 0.12 &&
                config.homeCenter.centerMidVoltage == 0.075 &&
                config.homeCenter.centerSlowVoltage == 0.03,
            "home-center return speed profile mismatch");
    require(config.balanceControl.pendulumPulsesPerRevolution == 2000,
            "pendulum PPR mismatch");
    require(config.balanceControl.pendulumCountsPerRevolution == 8000,
            "pendulum X4 counts/rev mismatch");
    require(config.balanceControl.driveModel == "SGD7S-180A00A002",
            "servo drive model mismatch");
    require(config.balanceControl.driveControlMode == "ANALOG_VELOCITY",
            "servo drive control mode mismatch");
    require(config.balanceControl.frequencyHz == 100,
            "reference control frequency mismatch");
    require(std::abs(config.balanceControl.maximumBalanceAngleRadians -
                     std::numbers::pi / 6.0) < 1e-12,
            "manual-upright LQR region mismatch");
    require(config.balanceControl.maximumBalanceStartPositionFraction == 0.1 &&
                config.balanceControl.maximumBalancePositionFraction == 1.0,
            "cart safety envelope mismatch");

    bool rejected = false;
    try {
        config.validateForSafeOutputTest();
    } catch (const std::exception&) {
        rejected = true;
    }
    require(rejected, "default config must reject safe output test");
    config.validateForManualConsole();
}

void testReferenceLqrVelocityController() {
    using Controller = pendulum::control::ReferenceLqrVelocityController;
    Controller controller;
    controller.reset();

    const auto zero = controller.update(0, 0);
    require(zero.outputVoltage == 0.0 &&
                zero.velocityReferenceMetersPerSecond == 0.0,
            "reference controller zero-state output mismatch");

    const auto negativeAfterModelTick = controller.update(-1, 0);
    require(negativeAfterModelTick.velocityReferenceMetersPerSecond < 0.0 &&
                negativeAfterModelTick.outputVoltage < 0.0,
            "reference t=0 state update ordering mismatch");

    controller.reset();
    const auto firstNegativeAcceleration = controller.update(-1, 0);
    require(firstNegativeAcceleration.velocityReferenceMetersPerSecond == 0.0 &&
                firstNegativeAcceleration.outputVoltage == 0.0,
            "reference first-step clamp state mismatch");

    controller.reset();

    const auto onePendulumCount = controller.update(1, 0);
    const double theta = -2.0 * std::numbers::pi / 8000.0;
    const double omega = theta / 0.01;
    const double acceleration =
        std::clamp(-58.6 * theta - 10.69 * omega, -10.0, 10.0);
    const double velocityReference = 0.005 * acceleration;
    const double integralVoltage = 0.005 * velocityReference * 54.0;
    const double expectedVoltage =
        std::clamp(0.18 * velocityReference + integralVoltage, -1.0, 1.0);
    require(std::abs(onePendulumCount.pendulumAngleRadians - theta) < 1e-15 &&
                std::abs(onePendulumCount.accelerationCommandMetersPerSecondSquared -
                         acceleration) < 1e-12 &&
                std::abs(onePendulumCount.velocityReferenceMetersPerSecond -
                         velocityReference) < 1e-12 &&
                std::abs(onePendulumCount.outputVoltage - expectedVoltage) < 1e-12,
            "reference generated-code sequence mismatch");

    controller.reset();
    const auto cartCount = controller.update(0, 1);
    const double x = -0.163 / 8000.0;
    const double velocity = x / 0.01;
    require(std::abs(cartCount.cartPositionMeters - x) < 1e-15 &&
                std::abs(cartCount.cartVelocityMetersPerSecond - velocity) < 1e-15,
            "reference cart encoder sign or scaling mismatch");

    controller.reset();
    for (int index = 0; index < 200; ++index) {
        const auto saturated = controller.update(1000, 0);
        require(saturated.accelerationCommandMetersPerSecondSquared >= -10.0 &&
                    saturated.accelerationCommandMetersPerSecondSquared <= 10.0 &&
                    saturated.velocityReferenceMetersPerSecond >= -0.6 &&
                    saturated.velocityReferenceMetersPerSecond <= 0.6 &&
                    saturated.outputVoltage >= -1.0 &&
                    saturated.outputVoltage <= 1.0,
                "reference controller saturation mismatch");
    }

    controller.reset();
    const auto firstDownwardAuto = controller.update(-4000, 0, true);
    require(firstDownwardAuto.swingUpActive &&
                std::abs(firstDownwardAuto.swingUpAccelerationMetersPerSecondSquared +
                         5.0) < 1e-12 &&
                std::abs(firstDownwardAuto.accelerationCommandMetersPerSecondSquared +
                         5.0) < 1e-12 &&
                firstDownwardAuto.outputVoltage == 0.0,
            "reference first swing-up tick mismatch");

    const auto stationaryDownwardAuto = controller.update(-4000, 0, true);
    require(stationaryDownwardAuto.swingUpActive &&
                stationaryDownwardAuto.swingUpAccelerationMetersPerSecondSquared == 0.0,
            "reference stationary-down swing-up state mismatch");

    controller.reset();
    const auto nearUprightAuto = controller.update(100, 0, true);
    require(!nearUprightAuto.swingUpActive &&
                nearUprightAuto.accelerationCommandMetersPerSecondSquared ==
                    nearUprightAuto.lqrAccelerationMetersPerSecondSquared,
            "reference Swing_up-to-LQR switch mismatch");

    controller.reset();
    const auto manualOutsideRegion = controller.update(-4000, 0, false);
    require(!manualOutsideRegion.swingUpActive &&
                manualOutsideRegion.accelerationCommandMetersPerSecondSquared ==
                    manualOutsideRegion.lqrAccelerationMetersPerSecondSquared,
            "manual-upright branch must remain LQR-only");
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
    require(MotorEncoderCalibration::deltaWithRollover(0U, 4294964277U) == -3019,
            "unsigned representation of a negative encoder position is wrong");

    const std::vector<std::uint32_t> rolloverSamples{
        0xFFFFFFFEU, 0xFFFFFFFFU, 0U, 1U, 0U};
    require(MotorEncoderCalibration::stableRepresentative(rolloverSamples, 3) == 0U,
            "rollover-safe representative is wrong");

    const auto result = MotorEncoderCalibration::calculate(1000U, 81000U, 100.0);
    require(result.signedDeltaCounts == 80000, "calibration delta is wrong");
    require(result.countsPerMillimeter == 800.0, "counts_per_mm calculation is wrong");
}

void testMotorZeroSelectionMath() {
    using pendulum::calibration::MotorZeroCalibrator;
    require(MotorZeroCalibrator::median({3.0, 1.0, 2.0}) == 2.0,
            "odd median is wrong");
    require(MotorZeroCalibrator::median({4.0, 1.0, 3.0, 2.0}) == 2.5,
            "even median is wrong");

    const std::vector<double> voltages{-0.002, -0.001, 0.0, 0.001, 0.002};
    const std::vector<double> crossingSpeeds{-8.0, -4.0, 2.0, 5.0, 9.0};
    const double crossing = MotorZeroCalibrator::findZeroCrossing(
        voltages, crossingSpeeds);
    require(std::abs(crossing - (-0.0003333333333333333)) < 1e-12,
            "interpolated zero crossing is wrong");

    const std::vector<double> deadbandSpeeds{4.0, 1.0, -0.5, 1.5, 4.0};
    const auto deadband = MotorZeroCalibrator::findLongestDeadband(
        voltages, deadbandSpeeds, 2.0);
    require(deadband.found && deadband.low == -0.001 && deadband.high == 0.001,
            "longest deadband is wrong");
}

void testHomeCenterMath() {
    using pendulum::calibration::HomeCenterController;
    require(HomeCenterController::midpoint(-1000, 3000) == 1000,
            "home-center midpoint is wrong");
    require(HomeCenterController::midpoint(3001, -1000) == 1000,
            "reverse home-center midpoint is wrong");

    pendulum::config::HomeCenterConfig settings;
    settings.searchVoltage = 0.1;
    settings.fineVoltage = 0.02;
    settings.escapeVoltage = 0.03;
    settings.centerFastVoltage = 0.03;
    settings.centerMidVoltage = 0.02;
    settings.centerSlowVoltage = 0.015;
    settings.escapeCounts = 2;
    settings.minimumTravelCounts = 10;
    settings.centerToleranceFraction = 0.01;
    settings.minimumCenterToleranceCounts = 1;
    settings.searchTimeoutSeconds = 1.0;
    settings.backoffTimeoutSeconds = 1.0;
    settings.centerTimeoutSeconds = 1.0;
    settings.centerSettleMilliseconds = 1;
    settings.pollPeriodMilliseconds = 1;

    std::int64_t position = 0;
    double voltage = 0.0;
    bool firstBoundaryRefined = false;
    double maximumCommandAfterFirstBoundary = 0.0;
    bool sawKnownCenterRightRelease = false;
    HomeCenterController controller(
        settings, true,
        [&] {
            if (voltage > 0.0) {
                --position;
            } else if (voltage < 0.0) {
                ++position;
            }
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command, pendulum::calibration::LimitSide releaseSide) {
            voltage = command;
            sawKnownCenterRightRelease =
                sawKnownCenterRightRelease ||
                releaseSide == pendulum::calibration::LimitSide::Right;
        },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    const auto result = controller.run();
    require(result.leftBoundaryCounts == -10,
            "simulated left boundary is wrong");
    require(result.rightBoundaryCounts == 10,
            "simulated right boundary is wrong");
    require(result.travelCounts == 20 && result.centerCounts == 0,
            "simulated travel or center is wrong");
    require(result.forwardTravelCounts == 20 &&
                result.reverseTravelCounts == 20 &&
                result.travelDisagreementCounts == 0,
            "simulated round-trip verification is wrong");
    require(std::llabs(result.centerErrorCounts) <= 1,
            "simulated cart did not return to center");
    require(maximumCommandAfterFirstBoundary <= settings.escapeVoltage,
            "distance-critical homing reused the fast search voltage");

    position = 7;
    const auto returned = controller.returnToKnownCenter(0, 20, -1.0);
    require(std::llabs(returned) <= 1,
            "known-center return did not reuse the measured center");

    position = 10;
    voltage = 0.0;
    sawKnownCenterRightRelease = false;
    const auto returnedFromLimit =
        controller.returnToKnownCenter(0, 20, -1.0);
    require(std::llabs(returnedFromLimit) <= 1,
            "known-center return failed from an active limit");
    require(sawKnownCenterRightRelease,
            "known-center return did not declare the verified release side");

    position = -6;
    voltage = 0.0;
    HomeCenterController arbitraryStartController(
        settings, true,
        [&] {
            if (voltage > 0.0) {
                --position;
            } else if (voltage < 0.0) {
                ++position;
            }
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command,
            pendulum::calibration::LimitSide) { voltage = command; },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    const auto arbitraryStart = arbitraryStartController.run();
    require(arbitraryStart.leftBoundaryCounts == -10 &&
                arbitraryStart.rightBoundaryCounts == 10 &&
                arbitraryStart.travelCounts == 20 &&
                arbitraryStart.centerCounts == 0,
            "fresh two-limit centering failed from an arbitrary start");
    require(std::llabs(arbitraryStart.centerErrorCounts) <= 1,
            "fresh two-limit centering did not return the cart to center");

    auto invalidTravelSettings = settings;
    invalidTravelSettings.minimumTravelCounts = 25;
    position = 0;
    voltage = 0.0;
    bool invalidTravelRejected = false;
    HomeCenterController invalidTravelController(
        invalidTravelSettings, true,
        [&] {
            if (voltage > 0.0) {
                --position;
            } else if (voltage < 0.0) {
                ++position;
            }
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command, pendulum::calibration::LimitSide) {
            voltage = command;
            if (firstBoundaryRefined) {
                maximumCommandAfterFirstBoundary =
                    std::max(maximumCommandAfterFirstBoundary,
                             std::abs(command));
            }
        },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; },
        [&](const std::string& message) {
            if (message.starts_with("Refined")) {
                firstBoundaryRefined = true;
            }
        });
    try {
        static_cast<void>(invalidTravelController.run());
    } catch (const std::runtime_error&) {
        invalidTravelRejected = true;
    }
    require(invalidTravelRejected,
            "below-minimum measured travel was not rejected");
    require(position > -10 && position < 10 && voltage == 0.0,
            "travel rejection left the cart pressed against a limit");

    position = 4;
    voltage = 0.0;
    HomeCenterController travelMeasurementController(
        invalidTravelSettings, true,
        [&] {
            if (voltage > 0.0) {
                --position;
            } else if (voltage < 0.0) {
                ++position;
            }
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command,
            pendulum::calibration::LimitSide) { voltage = command; },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    const auto measuredTravel = travelMeasurementController.measureTravel();
    require(measuredTravel.travelCounts == 20 &&
                measuredTravel.leftBoundaryCounts == -10 &&
                measuredTravel.rightBoundaryCounts == 10,
            "measure-only command did not report relative two-limit travel");
    require(position > -10 && position < 10 && voltage == 0.0,
            "measure-only command left the cart pressed against a limit");

    std::int64_t physicalPosition = 0;
    std::int64_t distortedEncoderPosition = 0;
    voltage = 0.0;
    bool inconsistentTravelRejected = false;
    HomeCenterController inconsistentTravelController(
        settings, true,
        [&] {
            if (voltage > 0.0 && physicalPosition < 10) {
                ++physicalPosition;
                distortedEncoderPosition += 2;
            } else if (voltage < 0.0 && physicalPosition > -10) {
                --physicalPosition;
                --distortedEncoderPosition;
            }
            return pendulum::calibration::HomeCenterSample{
                distortedEncoderPosition, physicalPosition <= -10,
                physicalPosition >= 10};
        },
        [&](double command,
            pendulum::calibration::LimitSide) { voltage = command; },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    try {
        static_cast<void>(inconsistentTravelController.run());
    } catch (const std::runtime_error&) {
        inconsistentTravelRejected = true;
    }
    require(inconsistentTravelRejected,
            "direction-dependent encoder corruption was not rejected");
    require(physicalPosition > -10 && physicalPosition < 10 && voltage == 0.0,
            "round-trip rejection left the cart pressed against a limit");

    position = 0;
    voltage = 0.0;
    bool leftWasReached = false;
    bool rightWasReached = false;
    HomeCenterController reversedDirectionController(
        settings, true,
        [&] {
            // Deliberately contradict the configured +V -> LEFT mapping.
            if (voltage > 0.0) {
                ++position;
            } else if (voltage < 0.0) {
                --position;
            }
            leftWasReached = leftWasReached || position <= -10;
            rightWasReached = rightWasReached || position >= 10;
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command,
            pendulum::calibration::LimitSide) { voltage = command; },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    const auto reversed = reversedDirectionController.run();
    require(leftWasReached && rightWasReached,
            "homing did not require both limits with reversed direction");
    require(reversed.leftBoundaryCounts == -10 &&
                reversed.rightBoundaryCounts == 10,
            "reversed direction boundary discovery is wrong");
    require(std::llabs(reversed.centerErrorCounts) <= 1,
            "reversed direction homing did not return to center");

    auto activeLimitSettings = settings;
    activeLimitSettings.awayDirectionTestMilliseconds = 2;
    position = 10;
    voltage = 0.0;
    leftWasReached = false;
    rightWasReached = true;
    HomeCenterController activeLimitController(
        activeLimitSettings, true,
        [&] {
            // Simulate a mechanical stop: commands farther into an active
            // limit do not change position, while the opposite sign releases it.
            if (voltage > 0.0 && position < 10) {
                ++position;
            } else if (voltage < 0.0 && position > -10) {
                --position;
            }
            leftWasReached = leftWasReached || position <= -10;
            rightWasReached = rightWasReached || position >= 10;
            return pendulum::calibration::HomeCenterSample{
                position, position <= -10, position >= 10};
        },
        [&](double command,
            pendulum::calibration::LimitSide) { voltage = command; },
        [&](const std::string&) { voltage = 0.0; }, [] { return false; });
    const auto fromActiveLimit = activeLimitController.run();
    require(leftWasReached && rightWasReached,
            "active-limit startup did not discover both boundaries");
    require(std::llabs(fromActiveLimit.centerErrorCounts) <= 1,
            "active-limit startup did not return to center");
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

void testMotorZeroPersistence(const std::filesystem::path& sourceConfig) {
    const auto unique = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto directory =
        std::filesystem::temp_directory_path() / ("PendulumLabZeroTest_" + unique);
    std::filesystem::create_directories(directory);
    const auto testConfig = directory / "config.json";
    std::filesystem::copy_file(sourceConfig, testConfig);

    try {
        pendulum::config::AppConfig::saveMotorZeroCalibration(
            testConfig,
            pendulum::config::MotorZeroCalibrationRecord{
                0.00030517578125, 0.00030517578125, 0.5, 1.0, 2.0, 5.0});
        const auto loaded = pendulum::config::AppConfig::load(testConfig);
        require(loaded.pci1723.calibratedZeroVoltage.has_value(),
                "saved motor zero voltage is missing");
        require(*loaded.pci1723.calibratedZeroVoltage == 0.00030517578125,
                "saved motor zero voltage is wrong");
        require(!loaded.pci1723.zeroVoltageRequiresCalibration,
                "saved motor zero calibration flag is wrong");
    } catch (...) {
        std::filesystem::remove_all(directory);
        throw;
    }
    std::filesystem::remove_all(directory);
}

void testHomeCenterPersistence(const std::filesystem::path& sourceConfig) {
    const auto unique = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto directory =
        std::filesystem::temp_directory_path() / ("PendulumLabHomeTest_" + unique);
    std::filesystem::create_directories(directory);
    const auto testConfig = directory / "config.json";
    std::filesystem::copy_file(sourceConfig, testConfig);

    try {
        pendulum::config::AppConfig::saveHomeCenterCalibration(
            testConfig,
            pendulum::config::HomeCenterCalibrationRecord{
                -10000, 12000, 1000, 22000, 1004, 4, true});
        const auto loaded = nlohmann::json::parse(
            std::ifstream(testConfig));
        const auto& saved = loaded.at("home_center_calibration");
        require(saved.at("travel_counts").get<std::int64_t>() == 22000,
                "saved home-center travel is wrong");
        require(saved.at("center_error_counts").get<std::int64_t>() == 4,
                "saved home-center error is wrong");
        require(saved.at("reused_stored_travel").get<bool>(),
                "saved home-center reuse mode is wrong");
        const auto config = pendulum::config::AppConfig::load(testConfig);
        config.validateForManualConsole();
        require(config.homeCenterCalibration.has_value() &&
                    config.homeCenterCalibration->travelCounts == 22000,
                "persisted home-center calibration was not loaded");
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
        testReferenceLqrVelocityController();
        testSafetyOrderAndFaultContainment();
        testEncoderRolloverAndCalibration();
        testMotorZeroSelectionMath();
        testHomeCenterMath();
        testAtomicCalibrationPersistence(argv[1]);
        testMotorZeroPersistence(argv[1]);
        testHomeCenterPersistence(argv[1]);
        std::cout << "Phase 1 tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
}
