#include "pendulum/control/DoublePendulumLqrController.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace pendulum::control {
namespace {

double wrapToPi(double value) {
    return std::remainder(value, 2.0 * std::numbers::pi);
}

void validate(const DoublePendulumLqrSettings& s) {
    if (!std::isfinite(s.sampleSeconds) || s.sampleSeconds <= 0.0 ||
        !std::isfinite(s.cartMetersPerCount) || s.cartMetersPerCount <= 0.0 ||
        s.firstCountsPerRevolution <= 0 || s.secondCountsPerRevolution <= 0 ||
        !std::isfinite(s.velocityFilterHz) || s.velocityFilterHz <= 0.0 ||
        !std::isfinite(s.accelerationLimit) || s.accelerationLimit <= 0.0 ||
        !std::isfinite(s.velocityReferenceLimit) || s.velocityReferenceLimit <= 0.0 ||
        !std::isfinite(s.velocityProportionalGain) ||
        !std::isfinite(s.velocityIntegralGain) || s.velocityIntegralGain < 0.0 ||
        !std::isfinite(s.voltageLimit) || s.voltageLimit <= 0.0 ||
        !std::isfinite(s.stationaryVoltage) ||
        !std::isfinite(s.firstMassKilograms) || s.firstMassKilograms <= 0.0 ||
        !std::isfinite(s.secondMassKilograms) || s.secondMassKilograms <= 0.0 ||
        !std::isfinite(s.firstCenterOfMassMeters) || s.firstCenterOfMassMeters <= 0.0 ||
        !std::isfinite(s.secondCenterOfMassMeters) || s.secondCenterOfMassMeters <= 0.0 ||
        !std::isfinite(s.firstInertiaKilogramMetersSquared) ||
            s.firstInertiaKilogramMetersSquared <= 0.0 ||
        !std::isfinite(s.secondInertiaKilogramMetersSquared) ||
            s.secondInertiaKilogramMetersSquared <= 0.0 ||
        !std::isfinite(s.trackLimitMeters) || s.trackLimitMeters <= 0.0 ||
        !std::isfinite(s.softTrackLimitMeters) || s.softTrackLimitMeters <= 0.0 ||
        s.softTrackLimitMeters >= s.trackLimitMeters ||
        s.captureAngle1Radians <= 0.0 || s.captureAngle2Radians <= 0.0 ||
        s.balanceReentryAngle1Radians <= s.captureAngle1Radians ||
        s.balanceReentryAngle2Radians <= s.captureAngle2Radians ||
        s.captureRate1RadiansPerSecond <= 0.0 ||
        s.captureRate2RadiansPerSecond <= 0.0 ||
        s.assistRate1RadiansPerSecond <= 0.0 ||
        s.assistRate2RadiansPerSecond <= 0.0) {
        throw std::invalid_argument("Invalid double-pendulum LQR settings");
    }
    for (const auto gain : s.gain) {
        if (!std::isfinite(gain)) {
            throw std::invalid_argument("Double-pendulum LQR gain contains NaN or Inf");
        }
    }
}

}  // namespace

DoublePendulumLqrController::DoublePendulumLqrController(
    DoublePendulumLqrSettings settings)
    : settings_(std::move(settings)) {
    validate(settings_);
}

void DoublePendulumLqrController::reset() noexcept {
    initialized_ = false;
    previousX_ = 0.0;
    previousTheta1_ = 0.0;
    previousTheta2_ = 0.0;
    filteredXdot_ = 0.0;
    filteredTheta1dot_ = 0.0;
    filteredTheta2dot_ = 0.0;
    velocityReference_ = 0.0;
    velocityIntegral_ = 0.0;
    stage_ = 1;
}

void DoublePendulumLqrController::resetCommandIntegrators() noexcept {
    velocityReference_ = filteredXdot_;
    velocityIntegral_ = 0.0;
}

DoubleSoftwareTravelLimitOutput
DoublePendulumLqrController::applySoftwareTravelLimit(
    double requestedVoltage, double cartPositionMeters, double limitMeters,
    bool positiveVoltageMovesRight, double recoveryVoltage) {
    if (!std::isfinite(requestedVoltage) ||
        !std::isfinite(cartPositionMeters) ||
        !std::isfinite(limitMeters) || limitMeters <= 0.0 ||
        !std::isfinite(recoveryVoltage) || recoveryVoltage <= 0.0) {
        throw std::invalid_argument("Invalid double-pendulum software limit input");
    }
    DoubleSoftwareTravelLimitOutput result{requestedVoltage};
    const bool commandMovesRight = positiveVoltageMovesRight
        ? requestedVoltage > 0.0 : requestedVoltage < 0.0;
    const bool commandMovesLeft = positiveVoltageMovesRight
        ? requestedVoltage < 0.0 : requestedVoltage > 0.0;
    if (cartPositionMeters >= limitMeters) {
        result.side = DoubleSoftwareTravelLimitSide::Right;
        if (commandMovesRight) {
            result.outwardCommandBlocked = true;
            result.outputVoltage = positiveVoltageMovesRight
                ? -recoveryVoltage : recoveryVoltage;
        }
    } else if (cartPositionMeters <= -limitMeters) {
        result.side = DoubleSoftwareTravelLimitSide::Left;
        if (commandMovesLeft) {
            result.outwardCommandBlocked = true;
            result.outputVoltage = positiveVoltageMovesRight
                ? recoveryVoltage : -recoveryVoltage;
        }
    }
    return result;
}

const DoublePendulumLqrSettings& DoublePendulumLqrController::settings() const noexcept {
    return settings_;
}

DoublePendulumLqrOutput DoublePendulumLqrController::update(
    std::int64_t cartRelativeCounts, std::int64_t firstRelativeCounts,
    std::int64_t secondRelativeCounts, bool automaticSwingUp) {
    DoublePendulumLqrOutput out;
    out.cartPositionMeters =
        -static_cast<double>(cartRelativeCounts) * settings_.cartMetersPerCount;
    out.firstAngleRadians = wrapToPi(
        -static_cast<double>(firstRelativeCounts) * 2.0 * std::numbers::pi /
        static_cast<double>(settings_.firstCountsPerRevolution));
    const double relativeSecondAngle = wrapToPi(
        -static_cast<double>(secondRelativeCounts) * 2.0 * std::numbers::pi /
        static_cast<double>(settings_.secondCountsPerRevolution));
    out.secondAngleRadians = wrapToPi(out.firstAngleRadians + relativeSecondAngle);

    if (!initialized_) {
        previousX_ = out.cartPositionMeters;
        previousTheta1_ = out.firstAngleRadians;
        previousTheta2_ = out.secondAngleRadians;
        initialized_ = true;
    }
    const double alpha = std::exp(-2.0 * std::numbers::pi *
                                  settings_.velocityFilterHz * settings_.sampleSeconds);
    filteredXdot_ = alpha * filteredXdot_ + (1.0 - alpha) *
        (out.cartPositionMeters - previousX_) / settings_.sampleSeconds;
    filteredTheta1dot_ = alpha * filteredTheta1dot_ + (1.0 - alpha) *
        wrapToPi(out.firstAngleRadians - previousTheta1_) / settings_.sampleSeconds;
    filteredTheta2dot_ = alpha * filteredTheta2dot_ + (1.0 - alpha) *
        wrapToPi(out.secondAngleRadians - previousTheta2_) / settings_.sampleSeconds;
    out.cartVelocityMetersPerSecond = filteredXdot_;
    out.firstAngularRateRadiansPerSecond = filteredTheta1dot_;
    out.secondAngularRateRadiansPerSecond = filteredTheta2dot_;

    const std::array<double, 6> state{
        out.cartPositionMeters, out.firstAngleRadians, out.secondAngleRadians,
        filteredXdot_, filteredTheta1dot_, filteredTheta2dot_};
    double acceleration = 0.0;
    const int previousStage = stage_;
    if (!automaticSwingUp) {
        stage_ = 3;
    } else {
        if (stage_ == 3 &&
            std::abs(out.firstAngleRadians) >
                settings_.balanceReentryAngle1Radians) {
            stage_ = 1;
        } else if (stage_ == 3 &&
                   std::abs(out.secondAngleRadians) >
                       settings_.balanceReentryAngle2Radians) {
            stage_ = 2;
        } else if (stage_ == 1 &&
            std::abs(out.firstAngleRadians) <= settings_.stage1AngleRadians &&
            std::abs(filteredTheta1dot_) <=
                settings_.stage1CaptureRateRadiansPerSecond) {
            stage_ = 2;
        } else if (stage_ == 2 &&
                   std::abs(out.firstAngleRadians) >
                       settings_.stage1ReentryAngleRadians) {
            stage_ = 1;
        } else if (stage_ == 2 &&
                   std::abs(out.firstAngleRadians) <=
                       settings_.captureAngle1Radians &&
                   std::abs(out.secondAngleRadians) <=
                       settings_.captureAngle2Radians &&
                   std::abs(filteredXdot_) <=
                       settings_.captureCartVelocityMetersPerSecond &&
                   out.cartPositionMeters * filteredXdot_ <= 0.0 &&
                   out.cartPositionMeters * out.firstAngleRadians <= 0.0 &&
                   out.cartPositionMeters * out.secondAngleRadians >= 0.0 &&
                   std::abs(filteredTheta1dot_) <=
                       settings_.captureRate1RadiansPerSecond &&
                   std::abs(filteredTheta2dot_) <=
                       settings_.captureRate2RadiansPerSecond) {
            stage_ = 3;
        }
    }
    if (automaticSwingUp && previousStage == 3 && stage_ != 3) {
        resetCommandIntegrators();
    }

    if (stage_ == 1) {
        const double energy1 =
            0.5 * settings_.firstInertiaKilogramMetersSquared *
                filteredTheta1dot_ * filteredTheta1dot_ +
            settings_.firstMassKilograms * settings_.gravityMetersPerSecondSquared *
                settings_.firstCenterOfMassMeters *
                (std::cos(out.firstAngleRadians) - 1.0);
        const double sigma = std::numbers::pi - std::abs(out.firstAngleRadians);
        const double beta = sigma <= settings_.stage1Sigma1
            ? settings_.stage1Gain1
            : (sigma <= settings_.stage1Sigma2
                   ? settings_.stage1Gain2 : settings_.stage1Gain3);
        const double switching = energy1 * filteredTheta1dot_ *
                                 std::cos(out.firstAngleRadians);
        const double requested = std::abs(switching) < 1e-9 &&
                                 std::abs(out.firstAngleRadians) > 2.8
            ? settings_.stage1Gain1
            : -beta * (switching > 0.0 ? 1.0 : -1.0);
        if (std::abs(filteredXdot_) < settings_.cartVelocityLimitMetersPerSecond ||
            requested * filteredXdot_ <= 0.0) {
            acceleration = requested;
        }
    } else if (stage_ == 2) {
        acceleration = -(
            settings_.singlePendulumGain[0] * out.cartPositionMeters +
            settings_.singlePendulumGain[1] * out.firstAngleRadians +
            settings_.singlePendulumGain[2] * filteredXdot_ +
            settings_.singlePendulumGain[3] * filteredTheta1dot_);
        if (std::abs(out.secondAngleRadians) > settings_.stage2FarAngleRadians) {
            acceleration += settings_.stage2FarGain *
                (filteredTheta2dot_ > 0.0 ? 1.0 :
                 (filteredTheta2dot_ < 0.0 ? -1.0 : 0.0));
        } else {
            const double energy2 =
                0.5 * settings_.secondInertiaKilogramMetersSquared *
                    filteredTheta2dot_ * filteredTheta2dot_ +
                settings_.secondMassKilograms * settings_.gravityMetersPerSecondSquared *
                    settings_.secondCenterOfMassMeters *
                    (std::cos(out.secondAngleRadians) - 1.0);
            const double switching = (energy2 - settings_.secondEnergyTargetJoules) *
                                     filteredTheta2dot_ *
                                     std::cos(out.secondAngleRadians);
            acceleration += settings_.stage2NearGain *
                (switching > 0.0 ? 1.0 : (switching < 0.0 ? -1.0 : 0.0));
        }
        if (std::abs(out.firstAngleRadians) <= settings_.assistAngle1Radians &&
            std::abs(out.secondAngleRadians) <= settings_.assistAngle2Radians &&
            std::abs(filteredTheta1dot_) <= settings_.assistRate1RadiansPerSecond &&
            std::abs(filteredTheta2dot_) <= settings_.assistRate2RadiansPerSecond) {
            out.captureAssistActive = true;
            acceleration = 0.0;
            for (std::size_t index = 0; index < state.size(); ++index) {
                acceleration -= settings_.gain[index] * state[index];
            }
        }
    } else {
        for (std::size_t index = 0; index < state.size(); ++index) {
            acceleration -= settings_.gain[index] * state[index];
        }
    }

    if (stage_ != 3 &&
        std::abs(out.cartPositionMeters) >= settings_.softTrackLimitMeters &&
        out.cartPositionMeters * filteredXdot_ > 0.0) {
        const double remaining = std::max(
            settings_.trackLimitMeters - settings_.trackBrakeMarginMeters -
                std::abs(out.cartPositionMeters),
            0.005);
        const double requiredBrake = filteredXdot_ * filteredXdot_ /
                                     (2.0 * remaining);
        const double brake = std::max(
            settings_.minimumBrakeAccelerationMetersPerSecondSquared,
            requiredBrake);
        acceleration = -std::copysign(
            std::min(brake, settings_.accelerationLimit),
            out.cartPositionMeters);
    }
    out.accelerationCommandMetersPerSecondSquared = std::clamp(
        acceleration, -settings_.accelerationLimit, settings_.accelerationLimit);
    velocityReference_ = std::clamp(
        velocityReference_ + settings_.sampleSeconds *
                                 out.accelerationCommandMetersPerSecondSquared,
        -settings_.velocityReferenceLimit, settings_.velocityReferenceLimit);
    out.velocityReferenceMetersPerSecond = velocityReference_;
    out.velocityErrorMetersPerSecond = velocityReference_ - filteredXdot_;

    const double candidateIntegral = velocityIntegral_ + settings_.sampleSeconds *
        out.velocityErrorMetersPerSecond;
    const double unconstrained = settings_.stationaryVoltage +
        settings_.velocityProportionalGain * out.velocityErrorMetersPerSecond +
        settings_.velocityIntegralGain * candidateIntegral;
    out.outputVoltage = std::clamp(
        unconstrained, -settings_.voltageLimit, settings_.voltageLimit);
    out.voltageSaturated = out.outputVoltage != unconstrained;
    out.stage = stage_;
    out.swingUpActive = automaticSwingUp && stage_ != 3;
    if (!out.voltageSaturated ||
        std::signbit(out.velocityErrorMetersPerSecond) !=
            std::signbit(unconstrained - out.outputVoltage)) {
        velocityIntegral_ = candidateIntegral;
    }
    if (!std::isfinite(out.outputVoltage)) {
        throw std::runtime_error("Double-pendulum LQR produced NaN or Inf");
    }

    previousX_ = out.cartPositionMeters;
    previousTheta1_ = out.firstAngleRadians;
    previousTheta2_ = out.secondAngleRadians;
    return out;
}

}  // namespace pendulum::control
