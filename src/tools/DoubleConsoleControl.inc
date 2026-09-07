    void startDoubleBalance(bool automatic) {
        if (balanceRunning_.load() || !outputsAllowed() || calibrationRunning_.load() ||
            homingRunning_.load() || !homeResultAvailable_.load() || !pendulumZeroCaptured_.load()) {
            notify("Balance start requires clear limits, a session center and captured downward zeros.", true);
            return;
        }
        const auto& settings = doubleConfig_->controller;
        const auto sample = latestPendulumSample();
        const auto firstDown = wrappedCounts(sample.positionCounts - pendulumDownCount_.load(),
                                             settings.firstCountsPerRevolution);
        const auto secondDown = wrappedCounts(sample.secondPositionCounts - secondDownCount_.load(),
                                              settings.secondCountsPerRevolution);
        const double firstUpright = static_cast<double>(wrappedCounts(
            firstDown - settings.firstCountsPerRevolution / 2, settings.firstCountsPerRevolution)) *
            2.0 * std::numbers::pi / static_cast<double>(settings.firstCountsPerRevolution);
        const double secondUpright = firstUpright + static_cast<double>(secondDown) *
            2.0 * std::numbers::pi / static_cast<double>(settings.secondCountsPerRevolution);
        if ((automatic && (std::llabs(firstDown) > doubleConfig_->firstDownwardMaximumSpanCounts ||
                           std::llabs(secondDown) > doubleConfig_->secondDownwardMaximumSpanCounts)) ||
            (!automatic && (std::abs(firstUpright) > doubleConfig_->startAngleRadians ||
                            std::abs(secondUpright) > doubleConfig_->startAngleRadians))) {
            notify("Balance start rejected: both links must be at the requested downward/upright reference.", true);
            return;
        }
        if (std::abs(currentCartSessionState(0.0).positionFromCenterHalfTravel) >
            config_.balanceControl.maximumBalanceStartPositionFraction) {
            notify("Return to the session center before balance start.", true);
            return;
        }
        if (balanceThread_.joinable()) balanceThread_.join();
        pendulumUprightCount_.store(automatic
            ? pendulumDownCount_.load() + settings.firstCountsPerRevolution / 2 : sample.positionCounts);
        secondReferenceCount_.store(automatic ? secondDownCount_.load() : sample.secondPositionCounts);
        balanceCartReferenceCount_.store(homeCalibrationCenter_.load());
        balanceAbort_.store(false);
        balanceAutoMode_.store(automatic);
        balanceRunning_.store(true);
        balanceSoftwareLimitActive_.store(false);
        balanceMissedDeadlines_.store(0);
        balanceMaxJitterMicroseconds_.store(0.0);
        const auto runId = balanceRunId_.fetch_add(1) + 1;
        balanceThread_ = std::jthread([this, runId, automatic](std::stop_token token) {
            doubleBalanceLoop(token, runId, automatic);
        });
        notify("Double-pendulum balance started at 200 Hz; run_id=" + std::to_string(runId));
    }

    void doubleBalanceLoop(std::stop_token token, std::uint64_t runId, bool automatic) noexcept {
        try {
            const auto& config = *doubleConfig_;
            pendulum::control::DoublePendulumLqrController controller(config.controller);
            controller.reset();
            const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(config.controller.sampleSeconds));
            auto previous = latestPendulumSample();
            const auto start = previous.time;
            auto next = start;
            std::uint64_t index = 0;
            while (!token.stop_requested() && !balanceAbort_.load() &&
                   !faultLatched_.load() && !safety_.stopRequested()) {
                PendulumSample sample;
                {
                    std::unique_lock lock(pendulumSampleMutex_);
                    const bool ready = pendulumSampleCondition_.wait_for(lock,
                        std::chrono::duration<double>(config.sampleTimeoutSeconds), [&] {
                            return (pendulumSample_.sequence != previous.sequence && pendulumSample_.time >= next) ||
                                token.stop_requested() || balanceAbort_.load() || faultLatched_.load() || safety_.stopRequested();
                        });
                    if (token.stop_requested() || balanceAbort_.load() || faultLatched_.load() || safety_.stopRequested()) break;
                    if (!ready) throw std::runtime_error("double control sample timeout");
                    sample = pendulumSample_;
                }
                const double elapsed = std::chrono::duration<double>(sample.time - start).count();
                if (config.durationSeconds > 0 && elapsed >= config.durationSeconds) break;
                if (std::chrono::duration<double>(sample.time - previous.time).count() > config.sampleTimeoutSeconds)
                    throw std::runtime_error("double control sample timeout");
                if (std::llabs(sample.motorPositionCounts - previous.motorPositionCounts) > config.cartJumpCounts ||
                    std::llabs(sample.positionCounts - previous.positionCounts) > config.firstJumpCounts ||
                    std::llabs(sample.secondPositionCounts - previous.secondPositionCounts) > config.secondJumpCounts)
                    throw std::runtime_error("double control encoder jump");
                previous = sample;
                next += period;
                while (next <= sample.time) { next += period; balanceMissedDeadlines_.fetch_add(1); }
                if (!outputsAllowed()) throw std::runtime_error("double control limit or hardware fault");
                const auto output = controller.update(
                    sample.motorPositionCounts - balanceCartReferenceCount_.load(),
                    sample.positionCounts - pendulumUprightCount_.load(),
                    sample.secondPositionCounts - secondReferenceCount_.load(), automatic);
                if (!automatic && (std::abs(output.firstAngleRadians) >= config.stopAngleRadians ||
                                   std::abs(output.secondAngleRadians) >= config.stopAngleRadians))
                    throw std::runtime_error("double pendulum left the configured LQR angle region");
                if (std::abs(output.cartPositionMeters) >= config.positionStopMeters)
                    throw std::runtime_error("double cart exceeded software stop");
                auto limit = pendulum::control::DoubleSoftwareTravelLimitOutput{output.outputVoltage};
                if (automatic) {
                    limit = pendulum::control::DoublePendulumLqrController::applySoftwareTravelLimit(
                        output.outputVoltage, output.cartPositionMeters, config.positionWarningMeters,
                        config_.pci1723.positiveVoltageCartDirection == "RIGHT", config.softwareLimitRecoveryVoltage);
                    if (limit.outwardCommandBlocked) controller.resetCommandIntegrators();
                }
                balanceSoftwareLimitActive_.store(limit.side != pendulum::control::DoubleSoftwareTravelLimitSide::None);
                {
                    std::scoped_lock lock(outputMutex_);
                    if (balanceAbort_.load() || !outputsAllowed() || safety_.stopRequested()) break;
                    if (!servoOn_.load()) {
                        analogOutput_.writeVoltage(0.0);
                        ni_.setServoEnabled(true);
                        servoOn_.store(true);
                    }
                    analogOutput_.writeVoltage(limit.outputVoltage);
                    commandedVoltage_.store(limit.outputVoltage);
                }
                balanceAngleDegrees_.store(output.firstAngleRadians * 180.0 / std::numbers::pi);
                balanceAngularRateDegrees_.store(output.firstAngularRateRadiansPerSecond * 180.0 / std::numbers::pi);
                balanceReferenceAcceleration_.store(output.accelerationCommandMetersPerSecondSquared);
                balanceReferenceVelocity_.store(output.velocityReferenceMetersPerSecond);
                balanceCartPositionMeters_.store(output.cartPositionMeters);
                balanceCartVelocityMetersPerSecond_.store(output.cartVelocityMetersPerSecond);
                balanceVelocityError_.store(output.velocityErrorMetersPerSecond);
                balanceProportionalVoltage_.store(output.proportionalVoltage);
                balancePiIntegralVoltage_.store(output.integralVoltage);
                balanceSwingUpActive_.store(output.swingUpActive);
                if (++index % config_.balanceControl.telemetryDivider == 0) {
                    std::ostringstream log;
                    log << std::setprecision(17) << "run_id=" << runId << ",time_s=" << elapsed
                        << ",controller=double_swing_up_lqr,stage=" << output.stage
                        << ",cart_counts=" << sample.motorPositionCounts
                        << ",theta1_counts=" << sample.positionCounts << ",theta2_counts=" << sample.secondPositionCounts
                        << ",theta1_rad=" << output.firstAngleRadians << ",theta2_rad=" << output.secondAngleRadians
                        << ",x_m=" << output.cartPositionMeters << ",voltage=" << limit.outputVoltage;
                    logger_.log(pendulum::logging::Level::Info, "BalanceTelemetry", log.str());
                }
            }
        } catch (const std::exception& error) {
            record(std::string("Double balance stopped: ") + error.what(), pendulum::logging::Level::Error);
            notify(std::string("Double balance stopped: ") + error.what(), true);
        } catch (...) {
            record("Double balance stopped: unknown exception", pendulum::logging::Level::Error);
        }
        stopOutputs("double balance loop exit");
        balanceSwingUpActive_.store(false);
        balanceSoftwareLimitActive_.store(false);
        balanceRunning_.store(false);
    }
