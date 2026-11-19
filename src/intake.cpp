#include "../include/intake.h"

#include "../include/units.h"

#include <cmath>

Intake::Intake() {
    m_inputFlowK = 0;
    m_idleFlowK = 0;
    m_flow = 0;
    m_throttle = 1.0;
    m_idleThrottlePlatePosition = 0.0;
    m_crossSectionArea = 0.0;
    m_flowRate = 0;
    m_totalFuelInjected = 0;
    m_molecularAfr = 0;
    m_runnerLength = 0;

    m_turbo = false;
    m_turboMaxBoost = 0;
    m_turboSpoolTime = 1.0;
    m_turboFullBoostRpm = 0;
    m_turboMaxShaftRpm = 0;
    m_blowOffValve = true;
    m_whistleVolume = 0;
    m_whistleMinFrequency = 0;
    m_whistleMaxFrequency = 0;
    m_blowOffVolume = 0;

    m_boost = 0;
    m_turboShaftSpeed = 0;
    m_turboLoad = 0;
    m_filteredFuelRate = 0;
    m_lastTotalFuelInjected = 0;
    m_filteredPedal = 0;
    m_blowOffEnvelope = 0;
    m_flutterEnvelope = 0;

    m_diesel = false;
    m_dieselIdleFuel = 0;
    m_dieselMaxFuel = 1;
    m_dieselIdleRpm = 0;
    m_dieselGovernorGain = 0;
    m_engineSpeed = 0;
    m_injectorSound = nullptr;
}

Intake::~Intake() {
    /* void */
}

void Intake::initialize(Parameters &params) {
    const double width = std::sqrt(params.CrossSectionArea);
    m_system.initialize(
        units::pressure(1.0, units::atm),
        params.volume,
        units::celcius(25.0));
    m_system.setGeometry(
        width,
        params.volume / params.CrossSectionArea,
        1.0,
        0.0);

    m_atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0));
    m_atmosphere.setGeometry(
        units::distance(100.0, units::m),
        units::distance(100.0, units::m),
        1.0,
        0.0);

    m_inputFlowK = params.InputFlowK;
    m_molecularAfr = params.MolecularAfr;
    m_idleFlowK = params.IdleFlowK;
    m_idleThrottlePlatePosition = params.IdleThrottlePlatePosition;
    m_runnerLength = params.RunnerLength;
    m_crossSectionArea = params.CrossSectionArea;
    m_velocityDecay = params.VelocityDecay;
    m_runnerFlowRate = params.RunnerFlowRate;

    m_turbo = params.Turbo;
    m_turboMaxBoost = std::fmax(params.TurboMaxBoost, 0.0);
    m_turboSpoolTime = std::fmax(params.TurboSpoolTime, 0.01);
    m_turboFullBoostRpm = std::fmax(params.TurboFullBoostRpm, units::rpm(100.0));
    m_turboMaxShaftRpm = params.TurboMaxShaftRpm;
    m_blowOffValve = params.BlowOffValve;
    m_whistleVolume = params.WhistleVolume;
    m_whistleMinFrequency = params.WhistleMinFrequency;
    m_whistleMaxFrequency = params.WhistleMaxFrequency;
    m_blowOffVolume = params.BlowOffVolume;

    m_diesel = params.Diesel;
    m_dieselIdleFuel = std::fmax(params.DieselIdleFuel, 0.0);
    m_dieselMaxFuel = std::fmax(params.DieselMaxFuel, m_dieselIdleFuel + 1E-3);
    m_dieselIdleRpm = params.DieselIdleRpm;
    m_dieselGovernorGain = params.DieselGovernorGain;

    m_boost = 0;
    m_turboShaftSpeed = 0;
    m_turboLoad = 0;
    m_filteredFuelRate = 0;
    m_lastTotalFuelInjected = 0;
    m_filteredPedal = 0;
    m_blowOffEnvelope = 0;
    m_flutterEnvelope = 0;
}

void Intake::destroy() {
    /* void */
}

double Intake::getPedal() const {
    const double pedal = 1.0 - m_throttle;
    return (pedal < 0.0) ? 0.0 : (pedal > 1.0 ? 1.0 : pedal);
}

double Intake::getFuelingFraction() const {
    if (!m_diesel) return 1.0;

    // Pedal sets the fuel quantity directly
    const double pedalFuel =
        m_dieselIdleFuel + (m_dieselMaxFuel - m_dieselIdleFuel) * getPedal();

    // Idle governor: more fuel the further the engine drops below idle speed
    double governorFuel = 0.0;
    if (m_dieselIdleRpm > 0) {
        const double deficit = (m_dieselIdleRpm - std::abs(m_engineSpeed)) / m_dieselIdleRpm;
        governorFuel = std::fmin(std::fmax(deficit * m_dieselGovernorGain, 0.0), m_dieselMaxFuel);
    }

    return std::fmax(pedalFuel, governorFuel);
}

Intake::TurboSoundState Intake::getTurboSoundState() const {
    TurboSoundState state;
    if (!m_turbo) return state;

    const double s = m_turboShaftSpeed;
    const double load = std::fmin(std::fmax(m_turboLoad, 0.0), 1.0);
    const double pitch = std::fmin(s / 1.25, 1.0);

    state.whistleFrequency = static_cast<float>(
        m_whistleMinFrequency + (m_whistleMaxFrequency - m_whistleMinFrequency) * pitch);
    state.whistleAmplitude = static_cast<float>(
        m_whistleVolume * std::fmin(s * s, 1.5) * (0.35 + 0.65 * load));
    state.hissAmplitude = static_cast<float>(
        0.6 * m_whistleVolume * load * std::fmin(s, 1.2));
    state.blowOffAmplitude = static_cast<float>(m_blowOffVolume * m_blowOffEnvelope);
    state.flutterAmplitude = static_cast<float>(m_blowOffVolume * m_flutterEnvelope);

    return state;
}

void Intake::updateTurbo(double dt, double referenceAirFlow) {
    // The total can be reset elsewhere (fuel consumption reset), so clamp
    const double fuelInjected =
        std::fmax(m_totalFuelInjected - m_lastTotalFuelInjected, 0.0);
    m_lastTotalFuelInjected = m_totalFuelInjected;

    if (!m_turbo || dt <= 0) return;

    // Exhaust energy is roughly proportional to the rate fuel is burned
    const double fuelRate = fuelInjected / dt;
    const double alpha = dt / (0.06 + dt);
    m_filteredFuelRate += (fuelRate - m_filteredFuelRate) * alpha;

    // Fuel flow that corresponds to "full boost" exhaust energy
    const double airToFuel = m_diesel
        ? (4.0 * m_molecularAfr / m_dieselMaxFuel)
        : (0.8 * m_molecularAfr * 4);
    const double referenceFuelRate = referenceAirFlow / airToFuel;

    m_turboLoad = (referenceFuelRate > 0)
        ? m_filteredFuelRate / referenceFuelRate
        : 0.0;

    // Shaft speed follows exhaust energy with a lag (turbo lag)
    const double targetSpeed = std::sqrt(std::fmin(std::fmax(m_turboLoad, 0.0), 1.69));
    const double tau = (targetSpeed > m_turboShaftSpeed)
        ? m_turboSpoolTime
        : m_turboSpoolTime * 1.6;
    m_turboShaftSpeed += (targetSpeed - m_turboShaftSpeed) * std::fmin(dt / tau, 1.0);

    // Compressor pressure rise ~ shaft speed squared, capped by the wastegate
    double boostTarget =
        m_turboMaxBoost * std::fmin(m_turboShaftSpeed * m_turboShaftSpeed, 1.0);

    const double pedal = getPedal();
    const double lastFilteredPedal = m_filteredPedal;
    m_filteredPedal += (pedal - m_filteredPedal) * std::fmin(dt / 0.25, 1.0);

    if (!m_diesel && pedal < 0.12) {
        const bool snappedShut =
            (lastFilteredPedal - pedal) > 0.2
            && m_boost > 0.15 * m_turboMaxBoost;

        if (m_blowOffValve) {
            // BOV opens and vents the boost to atmosphere
            if (snappedShut && m_blowOffEnvelope < 0.3) {
                m_blowOffEnvelope = std::fmin(std::fmax(m_boost / m_turboMaxBoost, 0.3), 1.0);
            }

            boostTarget = 0.0;
        }
        else {
            // No BOV: pressure is trapped and the compressor surges
            if (snappedShut && m_flutterEnvelope < 0.3) {
                m_flutterEnvelope = std::fmin(std::fmax(m_boost / m_turboMaxBoost, 0.3), 1.0);
            }

            m_turboShaftSpeed -= m_turboShaftSpeed * std::fmin(dt * 1.5, 1.0);
        }
    }

    m_boost += (boostTarget - m_boost) * std::fmin(dt / 0.05, 1.0);
    if (m_boost < 0) m_boost = 0;

    m_blowOffEnvelope *= std::exp(-dt / 0.35);
    m_flutterEnvelope *= std::exp(-dt / 0.6);
}

void Intake::process(double dt) {
    // Diesel: only air comes through the intake (fuel is injected directly
    // into the cylinder, see CombustionChamber::ignite)
    const double airToFuel = 0.8 * m_molecularAfr * 4;
    const double p_air = m_diesel ? 1.0 : airToFuel / (1 + airToFuel);
    GasSystem::Mix fuelAirMix;
    fuelAirMix.p_fuel = 1 - p_air;
    fuelAirMix.p_inert = p_air * 0.75;
    fuelAirMix.p_o2 = p_air * 0.25;

    const double idle_afr = 2.0;
    const double p_idle_air = idle_afr / (1 + idle_afr);
    GasSystem::Mix fuelMix;
    fuelMix.p_fuel = (1.0 - p_idle_air);
    fuelMix.p_inert = p_idle_air * 0.75;
    fuelMix.p_o2 = p_idle_air * 0.25;

    const double throttle = getThrottlePlatePosition();
    const double flowAttenuation = m_diesel
        ? 1.0
        : std::cos(throttle * constants::pi / 2);

    // Charge air: boost pressure from the turbo (intercooled, slightly warm)
    const double ambient = units::pressure(1.0, units::atm);
    const double sourcePressure = ambient + (m_turbo ? m_boost : 0.0);
    const double sourceTemperature =
        units::celcius(25.0)
        + 15.0 * (m_turbo ? m_boost / units::pressure(1.0, units::bar) : 0.0);

    GasSystem::FlowParameters flowParams;
    flowParams.crossSectionArea_0 = units::area(10, units::m2);
    flowParams.crossSectionArea_1 = m_crossSectionArea;
    flowParams.direction_x = 0.0;
    flowParams.direction_y = -1.0;
    flowParams.dt = dt;

    m_atmosphere.reset(sourcePressure, sourceTemperature, fuelAirMix);
    flowParams.system_0 = &m_atmosphere;
    flowParams.system_1 = &m_system;
    flowParams.k_flow = flowAttenuation * m_inputFlowK;
    m_flow = m_system.flow(flowParams);

    double idleCircuitFlow = 0.0;
    if (!m_diesel) {
        m_atmosphere.reset(ambient, units::celcius(25.0), fuelMix);
        flowParams.system_0 = &m_atmosphere;
        flowParams.system_1 = &m_system;
        flowParams.k_flow = m_idleFlowK;
        idleCircuitFlow = m_system.flow(flowParams);
    }

    m_system.dissipateExcessVelocity();
    m_system.updateVelocity(dt, m_velocityDecay);

    if (m_flow > 0) {
        m_totalFuelInjected += fuelAirMix.p_fuel * m_flow;
    }

    if (idleCircuitFlow > 0) {
        m_totalFuelInjected += fuelMix.p_fuel * idleCircuitFlow;
    }
}
