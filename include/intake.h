#ifndef ATG_ENGINE_SIM_INTAKE_H
#define ATG_ENGINE_SIM_INTAKE_H

#include "part.h"

#include "gas_system.h"
#include "units.h"
#include "impulse_response.h"

class Intake : public Part {
    public:
        struct Parameters {
            // Plenum volume
            double volume;

            // Plenum dimensions
            double CrossSectionArea;

            // Input flow constant
            double InputFlowK;

            // Idle-circuit flow constant
            double IdleFlowK;

            // Flow rate from plenum to runner
            double RunnerFlowRate;

            // Molecular air fuel ratio (defaults to ideal for octane)
            double MolecularAfr = (25.0 / 2.0);

            // Throttle plate position at idle
            double IdleThrottlePlatePosition = 0.975;

            // Runner volume
            double RunnerLength = units::distance(4.0, units::inch);

            // Velocity decay factor
            double VelocityDecay = 0.5;

            // ---- Turbocharger ----
            // Enable the turbocharger model
            bool Turbo = false;

            // Peak boost pressure above atmospheric (wastegate setting)
            double TurboMaxBoost = units::pressure(1.0, units::bar);

            // Time constant for the turbo shaft to spool up (seconds)
            double TurboSpoolTime = 0.8;

            // Engine speed at which full boost is reached at full load
            double TurboFullBoostRpm = units::rpm(2500.0);

            // Shaft speed at full boost (only used for display/sound)
            double TurboMaxShaftRpm = 160000.0;

            // true = blow-off valve "pssh" on lift-off,
            // false = compressor surge flutter ("stu-tu-tu")
            bool BlowOffValve = true;

            // Sound settings
            double WhistleVolume = 1.0;
            double WhistleMinFrequency = 1200.0;
            double WhistleMaxFrequency = 7500.0;
            double BlowOffVolume = 1.0;

            // ---- Diesel ----
            // Unthrottled intake, accelerator pedal controls fuel quantity
            bool Diesel = false;

            // Minimum fuel delivered at zero pedal (fraction of a stoichiometric charge)
            double DieselIdleFuel = 0.0;

            // Idle governor: adds fuel below this speed (also makes cranking work)
            double DieselIdleRpm = units::rpm(800.0);

            // Idle governor strength: fuel added per fraction of speed below idle
            double DieselGovernorGain = 3.0;

            // Fuel delivered at full pedal, as a fraction of a stoichiometric charge
            double DieselMaxFuel = 0.85;
        };

        struct TurboSoundState {
            float whistleFrequency = 0.0f;
            float whistleAmplitude = 0.0f;
            float hissAmplitude = 0.0f;
            float blowOffAmplitude = 0.0f;
            float flutterAmplitude = 0.0f;
        };

    public:
        Intake();
        virtual ~Intake();

        void initialize(Parameters &params);
        virtual void destroy();

        void process(double dt);

        // Updates the turbocharger. referenceAirFlow is the naturally aspirated
        // air flow (mol/s) this intake feeds at TurboFullBoostRpm.
        void updateTurbo(double dt, double referenceAirFlow);

        inline bool hasTurbo() const { return m_turbo; }
        inline bool isDiesel() const { return m_diesel; }
        inline double getBoostPressure() const { return m_boost; }
        inline double getTurboMaxBoost() const { return m_turbo ? m_turboMaxBoost : 0.0; }
        inline double getTurboShaftRpm() const { return m_turboShaftSpeed * m_turboMaxShaftRpm; }
        double getPedal() const;
        double getFuelingFraction() const;
        void setEngineSpeed(double speed) { m_engineSpeed = speed; }
        void setInjectorSound(ImpulseResponse *sound) { m_injectorSound = sound; }
        ImpulseResponse *getInjectorSound() const { return m_injectorSound; }
        TurboSoundState getTurboSoundState() const;

        inline double getRunnerFlowRate() const { return m_runnerFlowRate; }
        inline double getThrottlePlatePosition() const { return m_idleThrottlePlatePosition * m_throttle; }
        inline double getRunnerLength() const { return m_runnerLength; }
        inline double getPlenumCrossSectionArea() const { return m_crossSectionArea; }
        inline double getVelocityDecay() const { return m_velocityDecay; }

        GasSystem m_system;
        double m_throttle;

        double m_flow;
        double m_flowRate;
        double m_totalFuelInjected;

    protected:
        double m_crossSectionArea;
        double m_inputFlowK;
        double m_idleFlowK;
        double m_runnerFlowRate;
        double m_molecularAfr;
        double m_idleThrottlePlatePosition;
        double m_runnerLength;
        double m_velocityDecay;

        GasSystem m_atmosphere;

        // Turbo state
        bool m_turbo;
        double m_turboMaxBoost;
        double m_turboSpoolTime;
        double m_turboFullBoostRpm;
        double m_turboMaxShaftRpm;
        bool m_blowOffValve;
        double m_whistleVolume;
        double m_whistleMinFrequency;
        double m_whistleMaxFrequency;
        double m_blowOffVolume;

        double m_boost;
        double m_turboShaftSpeed;      // normalized, 1.0 = full boost
        double m_turboLoad;            // normalized exhaust energy
        double m_filteredFuelRate;
        double m_lastTotalFuelInjected;
        double m_filteredPedal;
        double m_blowOffEnvelope;
        double m_flutterEnvelope;

        // Diesel state
        bool m_diesel;
        double m_dieselIdleFuel;
        double m_dieselMaxFuel;
        double m_dieselIdleRpm;
        double m_dieselGovernorGain;
        double m_engineSpeed;
        ImpulseResponse *m_injectorSound;

    public:
        double getTurboFullBoostRpm() const { return m_turboFullBoostRpm; }
};

#endif /* ATG_ENGINE_SIM_INTAKE_H */
