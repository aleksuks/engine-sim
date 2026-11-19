#ifndef ATG_ENGINE_SIM_INTAKE_NODE_H
#define ATG_ENGINE_SIM_INTAKE_NODE_H

#include "object_reference_node.h"

#include "engine_context.h"
#include "function_node.h"
#include "impulse_response_node.h"

#include "engine_sim.h"

#include <map>
#include <vector>

namespace es_script {

    class IntakeNode : public ObjectReferenceNode<IntakeNode> {
    public:
        IntakeNode() { /* void */ }
        virtual ~IntakeNode() { /* void */ }

        Intake *generate(EngineContext *context) {
            Intake *intake = context->getIntake(this);
            Intake::Parameters parameters = m_parameters;
            intake->initialize(parameters);
            if (m_injectorSound != nullptr) {
                intake->setInjectorSound(m_injectorSound->generate(context));
            }

            return intake;
        }

    protected:
        virtual void registerInputs() {
            addInput("plenum_volume", &m_parameters.volume);
            addInput("plenum_cross_section_area", &m_parameters.CrossSectionArea);
            addInput("intake_flow_rate", &m_parameters.InputFlowK);
            addInput("idle_flow_rate", &m_parameters.IdleFlowK);
            addInput("runner_flow_rate", &m_parameters.RunnerFlowRate);
            addInput("molecular_afr", &m_parameters.MolecularAfr);
            addInput("idle_throttle_plate_position", &m_parameters.IdleThrottlePlatePosition);
            addInput("throttle_gamma", &m_throttleGammaUnused);
            addInput("runner_length", &m_parameters.RunnerLength);
            addInput("velocity_decay", &m_parameters.VelocityDecay);
            addInput("turbo", &m_parameters.Turbo);
            addInput("turbo_max_boost", &m_parameters.TurboMaxBoost);
            addInput("turbo_spool_time", &m_parameters.TurboSpoolTime);
            addInput("turbo_full_boost_rpm", &m_parameters.TurboFullBoostRpm);
            addInput("turbo_max_shaft_rpm", &m_parameters.TurboMaxShaftRpm);
            addInput("blow_off_valve", &m_parameters.BlowOffValve);
            addInput("whistle_volume", &m_parameters.WhistleVolume);
            addInput("whistle_min_frequency", &m_parameters.WhistleMinFrequency);
            addInput("whistle_max_frequency", &m_parameters.WhistleMaxFrequency);
            addInput("blow_off_volume", &m_parameters.BlowOffVolume);
            addInput("diesel", &m_parameters.Diesel);
            addInput("diesel_idle_fuel", &m_parameters.DieselIdleFuel);
            addInput("diesel_max_fuel", &m_parameters.DieselMaxFuel);
            addInput("diesel_idle_rpm", &m_parameters.DieselIdleRpm);
            addInput("injector_sound", &m_injectorSound, InputTarget::Type::Object);
            addInput("diesel_governor_gain", &m_parameters.DieselGovernorGain);

            ObjectReferenceNode<IntakeNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        double m_throttleGammaUnused = 0.0; // Deprecated; to be removed in a future release
        Intake::Parameters m_parameters;
        ImpulseResponseNode *m_injectorSound = nullptr;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_INTAKE_NODE_H */
