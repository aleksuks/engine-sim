// Headless engine test: runs an engine script without the GUI, prints
// rpm / boost / torque over a scripted drive cycle and writes the audio to a
// WAV file. Run from a directory where ../assets and ../es exist (e.g. build/).
//
//   engine-sim-headless <script.mr> [out.wav] [hold_rpm]
//
// Drive cycle:
//   0.0 - 1.5 s   starter on, pedal 0
//   1.5 - 5.0 s   idle
//   5.0 - 11.0 s  dyno holds <hold_rpm>, full pedal (turbo spools)
//   11.0 - 14.0 s pedal snapped shut (blow-off / surge), dyno off

#include "../include/engine.h"
#include "../include/simulator.h"
#include "../include/units.h"
#include "../include/delta.h"
#include "../scripting/include/compiler.h"

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

static void writeWav(const std::string &path, const std::vector<int16_t> &samples, int rate) {
    std::ofstream f(path, std::ios::binary);
    auto u32 = [&](uint32_t v) { f.write(reinterpret_cast<const char *>(&v), 4); };
    auto u16 = [&](uint16_t v) { f.write(reinterpret_cast<const char *>(&v), 2); };
    const uint32_t dataBytes = static_cast<uint32_t>(samples.size() * 2);
    f.write("RIFF", 4); u32(36 + dataBytes); f.write("WAVE", 4);
    f.write("fmt ", 4); u32(16); u16(1); u16(1); u32(rate); u32(rate * 2); u16(2); u16(16);
    f.write("data", 4); u32(dataBytes);
    f.write(reinterpret_cast<const char *>(samples.data()), dataBytes);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("usage: engine-sim-headless <script.mr> [out.wav] [hold_rpm]\n");
        return 1;
    }

    const std::string wavPath = (argc >= 3) ? argv[2] : "headless.wav";
    const double holdRpm = (argc >= 4) ? std::atof(argv[3]) : 2500.0;

    es_script::Compiler compiler;
    compiler.initialize();
    if (!compiler.compile(argv[1])) {
        std::printf("Script failed to compile (see error_log.log)\n");
        return 1;
    }

    const es_script::Compiler::Output output = compiler.execute();
    compiler.destroy();

    Engine *engine = output.engine;
    if (engine == nullptr || output.vehicle == nullptr || output.transmission == nullptr) {
        std::printf("Script did not set engine/vehicle/transmission\n");
        return 1;
    }

    Simulator *simulator = engine->createSimulator(output.vehicle, output.transmission);
    engine->calculateDisplacement();
    simulator->setSimulationFrequency(engine->getSimulationFrequency());

    Synthesizer::AudioParameters audioParams = simulator->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    simulator->synthesizer().setAudioParameters(audioParams);

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();
        ysWindowsAudioWaveFile waveFile;
        waveFile.OpenFile(response->getFilename().c_str());
        waveFile.InitializeInternalBuffer(waveFile.GetSampleCount());
        waveFile.FillBuffer(0);
        waveFile.CloseFile();
        simulator->synthesizer().initializeImpulseResponse(
            reinterpret_cast<const int16_t *>(waveFile.GetBuffer()),
            waveFile.GetSampleCount(), response->getVolume(), i);
        waveFile.DestroyInternalBuffer();
    }


    for (int i = 0; i < engine->getIntakeCount(); ++i) {
        ImpulseResponse *clack = engine->getIntake(i)->getInjectorSound();
        if (clack == nullptr || clack->getVolume() <= 0) continue;

        ysWindowsAudioWaveFile waveFile;
        if (waveFile.OpenFile(clack->getFilename().c_str()) != ysAudioFile::Error::None) continue;
        waveFile.InitializeInternalBuffer(waveFile.GetSampleCount());
        waveFile.FillBuffer(0);
        waveFile.CloseFile();

        simulator->synthesizer().initializeClackSample(
            reinterpret_cast<const int16_t *>(waveFile.GetBuffer()),
            waveFile.GetSampleCount(),
            static_cast<float>(clack->getVolume()));

        waveFile.DestroyInternalBuffer();
        break;
    }

    simulator->startAudioRenderingThread();

    std::printf("Engine: %s  displacement %.2f L  turbo max boost %.2f bar\n",
        engine->getName().c_str(),
        units::convert(engine->getDisplacement(), units::L),
        units::convert(engine->getMaxTurboBoost(), units::bar));
    std::printf("%6s %7s %8s %9s %9s %8s %8s %6s\n",
        "t[s]", "rpm", "boost", "MAP[bar]", "torque", "shaftk", "whistle", "bov");

    const double frameDt = 1.0 / 60.0;
    std::vector<int16_t> audio;
    std::vector<int16_t> chunk(4096);
    double peakTemperature = 0.0;
    double nextPrint = 0.0;

    engine->getIgnitionModule()->m_enabled = true;

    for (double t = 0.0; t < 14.0; t += frameDt) {
        const bool starter = t < 1.5;
        const bool wot = t >= 5.0 && t < 11.0;

        simulator->m_starterMotor.m_enabled = starter;
        simulator->m_dyno.m_enabled = wot;
        simulator->m_dyno.m_hold = wot;
        simulator->m_dyno.m_rotationSpeed = units::rpm(holdRpm);
        engine->setSpeedControl(wot ? 1.0 : 0.0);

        simulator->startFrame(frameDt);
        while (simulator->simulateStep()) {
            for (int i = 0; i < engine->getCylinderCount(); ++i) {
                const double T = engine->getChamber(i)->m_system.temperature();
                if (T > peakTemperature) peakTemperature = T;
            }
        }
        simulator->endFrame();
        simulator->synthesizer().waitProcessed();

        const int n = simulator->readAudioOutput(735, chunk.data());
        audio.insert(audio.end(), chunk.begin(), chunk.begin() + n);

        if (t >= nextPrint) {
            const Intake::TurboSoundState s = (engine->getIntakeCount() > 0)
                ? engine->getIntake(0)->getTurboSoundState()
                : Intake::TurboSoundState();
            std::printf("%6.2f %7.0f %8.2f %9.2f %9.1f %8.0f %8.2f %6.2f   peakT %.0fK\n",
                t,
                engine->getRpm(),
                units::convert(engine->getBoostPressure(), units::bar),
                units::convert(engine->getManifoldPressure(), units::bar),
                units::convert(simulator->getFilteredDynoTorque(), units::Nm),
                engine->getIntakeCount() > 0 ? engine->getIntake(0)->getTurboShaftRpm() / 1000.0 : 0.0,
                s.whistleAmplitude,
                s.blowOffAmplitude + s.flutterAmplitude,
                peakTemperature);
            if (getenv("ES_DEBUG") != nullptr && engine->getIntakeCount() > 0) {
                const GasSystem::Mix m = engine->getIntake(0)->m_system.mix();
                std::printf("       plenum o2=%.3f fuel=%.4f runner o2=%.3f cyl o2=%.3f pedal=%.2f fuelFrac=%.2f\n",
                    m.p_o2, m.p_fuel,
                    engine->getChamber(0)->m_intakeRunnerAndManifold.mix().p_o2,
                    engine->getChamber(0)->m_system.mix().p_o2,
                    engine->getIntake(0)->getPedal(),
                    engine->getIntake(0)->getFuelingFraction());
            }
            peakTemperature = 0.0;
            nextPrint += 0.5;
        }
    }

    simulator->endAudioRenderingThread();
    writeWav(wavPath, audio, 44100);
    std::printf("Wrote %zu samples (%.1f s) to %s\n",
        audio.size(), audio.size() / 44100.0, wavPath.c_str());

    return 0;
}
