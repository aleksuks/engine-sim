#ifndef ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H
#define ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H

#include "convolution_filter.h"
#include "leveling_filter.h"
#include "derivative_filter.h"
#include "low_pass_filter.h"
#include "jitter_filter.h"
#include "ring_buffer.h"
#include "butterworth_low_pass_filter.h"

#include <cinttypes>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

class Synthesizer {
    public:
        struct AudioParameters {
            float volume = 1.0f;
            float convolution = 1.0f;
            float dF_F_mix = 0.01f;
            float inputSampleNoise = 0.5f;
            float inputSampleNoiseFrequencyCutoff = 10000.0f;
            float airNoise = 1.0f;
            float airNoiseFrequencyCutoff = 2000.0f;
            float levelerTarget = 30000.0f;
            float levelerMaxGain = 1.9f;
            float levelerMinGain = 0.00001f;
        };

        struct Parameters {
            int inputChannelCount = 1;
            int inputBufferSize = 1024;
            int audioBufferSize = 44100;
            float inputSampleRate = 10000;
            float audioSampleRate = 44100;
            AudioParameters initialAudioParameters;
        };

        struct TurboAudio {
            float whistleFrequency = 0.0f;
            float whistleAmplitude = 0.0f;
            float hissAmplitude = 0.0f;
            float blowOffAmplitude = 0.0f;
            float flutterAmplitude = 0.0f;
        };

        struct InputChannel {
            RingBuffer<float> data;
            float *transferBuffer = nullptr;
            double lastInputSample = 0.0f;
        };

        struct ProcessingFilters {
            ConvolutionFilter convolution;
            DerivativeFilter derivative;
            JitterFilter jitterFilter;
            ButterworthLowPassFilter<float> airNoiseLowPass;
            LowPassFilter inputDcFilter;
            ButterworthLowPassFilter<double> antialiasing;
        };

    public:
        Synthesizer();
        ~Synthesizer();

        void initialize(const Parameters &p);
        void initializeImpulseResponse(
            const int16_t *impulseResponse,
            unsigned int samples,
            float volume,
            int index);
        void startAudioRenderingThread();
        void endAudioRenderingThread();
        void destroy();

        int readAudioOutput(int samples, int16_t *buffer);

        void writeInput(const double *data);
        void endInputBlock();

        void waitProcessed();

        void audioRenderingThread();
        void renderAudio();

        double getLatency() const;

        int inputDelta(int s1, int s0) const;
        double inputDistance(double s1, double s0) const;

        void setInputSampleRate(double sampleRate);
        double getInputSampleRate() const { return m_inputSampleRate; }

        int16_t renderAudio(int inputOffset);

        void setTurboAudio(const TurboAudio &audio);

        // Diesel injector clatter: a short recorded sample retriggered on every
        // injection, aligned to the simulation timeline
        void initializeClackSample(const int16_t *samples, unsigned int count, float volume);
        void triggerClack(float amplitude);
        float renderClack(float trigger);
        float renderTurbo();

        double getLevelerGain();
        AudioParameters getAudioParameters();
        void setAudioParameters(const AudioParameters &params);

    //protected:
        ButterworthLowPassFilter<float> m_antialiasing;
        LevelingFilter m_levelingFilter;
        InputChannel *m_inputChannels;
        AudioParameters m_audioParameters;
        int m_inputChannelCount;
        int m_inputBufferSize;
        int m_inputSamplesRead;
        int m_latency;
        double m_inputWriteOffset;
        double m_lastInputSampleOffset;

        RingBuffer<int16_t> m_audioBuffer;
        int m_audioBufferSize;

        float m_inputSampleRate;
        float m_audioSampleRate;

        std::thread *m_thread;
        std::atomic<bool> m_run;
        bool m_processed;

        std::mutex m_inputLock;
        std::mutex m_lock0;
        std::condition_variable m_cv0;

        ProcessingFilters *m_filters;

        // Turbo sound (written by the simulation thread, read by audio thread)
        std::atomic<float> m_turboWhistleFrequency;
        std::atomic<float> m_turboWhistleAmplitude;
        std::atomic<float> m_turboHissAmplitude;
        std::atomic<float> m_turboBlowOffAmplitude;
        std::atomic<float> m_turboFlutterAmplitude;

        // Turbo DSP state (audio thread only)
        double m_whistlePhase;
        double m_flutterPhase;
        double m_vibratoPhase;
        float m_smoothWhistleFrequency;
        float m_smoothWhistleAmplitude;
        float m_smoothHissAmplitude;
        float m_smoothBlowOffAmplitude;
        float m_smoothFlutterAmplitude;
        float m_hissLp;
        float m_hissHp;
        float m_bovLp;
        float m_bovHp;
        float m_flutterLp;
        uint32_t m_noiseState;

        // Injector clatter
        static constexpr int ClackVoices = 8;
        struct ClackVoice {
            double position = -1.0;
            double rate = 1.0;
            float amplitude = 0.0f;
        };
        float *m_clackSample;
        unsigned int m_clackSampleLength;
        float m_clackVolume;
        float m_pendingClack;
        RingBuffer<float> m_clackTriggerData;
        float *m_clackTriggerTransfer;
        ClackVoice m_clackVoices[ClackVoices];
};

#endif /* ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H */
