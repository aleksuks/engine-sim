#include "../include/synthesizer.h"

#include "../include/utilities.h"
#include "../include/delta.h"

#include <cassert>
#include <cmath>

#undef min
#undef max

Synthesizer::Synthesizer() {
    m_inputChannels = nullptr;
    m_inputChannelCount = 0;
    m_inputBufferSize = 0;
    m_inputWriteOffset = 0.0;
    m_inputSamplesRead = 0;

    m_audioBufferSize = 0;

    m_inputSampleRate = 0.0;
    m_audioSampleRate = 0.0;

    m_lastInputSampleOffset = 0.0;

    m_run = true;
    m_thread = nullptr;
    m_filters = nullptr;

    m_turboWhistleFrequency = 0.0f;
    m_turboWhistleAmplitude = 0.0f;
    m_turboHissAmplitude = 0.0f;
    m_turboBlowOffAmplitude = 0.0f;
    m_turboFlutterAmplitude = 0.0f;

    m_whistlePhase = 0.0;
    m_flutterPhase = 0.0;
    m_vibratoPhase = 0.0;
    m_smoothWhistleFrequency = 0.0f;
    m_smoothWhistleAmplitude = 0.0f;
    m_smoothHissAmplitude = 0.0f;
    m_smoothBlowOffAmplitude = 0.0f;
    m_smoothFlutterAmplitude = 0.0f;
    m_hissLp = 0.0f;
    m_hissHp = 0.0f;
    m_bovLp = 0.0f;
    m_bovHp = 0.0f;
    m_flutterLp = 0.0f;
    m_noiseState = 0x12345678u;

    m_clackSample = nullptr;
    m_clackSampleLength = 0;
    m_clackVolume = 0.0f;
    m_pendingClack = 0.0f;
    m_clackTriggerTransfer = nullptr;
}

Synthesizer::~Synthesizer() {
    assert(m_inputChannels == nullptr);
    assert(m_thread == nullptr);
    assert(m_filters == nullptr);
}

void Synthesizer::initialize(const Parameters &p) {
    m_inputChannelCount = p.inputChannelCount;
    m_inputBufferSize = p.inputBufferSize;
    m_inputWriteOffset = p.inputBufferSize;
    m_audioBufferSize = p.audioBufferSize;
    m_inputSampleRate = p.inputSampleRate;
    m_audioSampleRate = p.audioSampleRate;
    m_audioParameters = p.initialAudioParameters;

    m_inputSamplesRead = 0;

    m_inputWriteOffset = 0;
    m_processed = true;

    m_audioBuffer.initialize(p.audioBufferSize);
    m_inputChannels = new InputChannel[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_inputChannels[i].transferBuffer = new float[p.inputBufferSize];
        m_inputChannels[i].data.initialize(p.inputBufferSize);
    }

    m_clackTriggerData.initialize(p.inputBufferSize);
    m_clackTriggerTransfer = new float[p.inputBufferSize];

    m_filters = new ProcessingFilters[p.inputChannelCount];
    for (int i = 0; i < p.inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            m_audioParameters.airNoiseFrequencyCutoff, m_audioSampleRate);

        m_filters[i].derivative.m_dt = 1 / m_audioSampleRate;

        m_filters[i].inputDcFilter.setCutoffFrequency(10.0);
        m_filters[i].inputDcFilter.m_dt = 1 / m_audioSampleRate;

        m_filters[i].jitterFilter.initialize(
            10,
            m_audioParameters.inputSampleNoiseFrequencyCutoff,
            m_audioSampleRate);

        m_filters[i].antialiasing.setCutoffFrequency(1900.0f, m_audioSampleRate);
    }

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    m_levelingFilter.p_maxLevel = m_audioParameters.levelerMaxGain;
    m_levelingFilter.p_minLevel = m_audioParameters.levelerMinGain;
    m_antialiasing.setCutoffFrequency(m_audioSampleRate * 0.45f, m_audioSampleRate);

    for (int i = 0; i < m_audioBufferSize; ++i) {
        m_audioBuffer.write(0);
    }
}

void Synthesizer::initializeImpulseResponse(
    const int16_t *impulseResponse,
    unsigned int samples,
    float volume,
    int index)
{
    unsigned int clippedLength = 0;
    for (unsigned int i = 0; i < samples; ++i) {
        if (std::abs(impulseResponse[i]) > 100) {
            clippedLength = i + 1;
        }
    }

    const unsigned int sampleCount = std::min(10000U, clippedLength);
    m_filters[index].convolution.initialize(sampleCount);
    for (unsigned int i = 0; i < sampleCount; ++i) {
        m_filters[index].convolution.getImpulseResponse()[i] =
            volume * impulseResponse[i] / INT16_MAX;
    }
}

void Synthesizer::startAudioRenderingThread() {
    m_run = true;
    m_thread = new std::thread(&Synthesizer::audioRenderingThread, this);
}

void Synthesizer::endAudioRenderingThread() {
    if (m_thread != nullptr) {
        m_run = false;
        endInputBlock();

        m_thread->join();
        delete m_thread;

        m_thread = nullptr;
    }
}

void Synthesizer::destroy() {
    m_audioBuffer.destroy();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.destroy();
        m_filters[i].convolution.destroy();
    }

    delete[] m_inputChannels;
    delete[] m_filters;

    m_clackTriggerData.destroy();
    delete[] m_clackTriggerTransfer;
    delete[] m_clackSample;
    m_clackTriggerTransfer = nullptr;
    m_clackSample = nullptr;
    m_clackSampleLength = 0;

    m_inputChannels = nullptr;
    m_filters = nullptr;

    m_inputChannelCount = 0;
}

int Synthesizer::readAudioOutput(int samples, int16_t *buffer) {
    std::lock_guard<std::mutex> lock(m_lock0);

    const int newDataLength = m_audioBuffer.size();
    if (newDataLength >= samples) {
        m_audioBuffer.readAndRemove(samples, buffer);
    }
    else {
        m_audioBuffer.readAndRemove(newDataLength, buffer);
        memset(
            buffer + newDataLength,
            0,
            sizeof(int16_t) * ((size_t)samples - newDataLength));
    }
    
    const int samplesConsumed = std::min(samples, newDataLength);

    return samplesConsumed;
}

void Synthesizer::waitProcessed() {
    {
        std::unique_lock<std::mutex> lk(m_lock0);
        m_cv0.wait(lk, [this] { return m_processed; });
    }
}

void Synthesizer::writeInput(const double *data) {
    m_inputWriteOffset += (double)m_audioSampleRate / m_inputSampleRate;
    if (m_inputWriteOffset >= (double)m_inputBufferSize) {
        m_inputWriteOffset -= (double)m_inputBufferSize;
    }

    for (int i = 0; i < m_inputChannelCount; ++i) {
        RingBuffer<float> &buffer = m_inputChannels[i].data;
        const double lastInputSample = m_inputChannels[i].lastInputSample;
        const size_t baseIndex = buffer.writeIndex();
        const double distance =
            inputDistance(m_inputWriteOffset, m_lastInputSampleOffset);
        double s =
            inputDistance(baseIndex, m_lastInputSampleOffset);
        for (; s <= distance; s += 1.0) {
            if (s >= m_inputBufferSize) s -= m_inputBufferSize;

            const double f = s / distance;
            const double sample = lastInputSample * (1 - f) + data[i] * f;

            buffer.write(m_filters[i].antialiasing.fast_f(static_cast<float>(sample)));
        }

        m_inputChannels[i].lastInputSample = data[i];
    }

    {
        const size_t baseIndex = m_clackTriggerData.writeIndex();
        const double distance =
            inputDistance(m_inputWriteOffset, m_lastInputSampleOffset);
        double s = inputDistance(baseIndex, m_lastInputSampleOffset);
        bool written = false;
        for (; s <= distance; s += 1.0) {
            if (s >= m_inputBufferSize) s -= m_inputBufferSize;
            m_clackTriggerData.write(written ? 0.0f : m_pendingClack);
            written = true;
        }

        if (written) m_pendingClack = 0.0f;
    }

    m_lastInputSampleOffset = m_inputWriteOffset;
}

void Synthesizer::endInputBlock() {
    std::unique_lock<std::mutex> lk(m_inputLock); 

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.removeBeginning(m_inputSamplesRead);
    }
    m_clackTriggerData.removeBeginning(m_inputSamplesRead);

    if (m_inputChannelCount != 0) {
        m_latency = m_inputChannels[0].data.size();
    }
    
    m_inputSamplesRead = 0;
    m_processed = false;

    lk.unlock();
    m_cv0.notify_one();
}

void Synthesizer::audioRenderingThread() {
    while (m_run) {
        renderAudio();
    }
}

#undef max
void Synthesizer::renderAudio() {
    std::unique_lock<std::mutex> lk0(m_lock0);

    m_cv0.wait(lk0, [this] {
        const bool inputAvailable =
            m_inputChannels[0].data.size() > 0
            && m_audioBuffer.size() < 2000;
        return !m_run || (inputAvailable && !m_processed);
    });

    const int n = std::min(
        std::max(0, 2000 - (int)m_audioBuffer.size()),
        (int)m_inputChannels[0].data.size());

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.read(n, m_inputChannels[i].transferBuffer);
    }
    m_clackTriggerData.read(n, m_clackTriggerTransfer);
    
    m_inputSamplesRead = n;
    m_processed = true;

    lk0.unlock();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            static_cast<float>(m_audioParameters.airNoiseFrequencyCutoff), m_audioSampleRate);
        m_filters[i].jitterFilter.setJitterScale(m_audioParameters.inputSampleNoise);
    }

    for (int i = 0; i < n; ++i) {
        m_audioBuffer.write(renderAudio(i));
    }

    m_cv0.notify_one();
}

double Synthesizer::getLatency() const {
    return (double)m_latency / m_audioSampleRate;
}

int Synthesizer::inputDelta(int s1, int s0) const {
    return (s1 < s0)
        ? m_inputBufferSize - s0 + s1
        : s1 - s0;
}

double Synthesizer::inputDistance(double s1, double s0) const {
    return (s1 < s0)
        ? (double)m_inputBufferSize - s0 + s1
        : s1 - s0;
}

void Synthesizer::setInputSampleRate(double sampleRate) {
    if (sampleRate != m_inputSampleRate) {
        std::lock_guard<std::mutex> lock(m_lock0);
        m_inputSampleRate = sampleRate;
    }
}

int16_t Synthesizer::renderAudio(int inputSample) {
    const float airNoise = m_audioParameters.airNoise;
    const float dF_F_mix = m_audioParameters.dF_F_mix;
    const float convAmount = m_audioParameters.convolution;

    float signal = 0;
    for (int i = 0; i < m_inputChannelCount; ++i) {
        const float r_0 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;

        const float jitteredSample =
            m_filters[i].jitterFilter.fast_f(m_inputChannels[i].transferBuffer[inputSample]);

        const float f_in = jitteredSample;
        const float f_dc = m_filters[i].inputDcFilter.fast_f(f_in);
        const float f = f_in - f_dc;
        const float f_p = m_filters[i].derivative.f(f_in);

        const float noise = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        const float r =
            m_filters->airNoiseLowPass.fast_f(noise);
        const float r_mixed =
            airNoise * r + (1 - airNoise);

        float v_in =
            f_p * dF_F_mix
            + f * r_mixed * (1 - dF_F_mix);
        if (fpclassify(v_in) == FP_SUBNORMAL) {
            v_in = 0;
        }

        const float v =
            convAmount * m_filters[i].convolution.f(v_in)
            + (1 - convAmount) * v_in;

        signal += v;
    }

    signal = m_antialiasing.fast_f(signal);

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    const float v_leveled =
        (m_levelingFilter.f(signal)
            + renderTurbo()
            + renderClack(m_clackTriggerTransfer[inputSample]))
        * m_audioParameters.volume;
    int r_int = std::lround(v_leveled);
    if (r_int > INT16_MAX) {
        r_int = INT16_MAX;
    }
    else if (r_int < INT16_MIN) {
        r_int = INT16_MIN;
    }

    return static_cast<int16_t>(r_int);
}

void Synthesizer::initializeClackSample(
    const int16_t *samples,
    unsigned int count,
    float volume)
{
    delete[] m_clackSample;
    m_clackSample = nullptr;
    m_clackSampleLength = 0;

    if (samples == nullptr || count == 0 || volume <= 0) return;

    m_clackSample = new float[count];
    for (unsigned int i = 0; i < count; ++i) {
        m_clackSample[i] = samples[i] / (float)INT16_MAX;
    }

    m_clackSampleLength = count;
    m_clackVolume = volume;
}

void Synthesizer::triggerClack(float amplitude) {
    if (m_clackSampleLength == 0) return;
    if (amplitude > m_pendingClack) m_pendingClack = amplitude;
}

float Synthesizer::renderClack(float trigger) {
    if (m_clackSampleLength == 0) return 0.0f;

    if (trigger > 0) {
        // Start a new voice (steal the oldest if all are busy)
        int slot = 0;
        double oldest = -1.0;
        for (int i = 0; i < ClackVoices; ++i) {
            if (m_clackVoices[i].position < 0) { slot = i; oldest = -2.0; break; }
            if (m_clackVoices[i].position > oldest) { oldest = m_clackVoices[i].position; slot = i; }
        }

        // Small per-hit pitch and level variation so it doesn't sound looped
        m_noiseState ^= m_noiseState << 13;
        m_noiseState ^= m_noiseState >> 17;
        m_noiseState ^= m_noiseState << 5;
        const double r0 = (double)m_noiseState / 4294967295.0;
        m_noiseState ^= m_noiseState << 13;
        m_noiseState ^= m_noiseState >> 17;
        m_noiseState ^= m_noiseState << 5;
        const double r1 = (double)m_noiseState / 4294967295.0;

        m_clackVoices[slot].position = 0.0;
        m_clackVoices[slot].rate = 0.94 + 0.12 * r0;
        m_clackVoices[slot].amplitude = trigger * (float)(0.75 + 0.25 * r1);
    }

    float out = 0.0f;
    for (int i = 0; i < ClackVoices; ++i) {
        ClackVoice &v = m_clackVoices[i];
        if (v.position < 0) continue;

        const unsigned int i0 = (unsigned int)v.position;
        if (i0 + 1 >= m_clackSampleLength) {
            v.position = -1.0;
            continue;
        }

        const float f = (float)(v.position - i0);
        out += v.amplitude * (m_clackSample[i0] * (1 - f) + m_clackSample[i0 + 1] * f);
        v.position += v.rate;
    }

    return 9000.0f * m_clackVolume * out;
}

void Synthesizer::setTurboAudio(const TurboAudio &audio) {
    m_turboWhistleFrequency = audio.whistleFrequency;
    m_turboWhistleAmplitude = audio.whistleAmplitude;
    m_turboHissAmplitude = audio.hissAmplitude;
    m_turboBlowOffAmplitude = audio.blowOffAmplitude;
    m_turboFlutterAmplitude = audio.flutterAmplitude;
}

float Synthesizer::renderTurbo() {
    constexpr double TwoPi = 6.283185307179586;
    const float sampleRate = m_audioSampleRate;

    const float targetFrequency = m_turboWhistleFrequency.load(std::memory_order_relaxed);
    const float targetWhistle = m_turboWhistleAmplitude.load(std::memory_order_relaxed);
    const float targetHiss = m_turboHissAmplitude.load(std::memory_order_relaxed);
    const float targetBlowOff = m_turboBlowOffAmplitude.load(std::memory_order_relaxed);
    const float targetFlutter = m_turboFlutterAmplitude.load(std::memory_order_relaxed);

    if (targetWhistle <= 0 && targetHiss <= 0 && targetBlowOff <= 0 && targetFlutter <= 0
        && m_smoothWhistleAmplitude < 1E-4f && m_smoothHissAmplitude < 1E-4f
        && m_smoothBlowOffAmplitude < 1E-4f && m_smoothFlutterAmplitude < 1E-4f)
    {
        return 0.0f;
    }

    // Parameter smoothing (~15 ms) so the sim's frame-rate updates don't zipper
    const float k = 1.0f - std::exp(-1.0f / (0.015f * sampleRate));
    if (m_smoothWhistleFrequency <= 0) m_smoothWhistleFrequency = targetFrequency;
    m_smoothWhistleFrequency += (targetFrequency - m_smoothWhistleFrequency) * k;
    m_smoothWhistleAmplitude += (targetWhistle - m_smoothWhistleAmplitude) * k;
    m_smoothHissAmplitude += (targetHiss - m_smoothHissAmplitude) * k;
    m_smoothBlowOffAmplitude += (targetBlowOff - m_smoothBlowOffAmplitude) * (k * 4.0f);
    m_smoothFlutterAmplitude += (targetFlutter - m_smoothFlutterAmplitude) * (k * 4.0f);

    // White noise (xorshift)
    auto noise = [this]() {
        m_noiseState ^= m_noiseState << 13;
        m_noiseState ^= m_noiseState >> 17;
        m_noiseState ^= m_noiseState << 5;
        return (float)((double)m_noiseState / 4294967295.0 * 2.0 - 1.0);
    };

    // Whistle: compressor tone with a little wobble and a second harmonic
    m_vibratoPhase += TwoPi * 6.5 / sampleRate;
    if (m_vibratoPhase > TwoPi) m_vibratoPhase -= TwoPi;
    const double frequency =
        m_smoothWhistleFrequency * (1.0 + 0.004 * std::sin(m_vibratoPhase));
    m_whistlePhase += TwoPi * frequency / sampleRate;
    if (m_whistlePhase > TwoPi) m_whistlePhase -= TwoPi;
    const float whistle =
        (float)(std::sin(m_whistlePhase)
            + 0.22 * std::sin(2.0 * m_whistlePhase + 0.7)
            + 0.08 * std::sin(3.0 * m_whistlePhase + 1.3));

    // Intake hiss: band-passed noise around the whistle
    const float n0 = noise();
    const float hissLpK = 1.0f - std::exp(-(float)TwoPi * 6000.0f / sampleRate);
    const float hissHpK = 1.0f - std::exp(-(float)TwoPi * 1500.0f / sampleRate);
    m_hissLp += (n0 - m_hissLp) * hissLpK;
    m_hissHp += (m_hissLp - m_hissHp) * hissHpK;
    const float hiss = m_hissLp - m_hissHp;

    // Blow-off valve: bright "pssh" burst
    const float n1 = noise();
    const float bovLpK = 1.0f - std::exp(-(float)TwoPi * 9000.0f / sampleRate);
    const float bovHpK = 1.0f - std::exp(-(float)TwoPi * 1800.0f / sampleRate);
    m_bovLp += (n1 - m_bovLp) * bovLpK;
    m_bovHp += (m_bovLp - m_bovHp) * bovHpK;
    const float bov = m_bovLp - m_bovHp;

    // Compressor surge: chopped low-mid noise ("stu-tu-tu")
    const float n2 = noise();
    const float flutterLpK = 1.0f - std::exp(-(float)TwoPi * 1400.0f / sampleRate);
    m_flutterLp += (n2 - m_flutterLp) * flutterLpK;
    const double flutterRate = 14.0 + 10.0 * m_smoothFlutterAmplitude;
    m_flutterPhase += TwoPi * flutterRate / sampleRate;
    if (m_flutterPhase > TwoPi) m_flutterPhase -= TwoPi;
    const float chop = (float)std::pow(0.5 + 0.5 * std::sin(m_flutterPhase), 6.0);
    const float flutter = m_flutterLp * chop;

    return
        2200.0f * m_smoothWhistleAmplitude * whistle
        + 2600.0f * m_smoothHissAmplitude * hiss
        + 9000.0f * m_smoothBlowOffAmplitude * bov
        + 12000.0f * m_smoothFlutterAmplitude * flutter;
}

double Synthesizer::getLevelerGain() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_levelingFilter.getAttenuation();
}

Synthesizer::AudioParameters Synthesizer::getAudioParameters() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_audioParameters;
}

void Synthesizer::setAudioParameters(const AudioParameters &params) {
    std::lock_guard<std::mutex> lock(m_lock0);
    m_audioParameters = params;
}
