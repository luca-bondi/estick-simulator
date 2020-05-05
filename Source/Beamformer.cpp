/*
 Beamforming processing class
 
 Authors:
 Luca Bondi (luca.bondi@polimi.it)
 */

#include "Beamformer.h"

// ==============================================================================
Beamformer::Beamformer(int numSources_, MicConfig mic, double sampleRate_, int maximumExpectedSamplesPerBlock_) {
    
    numSources = numSources_;
    micConfig = mic;
    sampleRate = sampleRate_;
    maximumExpectedSamplesPerBlock = maximumExpectedSamplesPerBlock_;
    
    /** Alpha for FIR update */
    alpha = 1 - exp(-(maximumExpectedSamplesPerBlock / sampleRate) / firUpdateTimeConst);
    
    firIR.resize(numSources);
    firFFT.resize(numSources);
    
    /** Distance between microphones in eSticks*/
    const float micDistX = 0.03;
    const float micDistY = 0.03;
    
    /** Determine configuration parameters */
    switch (micConfig) {
        case ULA_1ESTICK:
            numMic = 16;
            numRows = 1;
            break;
        case ULA_2ESTICK:
            numMic = 32;
            numRows = 1;
            break;
        case URA_2ESTICK:
            numMic = 32;
            numRows = 2;
            break;
        case ULA_3ESTICK:
            numMic = 48;
            numRows = 1;
            break;
        case URA_3ESTICK:
            numMic = 48;
            numRows = 3;
            break;
        case ULA_4ESTICK:
            numMic = 64;
            numRows = 1;
            break;
        case URA_4ESTICK:
            numMic = 64;
            numRows = 4;
            break;
        case URA_2x2ESTICK:
            numMic = 64;
            numRows = 2;
            break;
    }
    alg = std::make_unique<DAS::FarfieldURA>(micDistX, micDistY, numMic, numRows, sampleRate, soundspeed);
    
    firLen = alg->getFirLen();
    
    /** Create shared FFT object */
    fft = std::make_shared<juce::dsp::FFT>(ceil(log2(firLen + maximumExpectedSamplesPerBlock - 1)));
    
    /** Allocate FIR filters */
    for (auto &f : firIR) {
        f = AudioBuffer<float>(numMic, firLen);
        f.clear();
    }
    for (auto &f : firFFT) {
        f = AudioBufferFFT(numMic, fft);
        f.clear();
    }
    
    /** Allocate input buffers */
    inputBuffer = AudioBufferFFT(numSources, fft);
    
    /** Allocate convolution buffer */
    convolutionBuffer = AudioBufferFFT(1, fft);
    
    /** Allocate  output buffer */
    outBuffer.setSize(numMic, convolutionBuffer.getNumSamples() / 2);
    outBuffer.clear();
    
}

Beamformer::~Beamformer() {
}

MicConfig Beamformer::getMicConfig() const {
    return micConfig;
}

void Beamformer::setParams(int srcIdx, const BeamParameters &params) {
    if (alg == nullptr)
        return;
    alg->getFir(firIR[srcIdx], params, alpha);
    firFFT[srcIdx].setTimeSeries(firIR[srcIdx]);
    firFFT[srcIdx].prepareForConvolution();
}

void Beamformer::processBlock(const AudioBuffer<float> &inBuffer) {
        
    /** Compute inputs FFT */
    inputBuffer.setTimeSeries(inBuffer);
    inputBuffer.prepareForConvolution();
    
    for (auto srcIdx = 0; srcIdx < numSources; srcIdx++) {
        for (auto outCh = 0; outCh < numMic; outCh++) {
            /** Convolve inputs and FIR */
            convolutionBuffer.convolve(0, inputBuffer, srcIdx, firFFT[srcIdx], outCh);
            /** Overlap and add of convolutionBuffer into beamBuffer */
            convolutionBuffer.addToTimeSeries(0, outBuffer, outCh);
        }
    }
    
}

void Beamformer::getFir(AudioBuffer<float> &fir, const BeamParameters &params, float alpha) const {
    alg->getFir(fir, params, alpha);
}

void Beamformer::getOutput(AudioBuffer<float> &dst) {
    auto numSplsOut = dst.getNumSamples();
    auto numSplsShift = outBuffer.getNumSamples() - numSplsOut;
    for (auto outCh = 0; outCh < jmin(numMic,dst.getNumChannels()); outCh++) {
        /** Copy beamBuffer to outBuffer */
        dst.copyFrom(outCh, 0, outBuffer, outCh, 0, numSplsOut);
        /** Shift beamBuffer */
        FloatVectorOperations::copy(outBuffer.getWritePointer(outCh),
                                    outBuffer.getReadPointer(outCh) + numSplsOut, numSplsShift);
        outBuffer.clear(outCh, numSplsShift, outBuffer.getNumSamples() - numSplsShift);
    }
}
