/*
  Beamforming processing class
 
 Authors:
 Luca Bondi (luca.bondi@polimi.it)
*/

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "eStickSimDefs.h"
#include "AudioBufferFFT.h"
#include "BeamformingAlgorithms.h"



// ==============================================================================

class Beamformer;


// ==============================================================================

class Beamformer {

public:


    /** Initialize the Beamformer with a set of static parameters.
     @param numBeams: number of beams the beamformer has to compute
     @param mic: microphone configuration
     @param sampleRate:
     @param maximumExpectedSamplesPerBlock: 
     */
    Beamformer(int numBeams, MicConfig mic, double sampleRate, int maximumExpectedSamplesPerBlock);

    /** Destructor. */
    ~Beamformer();
    
    /** Get microphone configuration */
    MicConfig getMicConfig() const;

    /** Process a new block of samples.
     
     To be called inside AudioProcessor::processBlock.
     */
    void processBlock(const AudioBuffer<float> &inBuffer);

    /** Copy the current beams outputs to the provided output buffer
     
     To be called inside AudioProcessor::processBlock, after Beamformer::processBlock
     */
    void getOutput(AudioBuffer<float> &outBuffer);

    /** Set the parameters for a specific beam  */
    void setParams(int beamIdx, const BeamParameters &beamParams);

    /** Get FIR in time domain for a given direction of arrival
    
    @param fir: an AudioBuffer object with numChannels >= number of microphones and numSamples >= firLen
    @param params: beam parameters
    @param alpha: exponential interpolation coefficient. 1 means complete override (instant update), 0 means no override (complete preservation)
    */
    void getFir(AudioBuffer<float> &fir, const BeamParameters &params, float alpha = 1) const;


private:

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Beamformer);

    /** Sound speed [m/s] */
    const float soundspeed = 343;

    /** Sample rate [Hz] */
    float sampleRate = 48000;

    /** Maximum buffer size [samples] */
    int maximumExpectedSamplesPerBlock = 64;

    /** Number of microphones */
    int numMic = 16;
    
    /** Number of rows */
    int numRows = 1;

    /** Number of sources */
    int numSources;

    /** Beamforming algorithm */
    std::unique_ptr<BeamformingAlgorithm> alg;

    /** FIR filters length. Diepends on the algorithm */
    int firLen;

    /** Shared FFT pointer */
    std::shared_ptr<juce::dsp::FFT> fft;

    /** FIR filters for each beam */
    std::vector<AudioBuffer<float>> firIR;
    std::vector<AudioBufferFFT> firFFT;

    /** Inputs' buffer */
    AudioBufferFFT inputBuffer;

    /** Convolution buffer */
    AudioBufferFFT convolutionBuffer;

    /** Outputs buffer */
    AudioBuffer<float> outBuffer;

    /** FIR coefficients update time constant [s] */
    const float firUpdateTimeConst = 0.2;
    /** FIR coefficients update alpha */
    float alpha = 1;

    /** Microphones configuration */
    MicConfig micConfig = ULA_1ESTICK;

    /** Initialize the beamforming algorithm */
    void initAlg();


};
