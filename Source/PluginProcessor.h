/*
 eStickSimulator Plugin Processor
 
 Authors:
 Luca Bondi (bondi.luca@gmail.com)
 */

#pragma once

#include "../JuceLibraryCode/JuceHeader.h"
#include "Beamformer.h"

//==============================================================================

class EstickSimAudioProcessor :
public AudioProcessor,
public AudioProcessorValueTreeState::Listener {
public:
    
    //==============================================================================
    // JUCE plugin methods
    
    EstickSimAudioProcessor();
    
    ~EstickSimAudioProcessor();
    
    const String getName() const override;
    
    bool acceptsMidi() const override;
    
    bool producesMidi() const override;
    
    bool isMidiEffect() const override;
    
    double getTailLengthSeconds() const override;
    
    bool isBusesLayoutSupported(const BusesLayout &layouts) const override;
    
    void prepareToPlay(double sampleRate, int maximumExpectedSamplesPerBlock) override;
    
    void processBlock(AudioBuffer<float> &, MidiBuffer &) override;
    
    void releaseResources() override;
    
    int getNumPrograms() override;
    
    int getCurrentProgram() override;
    
    void setCurrentProgram(int index) override;
    
    const String getProgramName(int index) override;
    
    void changeProgramName(int index, const String &newName) override;
    
    AudioProcessorEditor *createEditor() override;
    
    bool hasEditor() const override;
    
    void getStateInformation(MemoryBlock &destData) override;
    
    void setStateInformation(const void *data, int sizeInBytes) override;
    
private:
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EstickSimAudioProcessor)
    
    //==============================================================================
    /** Number of active input channels */
    juce::uint32 numActiveInputChannels = 0;
    /** Number of active output channels */
    juce::uint32 numActiveOutputChannels = 0;
    
    //==============================================================================
    /** Time Constant for input gain variations */
    const float gainTimeConst = 0.1;
    /** Beam gain for each beam */
    dsp::Gain<float> sourceGain[NUM_SOURCES];
    
    //==============================================================================
    /** Previous HPF cut frequency */
    float prevHpfFreq = 0;
    /** Coefficients of the IIR HPF */
    IIRCoefficients iirCoeffHPF;
    /** IIR HPF */
    std::vector<IIRFilter> iirHPFfilters;
    
    //==============================================================================
    /** The active beamformer */
    std::unique_ptr<Beamformer> beamformer;
    
    //==============================================================================
    /** Lock to prevent releaseResources being called when processBlock is running. AudioPluginHost does it. */
    SpinLock processingLock;
    
    /** Resources for runtime are allocated.
     
     This flag is used to compensate for out-of-order calls to prepareToPlay, processBlock and releaseResources
     */
    bool resourcesAllocated = false;
    
    /** Sample rate [Hz] */
    float sampleRate = 48000;
    
    /** Maximum number of samples per block */
    int maximumExpectedSamplesPerBlock = 4096;
    
    //==============================================================================
    /** Set a new microphone configuration */
    void setMicConfig(const MicConfig &mc);
    
    //==============================================================================
    
    /** Measured average load */
    float load = 0;
    /** Load time constant [s] */
    const float loadTimeConst = 1;
    /** Load update factor (the higher the faster the update) */
    float loadAlpha = 1;
    /** Load lock */
    SpinLock loadLock;
    
    //==============================================================================
    
    /** Processor parameters tree */
    AudioProcessorValueTreeState parameters;
    
    //==============================================================================
    // VST parameters
    std::atomic<float> *steerXParam[NUM_SOURCES];
    std::atomic<float> *steerYParam[NUM_SOURCES];
    std::atomic<float> *levelParam[NUM_SOURCES];
    std::atomic<float> *muteParam[NUM_SOURCES];
    std::atomic<float> *hpfParam;
    std::atomic<float> *configParam;
    
    void parameterChanged(const String &parameterID, float newValue) override;
    
};
