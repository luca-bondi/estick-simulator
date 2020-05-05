/*
 eStickSimulator Plugin Processor
 
 Authors:
 Luca Bondi (bondi.luca@gmail.com)
 */

#include "PluginProcessor.h"

//==============================================================================
// Helper functions
AudioProcessorValueTreeState::ParameterLayout initializeParameters() {
    
    std::vector<std::unique_ptr<RangedAudioParameter>> params;
    
    // Values in dB
    params.push_back(std::make_unique<AudioParameterChoice>("config", //tag
                                                            "Configuration", //name
                                                            micConfigLabels, //choices
                                                            0 //default
                                                            ));
    
    
    // Values in Hz
    params.push_back(std::make_unique<AudioParameterFloat>("hpf", //tag
                                                           "HPF",
                                                           20.0f, //min
                                                           500.0f, //max
                                                           250.0f //default
                                                           ));
    
    {
        for (auto srcIdx = 0; srcIdx < NUM_SOURCES; ++srcIdx) {
            auto defaultDirectionX = srcIdx == 0 ? -0.5 : 0.5;
            params.push_back(std::make_unique<AudioParameterFloat>("steerX" + String(srcIdx + 1), //tag
                                                                   "Steer " + String(srcIdx + 1) + " hor", //name
                                                                   -1.0f, //min
                                                                   1.0f, //max
                                                                   defaultDirectionX //default
                                                                   ));
            params.push_back(std::make_unique<AudioParameterFloat>("steerY" + String(srcIdx + 1), //tag
                                                                   "Steer " + String(srcIdx + 1) + " ver", //name
                                                                   -1.0f, //min
                                                                   1.0f, //max
                                                                   0 //default
                                                                   ));

            params.push_back(std::make_unique<AudioParameterFloat>("level" + String(srcIdx + 1), //tag
                                                                   "Level " + String(srcIdx + 1), //name
                                                                   -10.0f, //min
                                                                   10.0f, //max
                                                                   0.0f //default
                                                                   ));
            
            params.push_back(std::make_unique<AudioParameterBool>("mute" + String(srcIdx + 1), //tag
                                                                  "Mute " + String(srcIdx + 1), //name
                                                                  false //default
                                                                  ));
            
        }
    }
    
    return {params.begin(), params.end()};
}


//==============================================================================
EstickSimAudioProcessor::EstickSimAudioProcessor()
: AudioProcessor(BusesProperties() //The default bus layout accommodates for 4 buses of 16 channels each.
                 .withOutput("eStick#1", AudioChannelSet::ambisonic(3), true)
                 .withOutput("eStick#2", AudioChannelSet::ambisonic(3), true)
                 .withOutput("eStick#3", AudioChannelSet::ambisonic(3), true)
                 .withOutput("eStick#4", AudioChannelSet::ambisonic(3), true)
                 .withInput("Input", AudioChannelSet::stereo(), true)
                 ), parameters(*this, nullptr, Identifier("eStickSimParams"), initializeParameters()) {
    
    /** Get parameters pointers */
    configParam = parameters.getRawParameterValue("config");
    parameters.addParameterListener("config", this);
    hpfParam = parameters.getRawParameterValue("hpf");
    
    for (auto srcIdx = 0; srcIdx < NUM_SOURCES; srcIdx++) {
        steerXParam[srcIdx] = parameters.getRawParameterValue("steerX" + String(srcIdx + 1));
        steerYParam[srcIdx] = parameters.getRawParameterValue("steerY" + String(srcIdx + 1));
        levelParam[srcIdx] = parameters.getRawParameterValue("level" + String(srcIdx + 1));
        muteParam[srcIdx] = parameters.getRawParameterValue("mute" + String(srcIdx + 1));
    }
    
}

//==============================================================================
bool EstickSimAudioProcessor::isBusesLayoutSupported(const BusesLayout &layouts) const {
    // This plug-in supports up to 4 eSticks, for a total amount of 64 channels in output.
    // VST3 allows for a maximum of 25 channels per bus.
    // To make things simpler in terms of patching, each output bus counts for at most 16 channels.
    // This configuration allows REAPER to be configured with a 64 channels track.
    for (auto bus : layouts.inputBuses) {
        if (bus.size() > 16) {
            return false;
        }
    }
    for (auto bus : layouts.outputBuses) {
        if (bus.size() > 16) {
            // We have to allow the output bus to grow to the size of the input bus for compatibility with REAPER
            return false;
        }
    }
    if ((layouts.getMainInputChannels() < 2) || (layouts.getMainOutputChannels() < 2)) {
        // In any case don't allow less than 2 input and 2 output channels
        return false;
    }
    return true;
}

//==============================================================================
void EstickSimAudioProcessor::prepareToPlay(double sampleRate_, int maximumExpectedSamplesPerBlock_) {
    
    GenericScopedLock<SpinLock> lock(processingLock);
    
    sampleRate = sampleRate_;
    maximumExpectedSamplesPerBlock = maximumExpectedSamplesPerBlock_;
    
    /** Number of active input channels */
    numActiveInputChannels = jmin(NUM_SOURCES,getTotalNumInputChannels());
    
    /** Number of active output channels */
    numActiveOutputChannels = getTotalNumOutputChannels();
    
    
    /** Initialize the High Pass Filters */
    iirHPFfilters.clear();
    iirHPFfilters.resize(numActiveInputChannels);
    prevHpfFreq = 0;
    
    /** Initialize the beamformer */
    beamformer = std::make_unique<Beamformer>(NUM_SOURCES, static_cast<MicConfig>((int) *configParam),sampleRate, maximumExpectedSamplesPerBlock);
    
    /** Initialize level gains */
    for (auto srcIdx = 0; srcIdx < NUM_SOURCES; ++srcIdx) {
        sourceGain[srcIdx].reset();
        sourceGain[srcIdx].prepare({sampleRate, static_cast<uint32>(maximumExpectedSamplesPerBlock), 1});
        sourceGain[srcIdx].setGainDecibels(*levelParam[srcIdx]);
        sourceGain[srcIdx].setRampDurationSeconds(gainTimeConst);
    }
    
    resourcesAllocated = true;
    
    /** Time constants */
    loadAlpha = 1 - exp(-(maximumExpectedSamplesPerBlock / sampleRate) / loadTimeConst);
    
}

void EstickSimAudioProcessor::releaseResources() {
    
    GenericScopedLock<SpinLock> lock(processingLock);
    
    resourcesAllocated = false;
    
    /** Clear the HPF */
    iirHPFfilters.clear();
    
    /** Clear the Beamformer */
    beamformer.reset();
}


void EstickSimAudioProcessor::processBlock(AudioBuffer<float> &buffer, MidiBuffer &midiMessages) {
    
    const auto startTick = Time::getHighResolutionTicks();
    
    GenericScopedLock<SpinLock> lock(processingLock);
    
    /** If resources are not allocated this is an out-of-order request */
    if (!resourcesAllocated) {
        jassertfalse;
        return;
    }
    
    ScopedNoDenormals noDenormals;
    
    /** Renew IIR coefficient if cut frequency changed */
    if (prevHpfFreq != (bool) *hpfParam) {
        iirCoeffHPF = IIRCoefficients::makeHighPass(sampleRate, *hpfParam);
        prevHpfFreq = *hpfParam;
        for (auto &iirHPFfilter : iirHPFfilters) {
            iirHPFfilter.setCoefficients(iirCoeffHPF);
        }
    }
    
    /**Apply input gain directly on input buffer  */
    for (auto srcIdx = 0; srcIdx < numActiveInputChannels; srcIdx++){
        if ((bool)*muteParam[srcIdx]){
            buffer.clear(srcIdx, 0, buffer.getNumSamples());
        }else{
            sourceGain[srcIdx].setGainDecibels( *levelParam[srcIdx]);
            auto block = juce::dsp::AudioBlock<float>(buffer).getSubsetChannelBlock(srcIdx, 1);
            auto context = juce::dsp::ProcessContextReplacing<float>(block);
            sourceGain[srcIdx].process(context);
        }
    }
    
    /**Apply HPF directly on input buffer  */
    for (auto inChannel = 0; inChannel < numActiveInputChannels; ++inChannel) {
        iirHPFfilters[inChannel].processSamples(buffer.getWritePointer(inChannel), buffer.getNumSamples());
    }
    
    /** Set parameters */
    for (auto srcIdx = 0; srcIdx < NUM_SOURCES; srcIdx++) {
        float beamDoaX = -*steerXParam[srcIdx];
        float beamDoaY = (*steerYParam[srcIdx]);
        BeamParameters params = {beamDoaX,beamDoaY, 0};
        beamformer->setParams(srcIdx, params);
    }
    
    /** Call the beamformer  */
    beamformer->processBlock(buffer);
    
    /** Clear buffer */
    buffer.clear();
    
    /** Retrieve beamformer outputs */
    beamformer->getOutput(buffer);
    
    /** Update load */
    {
        const float elapsedTime = Time::highResolutionTicksToSeconds(Time::getHighResolutionTicks() - startTick);
        const float curLoad = elapsedTime / (maximumExpectedSamplesPerBlock / sampleRate);
        GenericScopedLock<SpinLock> lock(loadLock);
        load = (load * (1 - loadAlpha)) + (curLoad * loadAlpha);
    }
    
}

//==============================================================================

void EstickSimAudioProcessor::parameterChanged(const String &parameterID, float newValue) {
    if (parameterID == "config") {
        setMicConfig(static_cast<MicConfig>((int) (newValue)));
    }
}

//==============================================================================
void EstickSimAudioProcessor::setMicConfig(const MicConfig &mc) {
    prepareToPlay(sampleRate, maximumExpectedSamplesPerBlock);
}

//==============================================================================
void EstickSimAudioProcessor::getStateInformation(MemoryBlock &destData) {
    /** Root XML */
    std::unique_ptr<XmlElement> xml(new XmlElement("eStickSimRoot"));
    
    /** Parameters state */
    auto state = parameters.copyState();
    XmlElement *xmlParams = new XmlElement(*state.createXml());
    xml->addChildElement(xmlParams);
    
    copyXmlToBinary(*xml, destData);
}

void EstickSimAudioProcessor::setStateInformation(const void *data, int sizeInBytes) {
    
    std::unique_ptr<XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));
    
    if (xmlState.get() != nullptr) {
        if (xmlState->hasTagName("eStickSimRoot")) {
            forEachXmlChildElement (*xmlState, rootElement) {
                if (rootElement->hasTagName(parameters.state.getType())) {
                    /** Parameters state */
                    parameters.replaceState(ValueTree::fromXml(*rootElement));
                }
            }
        }
    }
}

//==============================================================================
// Unchanged JUCE default functions
EstickSimAudioProcessor::~EstickSimAudioProcessor() {
}

const String EstickSimAudioProcessor::getName() const {
    return JucePlugin_Name;
}

bool EstickSimAudioProcessor::acceptsMidi() const {
#if JucePlugin_WantsMidiInput
    return true;
#else
    return false;
#endif
}

bool EstickSimAudioProcessor::producesMidi() const {
#if JucePlugin_ProducesMidiOutput
    return true;
#else
    return false;
#endif
}

bool EstickSimAudioProcessor::isMidiEffect() const {
#if JucePlugin_IsMidiEffect
    return true;
#else
    return false;
#endif
}

double EstickSimAudioProcessor::getTailLengthSeconds() const {
    return 0.0;
}

int EstickSimAudioProcessor::getNumPrograms() {
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
    // so this should be at least 1, even if you're not really implementing programs.
}

int EstickSimAudioProcessor::getCurrentProgram() {
    return 0;
}

void EstickSimAudioProcessor::setCurrentProgram(int index) {
}

const String EstickSimAudioProcessor::getProgramName(int index) {
    return {};
}

void EstickSimAudioProcessor::changeProgramName(int index, const String &newName) {
}

AudioProcessor *JUCE_CALLTYPE createPluginFilter() {
    return new EstickSimAudioProcessor();
}

bool EstickSimAudioProcessor::hasEditor() const {
    return false;
}

AudioProcessorEditor * EstickSimAudioProcessor::createEditor(){
    return nullptr;
};
