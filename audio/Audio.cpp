#include "Audio.hpp"

Audio::Audio()
    :
        _stream(nullptr),
        _command(""),
        _deviceIndex(-1),
        _inputParameters()
{
    
}


Audio::~Audio()
{
    Pa_Terminate();
}

int Audio::getMicIndex()
{
    int numDevices = Pa_GetDeviceCount();
    for (size_t i = 0; i < numDevices; i++)
    {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info->maxInputChannels > 0)
        {
            fprintf(stdout, "Device %d: %s\n", i, info->name);
            if (strstr(info->name, "ReSpeaker") != NULL)
            {
                fprintf(stdout, "Found ReSpeaker at index %d\n", i);
                return i;
            }
        }
    }
    fprintf(stderr, "ReSpeaker not found, using default\n");
    return Pa_GetDefaultInputDevice();
}

bool Audio::startAudioStream()
{
    PaError err = Pa_Initialize();
    if (err != paNoError)
    {
        fprintf(stderr, "PortAudio init failed: %s\n", Pa_GetErrorText(err));
        return false;
    }

    _deviceIndex = getMicIndex();
    _inputParameters.device = _deviceIndex;
    _inputParameters.channelCount = 1;
    _inputParameters.sampleFormat = paInt16;
    _inputParameters.suggestedLatency = Pa_GetDeviceInfo(_deviceIndex)->defaultLowInputLatency;
    _inputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(
        &_stream,
        &_inputParameters,
        NULL,           // no output
        16000,          // sample rate
        8000,           // frames per buffer
        paClipOff,
        audioCallback,
        this
    );


}