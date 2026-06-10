#ifndef AUDIO_HPP
#define AUDIO_HPP

#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <portaudio.h>

class Audio
{
    private:
        PaStream* _stream;
        std::string _command;
        int         _deviceIndex;
        PaStreamParameters  _inputParameters;

    public:
        Audio();
        ~Audio();
        bool startAudioStream();
        int getMicIndex();
        void processAudio();
        static int audioCallback(const void* inputBuffer,
                             void* outputBuffer,
                             unsigned long framesPerBuffer,
                             const PaStreamCallbackTimeInfo* timeInfo,
                             PaStreamCallbackFlags statusFlags,
                             void* userData);
};


#endif