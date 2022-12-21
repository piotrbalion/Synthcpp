all:
	clang++ --std=c++20 main1.cpp -DLIBREMIDI_COREAUDIO=1 -DLIBREMIDI_HEADER_ONLY=1 -I ~/libremidi/include -o synth -lSDL2 -framework CoreMIDI -framework CoreAudio -framework CoreFoundation

run: all
	./synth