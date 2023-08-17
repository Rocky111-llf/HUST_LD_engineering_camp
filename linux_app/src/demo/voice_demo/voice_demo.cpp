#include <iostream>

#include "voice_demo.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << std::string(argv[0]) << " wav_file_path" << std::endl;
        return -1;
    }

    std::string wav_file_name(argv[1]);
    
    play_sound(wav_file_name);

    return 0;
}