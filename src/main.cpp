#include <iostream>

int main(int argc, char** argv)
{
    std::cout << "uav_onboard scaffold\n";
    std::cout << "args:";
    for (int i = 1; i < argc; ++i) {
        std::cout << ' ' << argv[i];
    }
    std::cout << '\n';
    return 0;
}
