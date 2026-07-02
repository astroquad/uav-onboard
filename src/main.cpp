#include "app/AstroquadOnboardApp.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv)
{
    try {
        onboard::app::AstroquadOnboardApp app;
        return app.run(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "[fatal] " << error.what() << '\n';
        return 1;
    }
}
