#pragma once
#include <iostream>
#include <cstdarg>
#include <thread>

#include "Falcor.h"
#include "Renderer.h"


int main(int argc, char** argv)
{
    Falcor::SampleAppConfig config;
    config.windowDesc.title = "HelloDXR";
    config.windowDesc.resizableWindow = true;

    Renderer renderer(config);
    return renderer.run();
}