cmake_minimum_required(VERSION 3.22)

project(Curves VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Core DSP: the control string + filter bank (no dependencies) ----
add_library(curves_string STATIC
    src/string_sim.cpp
    src/filter_bank.cpp
)
target_include_directories(curves_string PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- Physics validation runner (mirrors prototype/run_scenarios.py) ----
add_executable(scenarios src/scenarios_main.cpp)
target_link_libraries(scenarios PRIVATE curves_string)

# ---- Audio-thread cost benchmark (diagnosing real-time glitches) ----
add_executable(bench src/bench_main.cpp)
target_link_libraries(bench PRIVATE curves_string)

add_executable(gain_test src/gain_test_main.cpp)
target_link_libraries(gain_test PRIVATE curves_string)

add_executable(tension_test src/tension_test_main.cpp)
target_link_libraries(tension_test PRIVATE curves_string)

add_executable(wind_test src/wind_test_main.cpp)
target_link_libraries(wind_test PRIVATE curves_string)

add_executable(display_test src/display_test_main.cpp)
target_link_libraries(display_test PRIVATE curves_string)

add_executable(spatial_test src/spatial_test_main.cpp)
target_link_libraries(spatial_test PRIVATE curves_string)

# ---- JUCE / CLAP plugin shell (M0 second half) ----
# enable via -DCURVES_BUILD_PLUGIN=ON; FetchContent pulls JUCE so the repo
# stays light. CLAP comes from clap-juce-extensions (JUCE has no native CLAP).
set(CURVES_BUILD_PLUGIN OFF CACHE BOOL "Build the Curves plugin (JUCE + CLAP)")
if(CURVES_BUILD_PLUGIN)
    include(FetchContent)
    FetchContent_Declare(
        JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG 8.0.8
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(
        clap_juce_extensions
        GIT_REPOSITORY https://github.com/free-audio/clap-juce-extensions.git
        GIT_TAG main
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(JUCE clap_juce_extensions)

    juce_add_plugin(Curves
        COMPANY_NAME "Curves"
        PLUGIN_MANUFACTURER_CODE Crvs
        PLUGIN_CODE Crv1
        FORMATS Standalone VST3
        PRODUCT_NAME "Curves"
        IS_SYNTH FALSE
        NEEDS_MIDI_INPUT FALSE
        COPY_PLUGIN_AFTER_BUILD TRUE  # auto-installs VST3 into Common Files for Reaper testing
    )

    # MIT-licensed CLAP target alongside the native JUCE formats.
    clap_juce_extensions_plugin(TARGET Curves
        CLAP_ID "com.curves.curves"
        CLAP_FEATURES audio-effect equalizer)

    target_sources(Curves PRIVATE
        src/plugin/PluginProcessor.cpp
        src/plugin/PluginEditor.cpp
    )
    target_compile_definitions(Curves PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0
    )
    target_link_libraries(Curves
        PRIVATE
            juce::juce_audio_utils
            juce::juce_dsp
            curves_string
        PUBLIC
            juce::juce_recommended_config_flags
            juce::juce_recommended_lto_flags
            juce::juce_recommended_warning_flags
    )
endif()
