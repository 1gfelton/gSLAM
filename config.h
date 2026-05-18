#pragma once
#include <random>
#include <fstream>
#include <nlohmann/json.hpp>
namespace CONFIG {
    inline std::ifstream f("config.json");
    inline nlohmann::json cfg = nlohmann::json::parse(f);
    /* $\Delta t$ */
    inline double DT        = cfg["DT"];
    /* $v$ */
    inline double ALPHA_1   = cfg["ALPHA_1"];
    inline double ALPHA_2   = cfg["ALPHA_2"];
    /* $\omega$ */
    inline double ALPHA_3   = cfg["ALPHA_3"];
    inline double ALPHA_4   = cfg["ALPHA_4"];
    /* $\hat{\gamma}$ */
    inline double ALPHA_5   = cfg["ALPHA_5"];
    inline double ALPHA_6   = cfg["ALPHA_6"];

    inline double N_SAMPLES = cfg["N_SAMPLES"];
    inline double THETA     = cfg["THETA"];
    inline double W         = cfg["W"];
    inline double V         = cfg["V"];
}