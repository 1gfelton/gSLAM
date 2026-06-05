#pragma once
#include "utils.h"
#include <random>
#include <fstream>
#include <math.h>
#include <nlohmann/json.hpp>
namespace CONFIG {
    inline std::ifstream f("config.json");
    inline nlohmann::json cfg = nlohmann::json::parse(f);
    /* $\Delta t$ */
    inline double DT            = cfg["DT"];
    /* $v$ */
    inline double ALPHA_1       = cfg["ALPHA_1"];
    inline double ALPHA_2       = cfg["ALPHA_2"];
    /* $\omega$ */
    inline double ALPHA_3       = cfg["ALPHA_3"];
    inline double ALPHA_4       = cfg["ALPHA_4"];
    /* $\hat{\gamma}$ */
    inline double ALPHA_5       = cfg["ALPHA_5"];
    inline double ALPHA_6       = cfg["ALPHA_6"];

    /* Noise from the sensor readings for the range, heading, and label */
    /* $\varepsilon_{\sigma^r}$ */
    inline double SIGMA_R       = cfg["SIGMA_R"];
    /* $\varepsilon_{\sigma^\phi}$ */
    inline double SIGMA_PHI     = cfg["SIGMA_PHI"];
    /* $\varepsilon_{\sigma^s}$ */
    inline double SIGMA_S       = cfg["SIGMA_S"];

    inline double N_SAMPLES     = cfg["N_SAMPLES"];
    inline double THETA         = to_radians(cfg["THETA"]);
    inline double W             = cfg["W"]; // rad / s
    inline double V             = cfg["V"];
    inline int N_LANDMARKS      = cfg["N_LANDMARKS"];

    inline double SIGMA_X       = cfg["SIGMA_X"];
    inline double SIGMA_Y       = cfg["SIGMA_Y"];
    inline double SIGMA_THETA   = cfg["SIGMA_THETA"];

    inline int STATE_VEC_SIZE   = ((int)cfg["N_LANDMARKS"] * 3) + 3;
    inline int N_STEPS          = cfg["N_STEPS"]; 
    inline int N_POSES          = N_STEPS + 1; // plus 1 to account for the inital 0-pose [0, 0, 0]
}