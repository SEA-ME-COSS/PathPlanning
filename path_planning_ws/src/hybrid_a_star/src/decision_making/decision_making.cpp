#include "decision_making/decision_making.hpp"

DecisionMaking::DecisionMaking(VehicleState current_state, float normal_throttle,
                std::vector<Sign> *signs, std::vector<Light> *lights, Pose *pose) {
    // Use Pointer
    this->signs = signs;
    this->lights = lights;
    this->pose = pose;

    // Initialize Parameters
    this->throttle = 0.0;
    this->current_state = current_state;
    this->normal_throttle = normal_throttle;

    // Timer
    this->crosswalknow_timecheck = false;
    this->crosswalknow_stoptime = std::chrono::steady_clock::time_point::min();

    this->crosswalkworthy_timecheck = false;
    this->crosswalkworthy_time = std::chrono::steady_clock::time_point::min();

    this->rotarynow_timecheck = false;
    this->rotarynow_stoptime = std::chrono::steady_clock::time_point::min();

    this->rotaryworthy_timecheck = false;
    this->rotaryworthy_time = std::chrono::steady_clock::time_point::min();

    // Initialization
    this->crosswalksign_ignore = 3;
    this->rotarysign_ignore = 5;
    this->sign_mindistance = 3.0;
    this->status_break_time = 10;

    this->trafficlight_status = false;
}

DecisionMaking::DecisionMaking() {
}

DecisionMaking::~DecisionMaking() {
}

void DecisionMaking::decide() {
    if(!lights->empty()) {
        TrafficLightStatusDecision();
        if (this->trafficlight_status) {return;}
    }
    
    if(!signs->empty()) {
        SignStatusDecision();
    }
    else {
        RacingState();
    }
}

void DecisionMaking::TrafficLightStatusDecision() {
    std::string light_info = (*lights)[0].id;
    float light_distance = (*lights)[0].distance;

    this->trafficlight_status = true;

    if (light_info == "traffic_red") {
        current_state = VehicleState::TrafficLightRedBefore;
        if(light_distance < this->sign_mindistance) {
            current_state = VehicleState::TrafficLightRedNow;
        }
    }
    else if (light_info == "traffic_yellow") {
        current_state = VehicleState::TrafficLightYellowBefore;
        if(light_distance < this->sign_mindistance) {
            current_state = VehicleState::TrafficLightYellowNow;
        }
    }
    else if (light_info == "traffic_green") {
        current_state = VehicleState::TrafficLightGreenBefore;
        if(light_distance < this->sign_mindistance) {
            current_state = VehicleState::TrafficLightGreenNow;
        }
    }
    else {
        this->trafficlight_status = false;
    }

    switch (current_state) {
        case VehicleState::TrafficLightRedBefore:
            TrafficLightRedBeforeState();
            break;
        case VehicleState::TrafficLightRedNow:
            TrafficLightRedNowState();
            break;
        case VehicleState::TrafficLightYellowBefore:
            TrafficLightYellowBeforeState();
            break;
        case VehicleState::TrafficLightYellowNow:
            TrafficLightYellowNowState();
            break;
        case VehicleState::TrafficLightGreenBefore:
            TrafficLightGreenBeforeState();
            break;
        case VehicleState::TrafficLightGreenNow:
            TrafficLightGreenNowState();
            break;
        default:
            break;
    }
}

void DecisionMaking::SignStatusDecision() {
    std::string sign_info = (*signs)[0].id;
    float sign_distance = (*signs)[0].distance;

    // Update State
    switch (current_state) {
        case VehicleState::Driving:
            if (sign_info=="None") { current_state = VehicleState::Driving; }
            else if (sign_info=="crosswalk") { current_state = VehicleState::CrosswalkBefore; }
            else if (sign_info=="rotary") { current_state = VehicleState::RoundAboutBefore; }
            else { current_state = VehicleState::Driving; }
            break;
        case VehicleState::CrosswalkBefore:
            if((sign_info == "crosswalk" && sign_distance < this->sign_mindistance)) {
                this->current_state = VehicleState::CrosswalkNow;
            }
            break;
        case VehicleState::CrosswalkNow:
            // Move slowly for "this->crosswalksign_ignore(default : 3.0)". After that change to Default
            break;
        case VehicleState::RoundAboutBefore:
            if((sign_info == "rotary" && sign_distance < this->sign_mindistance)) {
                this->current_state = VehicleState::RoundAboutNow;
            }
            break;
        case VehicleState::RoundAboutNow:
            break;
        default:
            current_state = VehicleState::Driving;
            break;
    }

    switch (current_state) {
        case VehicleState::Driving:
            DefaultState();
            break;
        case VehicleState::CrosswalkBefore:
            CrosswalkBeforeState();
            break;
        case VehicleState::CrosswalkNow:
            CrosswalkNowState();
            break;
        case VehicleState::RoundAboutBefore:
            RoundAboutBeforeState();
            break;
        case VehicleState::RoundAboutNow:
            RoundAboutNowState();
            break;
        default:
            DefaultState();
            break;
    }
}

void DecisionMaking::DefaultState() {
    std::cout << "Default Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::TrafficLightRedBeforeState() {
    std::cout << "Red Traffic Light Before Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::TrafficLightRedNowState() {
    std::cout << "Red Traffic Light Now Status" << std::endl;
    this->throttle = 0.0;
}

void DecisionMaking::TrafficLightYellowBeforeState() {
    std::cout << "Yellow Traffic Light Before Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::TrafficLightYellowNowState() {
    std::cout << "Yellow Traffic Light Now Status" << std::endl;
    this->throttle = 0.0;
}

void DecisionMaking::TrafficLightGreenBeforeState() {
    std::cout << "Green Traffic Light Before Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::TrafficLightGreenNowState() {
    std::cout << "Green Traffic Light Now Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::CrosswalkBeforeState() {
    std::cout << "CrossWalk Before Status" << std::endl;
    if(!isSignWorthy("rotary", this->crosswalkworthy_timecheck, this->crosswalkworthy_time)) {
        this->current_state = VehicleState::Driving;
    }
    this->throttle = this->normal_throttle;
}

void DecisionMaking::CrosswalkNowState() {
    std::cout << "CrossWalk Now Status" << std::endl;
    if (!this->crosswalknow_timecheck) {
        this->crosswalknow_timecheck = true;
        this->crosswalknow_stoptime= std::chrono::steady_clock::now();
        this->throttle = this->normal_throttle*0.5;
    } 
    else {
        auto now = std::chrono::steady_clock::now();
        auto time_gap = std::chrono::duration_cast<std::chrono::seconds>(now - this->crosswalknow_stoptime).count();
        if (time_gap >= this->crosswalksign_ignore) {
            this->current_state = VehicleState::Driving;
            this->crosswalknow_timecheck = false; 
        }
    }
}

void DecisionMaking::RoundAboutBeforeState() {
    std::cout << "Rotary Before Status" << std::endl;
    if(!isSignWorthy("rotary", this->rotaryworthy_timecheck, this->rotaryworthy_time)) {
        this->current_state = VehicleState::Driving;
    }
    this->throttle = this->normal_throttle;
}

void DecisionMaking::RoundAboutNowState() {
    std::cout << "Rotary Now Status" << std::endl;
    if (!this->rotarynow_timecheck) {
        this->rotarynow_timecheck = true;
        this->rotarynow_stoptime= std::chrono::steady_clock::now();
        this->throttle = this->normal_throttle;
    } 
    else {
        auto now = std::chrono::steady_clock::now();
        auto time_gap = std::chrono::duration_cast<std::chrono::seconds>(now - this->rotarynow_stoptime).count();
        if (time_gap >= this->rotarysign_ignore) {
            this->current_state = VehicleState::Driving;
            this->rotarynow_timecheck = false; 
        }
    }
}

void DecisionMaking::RacingState() {
    std::cout << "Racing Mode" << std::endl;
    this->throttle = this->normal_throttle;
}

bool DecisionMaking::isSignWorthy(const std::string state, bool& sign_check, std::chrono::steady_clock::time_point& sign_miss_time) {
    std::string sign_info = (*signs)[0].id;

    if (sign_info == state) {
        sign_check = false;
        return true;
    } 
    else {
        if (!sign_check) {
            sign_check = true;
            sign_miss_time = std::chrono::steady_clock::now();
        } 
        else {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - sign_miss_time).count();
            if (elapsed > this->status_break_time) {
                sign_check = false;
                return false;
            }
        }
    }
    return true;
}

float DecisionMaking::getThrottle() {
    return this->throttle;
}

int DecisionMaking::getState() {
    switch (current_state) {
        case VehicleState::Driving: return 0; break;
        case VehicleState::CrosswalkBefore: return 1; break;
        case VehicleState::CrosswalkNow: return 2; break;
        case VehicleState::RoundAboutBefore: return 3; break;
        case VehicleState::RoundAboutNow: return 4; break;
        case VehicleState::TrafficLightRedBefore: return 5; break;
        case VehicleState::TrafficLightRedNow: return 6; break;
        case VehicleState::TrafficLightYellowBefore: return 7; break;
        case VehicleState::TrafficLightYellowNow: return 8; break;
        case VehicleState::TrafficLightGreenBefore: return 9; break;
        case VehicleState::TrafficLightGreenNow: return 10; break;
        default: return 0; break;
    }
}