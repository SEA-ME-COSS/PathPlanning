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

    // Initialization
    this->crosswalksign_ignore = 3;
    this->sign_mindistance = 3.0;
    this->status_break_time = 10;
}

DecisionMaking::DecisionMaking() {
}

DecisionMaking::~DecisionMaking() {
}

void DecisionMaking::decide() {
    // if(!lights->empty()) {
    //     if(RedLightState()) {return;}
    // }
    
    if(!signs->empty()) {
        StatusDecision();
    }
    else {
        RacingState();
    }
}

void DecisionMaking::StatusDecision() {
    std::string sign_info = (*signs)[0].id;
    float sign_distance = (*signs)[0].distance;

    // Update State
    switch (current_state) {
        case VehicleState::Driving:
            if (sign_info=="None") { current_state = VehicleState::Driving; }
            else if (sign_info=="crosswalk") { current_state = VehicleState::CrosswalkBefore; }
            else { current_state = VehicleState::Driving; }
            break;
        case VehicleState::CrosswalkBefore:
            if((sign_info == "crosswalk" && sign_distance < this->sign_mindistance)) {
                this->current_state = VehicleState::CrosswalkNowwithHuman;
            }
            break;
        case VehicleState::CrosswalkNow:
            // Move slowly for "this->crosswalksign_ignore(default : 3.0)". After that change to Default
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
        default:
            DefaultState();
            break;
    }
}


void DecisionMaking::DefaultState() {
    std::cout << "Default Status" << std::endl;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::CrosswalkBeforeState() {
    std::cout << "CrossWalk Before Status" << std::endl;
    if(!isSignWorthy("crosswalk", this->crosswalkworthy_timecheck, this->crosswalkworthy_time)) {
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

float DecisionMaking::getThrottle() {
    return this->throttle;
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

void DecisionMaking::RacingState() {
    std::cout << "Racing Mode" << std::endl;
    this->throttle = this->normal_throttle;
}

int DecisionMaking::getState() {
    switch (current_state) {
    case VehicleState::Driving: return 0; break;
    case VehicleState::CrosswalkBefore: return 1; break;
    case VehicleState::CrosswalkNow: return 2; break;
    default: return 0; break;
    }
}