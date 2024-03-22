#include "decision_making/decision_making.hpp"

DecisionMaking::DecisionMaking(VehicleState current_state, float normal_throttle,
                    std::vector<StopLine> *stopline, std::vector<Sign> *signs, std::vector<Light> *lights, std::vector<Object> *objects, Pose *pose) {
    // Use Pointer
    this->signs = signs;
    this->lights = lights;
    this->objects = objects;
    this->pose = pose;
    this->stopline = stopline;

    // Initialize Parameters
    this->throttle = 0.0;
    this->current_state = current_state;
    this->normal_throttle = normal_throttle;

    // Timer
    this->crosswalknow_timecheck = false;
    this->crosswalknow_stoptime = std::chrono::steady_clock::time_point::min();

    // Initialization
    this->crosswalksign_ignore = 3.0;
    this->stopline_mindistance = 10.0;
}

DecisionMaking::DecisionMaking() {
}

DecisionMaking::~DecisionMaking() {
}

void DecisionMaking::decide() {
    if(signs->empty()) {
        std::cout << "ROS2 Sign Error" << std::endl;
        return;
    }
    std::string sign_info = (*signs)[0].id;

    // Update State
    switch (current_state) {
        case VehicleState::Driving:
            if (sign_info=="None") { current_state = VehicleState::Driving; }
            else if (sign_info=="crosswalk") { current_state = VehicleState::CrosswalkBefore; }
            else if (sign_info=="priority") { current_state = VehicleState::Priority;}
            else { current_state = VehicleState::Driving; }
            break;
        case VehicleState::CrosswalkBefore:
            if((*stopline)[0].distance > 0 && (*stopline)[0].distance < this->stopline_mindistance) {
                this->current_state = VehicleState::CrosswalkNow;
            }
            break;
        case VehicleState::CrosswalkNow:
            break;
        case VehicleState::CrosswalkAfter:
            break;
        case VehicleState::Priority:
            if(sign_info=="None") { current_state = VehicleState::Driving; }
            break;
        default:
            current_state = VehicleState::Driving;
            break;

    }

    // 현재 상태에 따른 행동 실행
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
        case VehicleState::CrosswalkAfter:
            CrosswalkAfterState();
            break;
        case VehicleState::Priority:
            PriorityState();
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
            this->current_state = VehicleState::CrosswalkAfter;
            this->crosswalknow_timecheck = false; 
        }
    }
}

void DecisionMaking::CrosswalkAfterState() {
    std::cout << "CrossWalk After Status" << std::endl;
    this->current_state = VehicleState::Driving;
    this->throttle = this->normal_throttle;
}

void DecisionMaking::PriorityState() {
    std::cout << "Priority Status" << std::endl;
    this->throttle = this->normal_throttle;
}

float DecisionMaking::getThrottle() {
    return this->throttle;
}