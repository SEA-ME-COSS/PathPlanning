#pragma once

enum class VehicleState {
    Driving,
    StopBefore,
    StopNow,
    StopAfter,
    CrosswalkBefore,
    CrosswalkNow,
    CrosswalkNowwithHuman,
    Priority,
    HighwayEntranceBefore,
    HighwayEntranceNow,
    HighwayExitBefore,
    HighwayExitNow,
    RoundAbout,
};