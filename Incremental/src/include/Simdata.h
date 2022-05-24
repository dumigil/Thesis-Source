#ifndef Simdata_h
#define Simdata_h
#include <assert.h>
#include <vector>
#include "Voxel.h"

struct simPoints{
    std::vector<Voxel> rooms, exits, fires;
    bool tilted = false;
    simPoints() {
        if (!tilted) {
            rooms = {
                    {43,  5,  106},
                    {121, 5,  27},
                    {45,  38, 194},
                    {124, 38, 103},
                    {56,  70, 126},
                    {50,  70, 158},
                    {115, 70, 160},
                    {119, 38, 78},
                    {124, 70, 103},
                    {43,  38, 106},
                    {47,  5,  234},
                    {43,  5,  106},
                    {119, 5,  198},
                    {111, 70, 127},
                    {131, 5,  235},
                    {43,  70, 59},
                    {131, 38, 235},
                    {50,  38, 158},
                    {50,  5,  158},
                    {47,  38, 234},
                    {119, 70, 198},
                    {131, 70, 235},
                    {107, 70, 17},
                    {124, 5,  103},
                    {47,  70, 234},
                    {111, 5,  127},
                    {119, 38, 198},
                    {121, 5,  27},
                    {43,  70, 106},
                    {115, 38, 160},
                    {119, 70, 78},
                    {111, 38, 127},
                    {45,  5,  194},
                    {56,  38, 126},
                    {45,  70, 194},
                    {56,  5,  126},
                    {115, 5,  160}
            };
            exits = {
                    {12,  5, 79},
                    {149, 5, 210}
            };

            fires = {
                    {88,  3,  64},
                    {80,  3,  230},
                    {102, 68, 191},
                    {6,   68, 125},
                    {90,  68, 32},
                    {73,  36, 103},
                    {68,  36, 211}
            };
        } else {
            rooms = {
                    {94,  5,  43},
                    {82,  5,  124},
                    {97,  5,  137},
                    {101, 5,  164},
                    {114, 5,  192},
                    {113, 5,  220},
                    {189, 5,  191},
                    {167, 5,  160},
                    {154, 5,  134},
                    {131, 5,  116},
                    {129, 5,  94},
                    {82,  32, 124},
                    {97,  32, 137},
                    {101, 32, 164},
                    {114, 32, 192},
                    {113, 32, 220},
                    {189, 32, 191},
                    {167, 32, 160},
                    {154, 32, 134},
                    {131, 32, 116},
                    {129, 32, 94},
                    {124, 32, 76},
                    {82,  60, 124},
                    {97,  60, 137},
                    {101, 60, 164},
                    {114, 60, 192},
                    {113, 60, 220},
                    {189, 60, 191},
                    {167, 60, 160},
                    {154, 60, 134},
                    {131, 60, 116},
                    {129, 60, 94},
                    {68,  60, 94},
                    {110, 60, 62}
            };
            exits = {{47,  5, 119},
                     {202, 5, 159}
            };
            fires = {
                    {90, 3, 78},
            };


        }
    };

};




#endif /* Simdata_h */
