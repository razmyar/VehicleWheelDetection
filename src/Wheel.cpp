//
// Created by soroush on 3/9/19.
//

#include "Wheel.h"

Wheel::Wheel(cv::Point p, float rad, int id) {

    //Add current center to the history
    centerHistory.emplace_back(p);
    radius = rad;
    existingWheelID = id;
    isTracked = true;
    existingWheel = true;
    disappeared = 0;

}

void Wheel::speedCalculation()
{
    int historySize = (int) centerHistory.size();


    if (historySize == 1) {
        speed = 0;

    } else if (historySize >= 2) {
        float deltaX = (centerHistory[historySize - 1].x) - (centerHistory[historySize - 2].x);
        speed = deltaX / 2;
    }
}
