//
// Created by soroush on 3/9/19.
//

#ifndef CARWHEELVELOCITIES_WHEEL_H
#define CARWHEELVELOCITIES_WHEEL_H

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;


class Wheel {
public:
    std::vector<cv::Point> centerHistory;
    bool existingWheel;
    bool isTracked;
    int disappeared;
    float radius;
    int existingWheelID;
    float speed;
    Wheel(cv::Point p, float rad, int id);
    void speedCalculation();
    virtual ~Wheel() = default;
};


#endif //CARWHEELVELOCITIES_WHEEL_H
