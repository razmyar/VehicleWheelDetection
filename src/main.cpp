#include <opencv2/opencv.hpp>
#include "Wheel.h"

using namespace std;

// global constants
char const *FILE_NAME = "../video/cars_passing_input.mp4";
char const *FILE_NAME_OUTPUT = "../video_output/results.avi";
char const *WINDOW_CURR = "Current Frame";

// Function prototype
cv::Mat resizeFrame(cv::Mat &src, float ratio);
vector<cv::Vec3f> wheelDetection(cv::Mat &src);
Wheel makeOneNewWheel(cv::Vec3f &circle, int &ID);
vector<Wheel> makeNewWheels(vector<cv::Vec3f> circles);
void printWheels(vector<Wheel> &Wheels, string s);
void matchDetectedWheelsWithExistingWheels(vector<Wheel> &detectedWhees, vector<Wheel> &existingWheels, int &id);
double distancePoints(cv::Point point1, cv::Point point2);
void updateExistingWheels(Wheel &detectedWheel, vector<Wheel> &existingWheels, int wheelIndex);
void drawWheels(cv::Mat &src, vector<Wheel> &wheels);

int main() {
    float resizeRatio = 1;
    float outputWidth = 600;
    int frameCount = 2;
    char EscKey = 0;
    int ID = 1;

    // create GUI windows
    cv::namedWindow(WINDOW_CURR, 0);
    cv::moveWindow(WINDOW_CURR, 550, 70);
    cv::VideoCapture cap;
    cv::VideoWriter writer;

    vector<cv::Vec3f> curFrameCircles;
    vector<Wheel> wheels;
    vector<Wheel> tempWheels;

    cap.open(FILE_NAME);
    float fps = (int) cap.get(CV_CAP_PROP_FPS);
    int fcc = CV_FOURCC('X', 'V', 'I', 'D');

    writer.open(FILE_NAME_OUTPUT, fcc, fps, cv::Size(600, 337));
    cv::Mat curFrameOrg, curFrame;

    while (cap.isOpened() && EscKey != 27) {
        cap.read(curFrameOrg);
        if (curFrameOrg.empty())
            break;
        resizeRatio = outputWidth / curFrameOrg.cols;
        curFrame = resizeFrame(curFrameOrg, resizeRatio);

        curFrameCircles = wheelDetection(curFrame);
        tempWheels = makeNewWheels(curFrameCircles);

        if (!curFrameCircles.empty() && curFrameCircles.front()[0] == 0)
            continue;
        if (!curFrameCircles.empty() && wheels.empty()) {
            if (curFrameCircles.front()[0] == 0)
                continue;
            cv::Vec3f c = curFrameCircles.front();
            Wheel w = makeOneNewWheel(c, ID);
            wheels.emplace_back(w);
            cout << "[INFO] Wheel Detected.  ID: " << w.existingWheelID << " + " << endl;
            ID++;
        } else {
            matchDetectedWheelsWithExistingWheels(tempWheels, wheels, ID);
        }

        for (auto &&w :wheels)
            w.speedCalculation();

        drawWheels(curFrame, wheels);

        cv::imshow(WINDOW_CURR, curFrame);
        writer.write(curFrame);
        tempWheels.clear();

        frameCount++;

        EscKey = (char) cv::waitKey(1000 / fps);
    }
    cap.release();
    writer.release();
    cv::destroyAllWindows();
    return 0;
}

cv::Mat resizeFrame(cv::Mat &src, float ratio) {
    cv::Mat resized;
    cv::resize(src, resized,
               cv::Size(int(src.cols * ratio),
                        int(src.rows * ratio)),
               0, 0, CV_INTER_LINEAR);
    return resized.clone();
}

vector<cv::Vec3f> wheelDetection(cv::Mat &src) {
    cv::Mat roi, mask, gray;
    mask = cv::Mat::zeros(src.rows, src.cols, CV_8U);

    cv::rectangle(mask, cv::Point(0, 135), cv::Point(src.cols, src.rows - 140),
                  cv::Scalar(255, 255, 255), -1);

    cv::bitwise_and(src, src, roi, mask);
    cv::dilate(roi, roi, cv::Mat());

    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0);

    cv::threshold(gray, gray, 80, 255, 2);
    vector<cv::Vec3f> circles;

    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
                     3.1,  // dp
                     200,  // minDist
                     200,  // param1
                     65,   // param2
                     5,    // minRadius
                     27    // maxRadius
    );
    return circles;

}

Wheel makeOneNewWheel(cv::Vec3f &circle, int &ID) {
    return Wheel(cv::Point(circle[0], circle[1]), circle[2], ID);
}

vector<Wheel> makeNewWheels(vector<cv::Vec3f> circles) {
    vector<Wheel> wheels;

    for (auto &&circle : circles) {
        Wheel w = Wheel(cv::Point(circle[0], circle[1]), circle[2], -1);
        w.isTracked = false;
        wheels.emplace_back(w);
    }
    return wheels;
}

void printWheels(vector<Wheel> &Wheels, string s) {
    if (Wheels.empty())
        return;
    cout << "\nWheel Name: " << s << endl;
    for (auto &&w:Wheels) {

        cout << "Wheel ID: " << w.existingWheelID
             << "      Position: " << w.centerHistory.back()
             << "      Radius: " << w.radius
             << "      Speed: " << w.speed
             << "      isTracked? " << w.isTracked
             << "      Number disappeared frames: " << w.disappeared << endl;
    }
}

void matchDetectedWheelsWithExistingWheels(vector<Wheel> &detectedWhees, vector<Wheel> &existingWheels, int &id) {

    for (auto &&existingWheel: existingWheels) {
        existingWheel.existingWheel = false;
        existingWheel.speedCalculation();

    }

    for (auto &&w: detectedWhees) {
        w.isTracked = false;
    }

    for (auto &&detectedWheel:detectedWhees) {
        if (detectedWheel.isTracked)
            continue;

        int indexOfLeastDistanse = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (int i = 0; i < existingWheels.size(); i++) {

            if (existingWheels[i].isTracked) {

                double dist = distancePoints(detectedWheel.centerHistory.back(),
                                             existingWheels[i].centerHistory.back());
                if (dist < minDistance) {
                    minDistance = dist;
                    indexOfLeastDistanse = i;
                }
            }
        }

        if (minDistance < 150) {
            detectedWheel.isTracked = true;
            detectedWheel.existingWheel = true;
            updateExistingWheels(detectedWheel, existingWheels, indexOfLeastDistanse);

        } else {
            detectedWheel.existingWheel = true;
            existingWheels.emplace_back(detectedWheel);
            existingWheels.back().existingWheelID = id;
            existingWheels.back().isTracked = true;
            id++;
            cout << "[INFO] Wheel Detected.  ID: " << existingWheels.back().existingWheelID << " + " << endl;
        }

    }

    bool deleteOperation = false;
    int index = 0;
    vector<int> deleteIndex;

    for (auto &&existingWheel:existingWheels) {
        if (!existingWheel.existingWheel || existingWheel.centerHistory.back().x > 450)
            existingWheel.disappeared++;
        if (existingWheel.disappeared >= 4) {
            deleteOperation = true;
            deleteIndex.emplace_back(index);
        }

        index++;
    }

    if (deleteOperation) {
        for (auto &&id : deleteIndex) {
            cout << "[INFO] Wheel Removed.   ID: " << existingWheels.at(id).existingWheelID << " - " << endl;
            existingWheels.erase(existingWheels.begin() + id);
        }
    }
    deleteOperation = false;
}

double distancePoints(cv::Point point1, cv::Point point2) {
    int intX = abs(point1.x - point2.x);
    int intY = abs(point1.y - point2.y);
    return (sqrt(pow(intX, 2) + pow(intY, 2)));
}

void updateExistingWheels(Wheel &detectedWheel, vector<Wheel> &existingWheels, int wheelIndex) {
    existingWheels[wheelIndex].centerHistory.emplace_back(detectedWheel.centerHistory.back());
    existingWheels[wheelIndex].isTracked = true;
    existingWheels[wheelIndex].existingWheel = true;

}

void drawWheels(cv::Mat &src, vector<Wheel> &wheels) {
    for (auto &&wheel: wheels) {
        if (wheel.centerHistory.back().x > 550)
            continue;
        cv::circle(src, wheel.centerHistory.back(), wheel.radius, cv::Scalar(255, 255, 255), 2);
        cv::putText(src,
                    "ID: " + to_string(wheel.existingWheelID),
                    cv::Point(wheel.centerHistory.back().x, 220),
                    cv::FONT_HERSHEY_PLAIN,
                    1,
                    cvScalar(255, 255, 255),
                    1);
        cv::putText(src, to_string(int(wheel.speed)) + " pxl/frame", cv::Point(wheel.centerHistory.back().x, 240),
                    cv::FONT_HERSHEY_PLAIN, 0.9, cvScalar(255, 255, 255), 1);
    }
}