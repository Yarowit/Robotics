// File: MyRobot.h
// Date:
// Description: Main Class file
// Author: Jarek and Bernardo
// Modifications:
#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_


#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>

#define MAX_RETURN_WAIT 2
#include <vector>
#include <math.h>
#include <stdlib.h>
#define THRESHOLD 80
#define yell_THRESHOLD 90
#define people_thres 70
// #define people_thres 44.5
#define ANGLE_THRESHOLD 0.1745
#define MAX_WAIT 10
#define people_needed 3 // people needed + 1

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 6
#define MAX_SPEED 10

#define WHEELS_DISTANCE 0.3606   //[=] meters
#define WHEEL_RADIUS    0.0825 //[=] meters

#define ENCODER_TICS_PER_RADIAN 1

class MyRobot : public Robot {
    public:
       
        //Define Modes
        enum MODE {
            FORWARD,
            TURN,
            STOP,
            CIRCLE
        };

        enum find_mode {
            SPIN,
            LOCATE,
            FOUND,
            CHECK
        };

        DistanceSensor*_distance_sensor[NUM_DISTANCE_SENSOR];
        const char *ds_name[NUM_DISTANCE_SENSOR] = {"ds0", "ds15","ds1",
        "ds14","ds2","ds13"};

        MyRobot();

        ~MyRobot();
        
        // Define functions
        void run(); // Run Function
        void id_people(); // Find People
        void id_yell(); // Find Yellow Line
        void init_pos(); // Get Initial Position
        bool bug_2(); // Bug 2 algorithm
        float encoder_tics_to_meters(float); // Convert encoder tics to meters
        bool goal_reached(); // Find if goal reached in bug 2
        bool onTheLine(); // Determine if on the line for bug 2
        void compute_odometry(); // Compute Odometry for bug 2
        bool frontWall(); // Front wall detection
        bool leftWall(); // Left wall detection
        bool rightWall(); // Right wall detection
        void print_odometry(); // Print Odometry
        bool spin_around(); // spin around 360 deg
        void go_to_green(); // go towards green posts

        // DIJKSTRA
        bool sideWall();

        bool frontCloseWall();
        bool frontInnerWall();
        bool navigate_points();
        bool navigate_goal_reached();
        bool navigate_angle_condition();

    // Initialize Variables
        // Define image widths and heights
        int image_width_f;
        int image_height_f;
        int image_width_s;
        int image_height_s;

        int people_num = 1; // num of people found
        double init_x; // initial x position
        double init_y; // initial y position
        double ang; // current angle position
        float _x=0, _y=0, _x_goal=0, _y_goal=0;   // [=] meters
        float _theta, _theta_goal;   // [=] rad
        float angle;
        float _sr=0, _sl=0;  // [=] meters
        enum MODE mode = STOP; // Initialize Enum
        enum find_mode spin_mode = LOCATE; // initialize spin mode
        bool turn_init = false; // turn initialization
        bool bug_done = false; // bug_2 done
        bool spin_done = false; // done with spin
        bool id_done = false; // done with finding people
        float speed; // speed of robot
        const double slowDown = 0.2; // slow down factor
        bool turn_direc = true; // determine what way to run after finding person, true is ccw (left), false is cw (right)
        float adjust = 1; // adjust threshold
        
        // DIJKSTRA
        bool dijkstra_done = false;
        vector<vector<int>> points;
    private:
        int _time_step; // control time step
        double _left_speed, _right_speed; // velocities

    // Instances of Sensors & Motors
        // Camera sensors
        Camera *_forward_camera;
        Camera *_spherical_camera;

        // Motors
        Motor* _left_wheel_motor;
        Motor* _right_wheel_motor;
        
        // Motor Position Sensor
        PositionSensor* _left_wheel_sensor;
        PositionSensor* _right_wheel_sensor;
        
        // Compass sensor
        Compass * _my_compass;
 
        // Get robot's GPS; initialize it  
        GPS * _my_gps;
};
#endif

