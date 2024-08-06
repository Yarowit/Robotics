// File: MyRobot.cpp
// Date:
// Description: Main Controller file
// Author: Jarek and Bernardo
// Modifications:

#include "MyRobot.h"

// ############ Mapping #############
#include "Mapping.hpp"

using namespace webots;

// moved
unsigned short stopCount = 0;
// Initialize Values
MyRobot::MyRobot() : Robot()
{
  // init default values
  _time_step = 64;

  // get cameras and enable them
  _forward_camera = getCamera("camera_f");
  _forward_camera->enable(_time_step);
  _spherical_camera = getCamera("camera_s");
  _spherical_camera->enable(_time_step);

  // Motor initialization
  _left_wheel_motor = getMotor("left wheel motor");
  _right_wheel_motor = getMotor("right wheel motor");

  // Set position to infinity to allow velocity control
  _right_wheel_motor->setPosition(INFINITY); 
  _left_wheel_motor->setPosition(INFINITY);
  
  // Initialize gps
  _my_gps = getGPS("gps"); 
  _my_gps->enable(_time_step); 
  
  // Get robot's compass; initialize it 
  _my_compass = getCompass("compass");
  _my_compass->enable(_time_step);

  // Set initial velocity to 0
  _right_wheel_motor->setVelocity(0.0);
  _left_wheel_motor->setVelocity(0.0);

  _x = _y = _theta = 0.0; // robot pose variables 
  _sr = _sl = 0.0; // displacement right and left wheels
  // _theta = *tmp;

  _x_goal = 18; _y_goal =0; 

  //initialize distance sensors
  for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++){
    cout << "Initializing distance sensor: " << ds_name[ind] <<endl;
    _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);
    _distance_sensor[ind]->enable(_time_step);
  }

  // Motor Position Sensor initialization
  _left_wheel_sensor = getPositionSensor("left wheel sensor");
  _right_wheel_sensor = getPositionSensor("right wheel sensor");

  _left_wheel_sensor->enable(_time_step);
  _right_wheel_sensor->enable(_time_step); 

  
  // ############ Mapping #############

  init_map();
  myfile.open("map.txt");
  // DistanceSensor ss;
  // myfile<<ss.getLookupTableSize()<<endl;
  // float lookup[1024];
  //   // _distance_sensor[0]->setValue(0.5);
  // myfile<<wb_distance_sensor_get_lookup_table_size<<endl;
  //   // _distance_sensor[0]->getValue();
  // for(int i=0; i<1024;i++){
  // }
  // myfile<<(_distance_sensor[0])->getMinValue()<<endl;
  // myfile<<DistanceSensor.getLookupTable()[5]<<endl;
}

// Delete instances
MyRobot::~MyRobot()
{
    // disable camera devices
    _forward_camera->disable();
    _spherical_camera->disable();

    // Disable robot's sensors
    _my_compass->disable();
    _my_gps->disable();

    // disable devices --> distance sensor
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++){
      cout << "Disabling distance sensor: " << ds_name[ind] <<endl;
      _distance_sensor[ind]->disable();
    }
}

// Main Code Section
void MyRobot::run()
{
    // ######### temp ############
    const int mapMax = 100;
    int mapPrintCount = 1;
    const int mapMaxr = 100;
    int mapPrintCountr = 1;
    // const int dijMax = 700;
    // int dijkstraCount = dijMax;
  // get size of images for forward camera
  image_width_f = _forward_camera->getWidth();
  image_height_f = _forward_camera->getHeight();

  // get size of images for spherical camera
  image_width_s = _spherical_camera->getWidth();
  image_height_s = _spherical_camera->getHeight();

  //initialize map in init pos

  // Main loop:
  while (step(_time_step) != -1) 
  {
    // myfile<<"GGGGGGGGGGGGGGGGG\n";
    // myfile<<_x<<" "<<_y<<endl; 
    // myfile<<"GGGGGGGGGGGGGGGGG\n";
    if(!bug_done)
    {
        // cout<<"Goal reached? "<<this->goal_reached()<<endl;
      this->bug_2();
    }
    else if (!id_done)
    {
      this->id_people();
    }
    else if(!dijkstra_done) // DIJKSTRA
    {
      compute_odometry();
      _left_wheel_motor->setVelocity(0);
      _right_wheel_motor->setVelocity(0);
      cout << "Done" << endl;
      myfile<<"START PREPROCESSING"<<endl;
      preprocessMap();
      myfile<<"ENDED PREPROCESSING"<<endl;
      int x = _x*pixelsPerMeter + x_off;
      int y = _y*pixelsPerMeter + y_off;
      int X = x_off;
      int Y = y_off;
      myfile<<x<<" "<<y<<" "<<X<<" "<<Y<<endl;
      printMap(x,y,X,Y);
      points = dijkstra(x,y,X,Y);
      for(auto point : points){
        cout<<point[0]<<" "<<point[1]<<endl;
      }
      cout<<"SENS: "<<(*_distance_sensor)->getMaxValue()<<endl;
      dijkstra_done = true;
      stopCount = 0;
    }else{
        cout<<"dijsktra is actually done"<<endl;
        compute_odometry();
      if(navigate_points())
        break;
      if(mapPrintCountr-- < 0){
        printMapWithPos(_x, _y);
        mapPrintCount = mapMaxr;
      }
    }
    
    //############ Mapping #############
    updateMap(_x, _y, _theta, _distance_sensor[4]->getValue(), _distance_sensor[5]->getValue(), max(_distance_sensor[0]->getValue(),_distance_sensor[1]->getValue()));
    // updateMap(_x, _y, _theta, _distance_sensor[4]->getValue(), _distance_sensor[5]->getValue(), 0.5*(_distance_sensor[0]->getValue()+_distance_sensor[1]->getValue()));
    if(!dijkstra_done && mapPrintCount-- < 0){
      // printMap();
      int x = _x*pixelsPerMeter + x_off;
      int y = (-_y*pixelsPerMeter + y_off);
      printMap(x,y,0,0);
      mapPrintCount = mapMax;
    }
  }
}

// Determine initial position
void MyRobot::init_pos()
{
  // we only use the gps and compass to know the initial position of the robot 
  if (step(_time_step) != -1)  
  { 
    init_x =_my_gps->getValues()[2]; 
    init_y =_my_gps->getValues()[0]; 
    
    // initialize map
    initZeroMap(init_x+6,init_y);
    
    // line angle
    angle = atan2((_y_goal-_y),(_x_goal-_x));
    if(angle < 0) angle += 2*M_PI;
  } 
}

// Find People using forward camera
void MyRobot::id_people()
{
  cout<<"M: ";
  switch(spin_mode){
  case SPIN:
      cout<<"SPIN"; break;
  case LOCATE:
      cout<<"LOCATE"; break;
  case FOUND:
      cout<<"FOUND"; break;
  case CHECK:
      cout<<"CHECK"; break;
  }
  cout<<endl;
  compute_odometry();

  switch(spin_mode){
  case SPIN:
    this->spin_around();
    break;
  case LOCATE:
    this->go_to_green();
    break;
  case FOUND:
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    id_done = true;
    spin_done = false;
    break;
  case CHECK:
    people_num++;
    if (people_num > people_needed)
    {
      spin_mode = FOUND;
      id_done = true;
      break;
    }
    else
    {
      turn_init = false;
      spin_mode = SPIN;
    }
    break;
  }
}

// Spin around 360 deg, return false when done
bool MyRobot::spin_around()
{
  this->compute_odometry();
  this->print_odometry();
  if (turn_init == false)
  {
    angle += _theta - 1.5;
  }
  
  if((abs(_theta-angle) < ANGLE_THRESHOLD || 2*M_PI-abs(_theta-angle) < ANGLE_THRESHOLD) && people_num != people_needed)
  {
    cout<<2*M_PI-abs(_theta-angle)<<endl;
    cout<<abs(_theta-angle)<<endl;
    
    if (turn_direc == true)
    {
      adjust = 0.2;
      turn_direc = false;
    }
    else 
    {
      adjust = 1;
      turn_direc = true;
    }   
    spin_mode = LOCATE;
    return true;
  }
  else if ((people_num == people_needed) && (abs(_theta-angle) < ANGLE_THRESHOLD || 2*M_PI-abs(_theta-angle) < ANGLE_THRESHOLD))
  {
    spin_mode = CHECK;
    return true;
  }
  
  turn_init = true;
  if(abs(_theta-angle) < slowDown + 0.15 || (abs(_theta-angle) > 2*M_PI - slowDown - 0.15)){
        speed = 0.1;
    }else{
        speed = 2;
    }
  
  _left_wheel_motor->setVelocity(-speed);
  _right_wheel_motor->setVelocity(speed);
  return false;
}

// go towards green posts
void MyRobot::go_to_green()
{
  // Define local vars
  int sum = 0; // sum for forward cam
  unsigned char green = 0, red = 0, blue = 0; // rgb totals
  double left_percentage_green = 0.0; // left green
  double right_percentage_green = 0.0; // right green
  double mid_percentage_green = 0.0; // center green
  double center_square_percent = 0.0;

  float sspeed = 1.0;

  if (turn_direc == true)
  {
    _left_wheel_motor->setVelocity(-sspeed);
    _right_wheel_motor->setVelocity(sspeed);
  }
  else
  {
    _left_wheel_motor->setVelocity(sspeed);
    _right_wheel_motor->setVelocity(-sspeed);
  }
  
  // get current image from left side of forward camera and green percent
  const unsigned char *image_f = _forward_camera->getImage();
  for (int x = 0; x < image_width_f /3; x++) {
      for (int y = 0; y < image_height_f; y++) {
          green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
          red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
          blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

          if ((green > THRESHOLD) && (red < THRESHOLD) && (blue < THRESHOLD)) {
              sum = sum + 1;
          }
      }
  }
  left_percentage_green = (sum / (float) (image_width_f/3 * image_height_f)) * 100;
  
  sum = 0;
  // get current image from right side of forward camera and green percent
  for (int x = image_width_f *2/3; x < image_width_f; x++) {
      for (int y = 0; y < image_height_f; y++) {
          green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
          red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
          blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

          if ((green > THRESHOLD) && (red < THRESHOLD) && (blue < THRESHOLD)) {
              sum = sum + 1;
          }
      }
  }
  right_percentage_green = (sum / (float) (image_width_f/3 * image_height_f)) * 100;

  sum = 0;
   // get current image from middle of forward camera and green percent
  for (int x = image_width_f/3; x < image_width_f * 2/3; x++) {
      for (int y = 0; y < image_height_f; y++) {
          green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
          red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
          blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

          if ((green > THRESHOLD) && (red < THRESHOLD) && (blue < THRESHOLD)) {
              sum = sum + 1;
          }
      }
  }
  mid_percentage_green = (sum / (float) (image_width_f/3 * image_height_f)) * 100;

  sum = 0;
  // get current image from center center of forward camera and green percent
  for (int x = image_width_f *1/3; x < image_width_f * 2/3; x++) {
      for (int y = 1/4 * image_height_f; y < image_height_f * 3/4; y++) {
          green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
          red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
          blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

          if ((green > THRESHOLD) && (red < THRESHOLD) && (blue < THRESHOLD)) {
              sum = sum + 1;
          }
      }
  }
  center_square_percent = (sum / (float) (image_width_f/3 * image_height_f/2)) * 100;

  cout << "Percentage of green middle in forward camera image: " << mid_percentage_green << endl;
  cout << "Percentage of green right in forward camera image: " << right_percentage_green << endl;
  cout << "Percentage of green left in forward camera image: " << left_percentage_green << endl;
  cout << "Percentage of green center square in forward camera image: " << center_square_percent << endl;

  if(abs(left_percentage_green - right_percentage_green) > 20){
    float x = min(48.0,abs(left_percentage_green - right_percentage_green));
    sspeed = min(2.0,-50.0/(x-50)-0.7);
    sspeed = exp(abs(left_percentage_green - right_percentage_green)/48)-1;
    sspeed = min(1.5,abs(left_percentage_green - right_percentage_green)/10+0.1);
    cout<<x<<" "<<sspeed<<endl;
    if(left_percentage_green > right_percentage_green){
      // go left
      _left_wheel_motor->setVelocity(-sspeed);
      _right_wheel_motor->setVelocity(sspeed);
    }else{
      // go right
      _left_wheel_motor->setVelocity(sspeed);
      _right_wheel_motor->setVelocity(-sspeed);
    }
    cout<<"turn\n";
  // }else if(center_square_percent > 1 && center_square_percent < 30){
  }else if(center_square_percent > 1 && mid_percentage_green < people_thres){
    sspeed = min(5.0,5.0*exp((-center_square_percent)/100));
    _left_wheel_motor->setVelocity(sspeed);
    _right_wheel_motor->setVelocity(sspeed);
    cout<<"forward\n";
  }else sspeed = -1;

  // if ((center_square_percent > 1 && center_square_percent < 30)){
  //   sspeed = min(5.0,exp((100-mid_percentage_green)/50)-1.0);
  //   _left_wheel_motor->setVelocity(sspeed);
  //   _right_wheel_motor->setVelocity(sspeed);
  // }else if (left_percentage_green > 15 && right_percentage_green < 30){
  //   // go left
  //   sspeed = min(2.0,exp(abs(left_percentage_green - right_percentage_green)/100*abs(left_percentage_green - right_percentage_green)/100 / 5)-1.0);
  //   _left_wheel_motor->setVelocity(-sspeed);
  //   _right_wheel_motor->setVelocity(sspeed);
  // }else if (left_percentage_green < 30 && right_percentage_green > 15){
  //   // go right
  //   sspeed = min(2.0,exp(abs(left_percentage_green - right_percentage_green)/100*abs(left_percentage_green - right_percentage_green)/100 / 5)-1.0);
  //   _left_wheel_motor->setVelocity(sspeed);
  //   _right_wheel_motor->setVelocity(-sspeed);
  // }

  cout<<"Sspeed: "<<sspeed<<endl;

 /*
  if (mid_percentage_green < 30 && (center_square_percent > 1 && center_square_percent < 30))
  {
    // go straight
    _left_wheel_motor->setVelocity(5);
    _right_wheel_motor->setVelocity(5);
  }

  if (mid_percentage_green > 30)
  {
    // go straight
    _left_wheel_motor->setVelocity(2);
    _right_wheel_motor->setVelocity(2);
  }
  else if (left_percentage_green > 15 && right_percentage_green < 30)
  {
    // go left
    _left_wheel_motor->setVelocity(-2);
    _right_wheel_motor->setVelocity(2);
  }
  else if (left_percentage_green < 30 && right_percentage_green > 15)
  {
    // go right
    _left_wheel_motor->setVelocity(2);
    _right_wheel_motor->setVelocity(-2);
  }
  */

  // Define local vars
  sum = 0; // sum for forward cam
  double percentage_green = 0.0; // total green

  // get current image from forward camera and green percent
  image_f = _forward_camera->getImage();
  for (int x = 0; x < image_width_f; x++) {
    for (int y = 0; y < image_height_f; y++) {
      green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
      red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
      blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

      if ((green > THRESHOLD) && (red < THRESHOLD) && (blue < THRESHOLD)) {
          sum = sum + 1;
      }
    }
  }
  percentage_green = (sum / (float) (image_width_f * image_height_f)) * 100;
  cout << "Percentage of green in forward camera image: " << percentage_green << endl;
  
  if (mid_percentage_green > people_thres)
  // if (percentage_green > people_thres)
  {
    cout << "Found Person # " << people_num << endl;
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    spin_mode = CHECK;
  }
}

// Find percentage of yellow present in spherical camera
void MyRobot::id_yell()
{
  // Declare local vars
  int sum_s = 0; // sum for spherical cam
  unsigned char green = 0, red = 0, blue = 0; // rgb totals
  double percentage_yellow = 0.0; // total yellow

  // get current image from spherical camera and yellow percent
  const unsigned char *image_s = _spherical_camera->getImage();
  for (int x = 0; x < image_width_s; x++) {
    for (int y = 0; y < image_height_s; y++) {
      green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
      red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
      blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

      if ((green > yell_THRESHOLD) && (red < yell_THRESHOLD/2) && (blue > yell_THRESHOLD)) {
          sum_s = sum_s + 1;
      }
    }
  }
  percentage_yellow = (sum_s / (float) (image_width_s * image_height_s)) * 100;    
  cout << "Percentage of yellow in spherical camera image: " << percentage_yellow << endl;  
}

// Convert encoder tics to meter
float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics/ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

const int front_wall_threshold = 100;
const int threshold = 20;
const int angleThreshold = 10;
// Determine if goal is reached
bool MyRobot::goal_reached()
{
  const double threshold = 0.5;
  return ((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal) < threshold);
}

// Determine if onTheLine
bool MyRobot::onTheLine(){
    const float angThreshold = 0.01;
    // const float angThreshold = 0.1;
    if((_y_goal-_y)*(_y_goal-_y)+(_x_goal-_x)*(_x_goal-_x) < 1){
      return (abs(angle - atan2((_y_goal-_y),(_x_goal-_x))) < 5*angThreshold);
    }
    return (abs(angle - atan2((_y_goal-_y),(_x_goal-_x))) < angThreshold);
}

// Compute Odometry for bug 2
void MyRobot::compute_odometry()
{
  double sr =  encoder_tics_to_meters(_right_wheel_sensor->getValue()) - _sr;
  double sl =  encoder_tics_to_meters(_left_wheel_sensor->getValue()) - _sl;
  
  _sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());
  _sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());
  _x += (sr+sl)/2 * cos(_theta + (sr-sl) / (2 * WHEELS_DISTANCE));
  _y += (sr+sl)/2 * sin(_theta + (sr-sl) / (2 * WHEELS_DISTANCE));
  _theta += (sr-sl) / WHEELS_DISTANCE;
  if(_theta < 0) _theta += 2*M_PI;
  if(_theta > 2*M_PI) _theta -= 2*M_PI;
} 

// Front wall detection
bool MyRobot::frontWall(){
    float diff = 0.25*(_distance_sensor[2]->getValue()+_distance_sensor[3]->getValue());
    if(0.5*(_distance_sensor[0]->getValue()+_distance_sensor[1]->getValue()+ diff) > front_wall_threshold)
        return true;
    else
    {
        return false;
}
}
bool MyRobot::frontInnerWall(){
    float diff = 0.5*(_distance_sensor[2]->getValue()+_distance_sensor[3]->getValue());
    if(0.5*(_distance_sensor[0]->getValue()+_distance_sensor[1]->getValue()+ diff) > front_wall_threshold)
        return true;
    else
    {
        return false;
}
}
bool MyRobot::frontCloseWall(){
    float diff = 0.25*(_distance_sensor[2]->getValue()+_distance_sensor[3]->getValue());
    if(0.5*(_distance_sensor[0]->getValue()+_distance_sensor[1]->getValue()+ diff) > front_wall_threshold/4)
        return true;
    else
    {
        return false;
}
}

bool MyRobot::sideWall(){
  if(leftTurningMode)
    return leftWall();
  else
    return rightWall();
}

// Left wall detection
bool MyRobot::leftWall(){
    if(_distance_sensor[4]->getValue() > threshold)
        return true;
    else
        return false;
}

// Right wall detection
bool MyRobot::rightWall(){
    if(_distance_sensor[5]->getValue() > threshold)
        return true;
    else
        return false;
}

// Print odometry
void MyRobot::print_odometry()
{
  cout << "x: " << _x << " y: " << _y << " theta: " << _theta <<" angle: "<<angle<< endl;
}

bool leftTheLine = false;
bool setup = true;
// Bug 2 Algorithm
bool MyRobot::bug_2()
{  
  noRightSide = false;
  noLeftSide = false;
  if(setup)
  {
    setup = false;
    this->init_pos();
    const double *tmp =  _my_compass->getValues();
    // _theta = atan2(tmp[0], tmp[2]) + M_PI/2;
    _theta = 2*M_PI -  (atan2(tmp[0], tmp[2])+ M_PI/2);
    // if(_theta > M_PI) _theta -= 2*M_PI;
  }
  // distance sensors initializations
  double front_ir = 0.0, innerLeft_ir = 0.0, innerRight_ir = 0.0, outerLeft_ir = 0.0,outerRight_ir = 0.0;
  front_ir = 0.5*(_distance_sensor[0]->getValue()+_distance_sensor[1]->getValue());
  innerLeft_ir = _distance_sensor[2]->getValue();
  innerRight_ir = _distance_sensor[3]->getValue();
  outerLeft_ir = _distance_sensor[4]->getValue();
  outerRight_ir = _distance_sensor[5]->getValue();
        
  this->compute_odometry(); 
  this->print_odometry();
  cout << "SENSORS--------------------------" << endl;
  cout << front_ir << endl;
  cout << innerLeft_ir << endl;
  cout << innerRight_ir << endl;
  cout << outerLeft_ir << endl;
  cout << outerRight_ir << endl;
  cout<< endl;
  cout << "---------------------------------" << endl;
           
  if(frontWall() && mode != CIRCLE){
      mode = STOP;
  }
  cout<<"M: ";
  switch(mode){
  case FORWARD:
      cout<<"FORWARD"; break;
  case TURN:
      cout<<"TURN"; break;
  case STOP:
      cout<<"STOP"; break;
  case CIRCLE:
      cout<<"CIRCLE"; break;
  }
  cout<<endl;
  cout<<"L: "<<_left_wheel_motor->getVelocity()<<endl;
  cout<<"R: "<<_right_wheel_motor->getVelocity()<<endl;
  cout<<"OnTheLine: "<<onTheLine()<<" "<<frontWall()<<" "<<leftTheLine<<endl;
  
  //MODIFICATION
      if(_x>18){
    //   if(this->goal_reached()){
        cout<<"GOAL REACHED"<<endl;
        _left_wheel_motor->setVelocity(0);
        _right_wheel_motor->setVelocity(0);
        bug_done = true;
        return false;
    }

  switch(mode){
    case CIRCLE:
      if(frontCloseWall()){
        speed = 2;
        if(!leftTurningMode){
          _left_wheel_motor->setVelocity(-speed);
          _right_wheel_motor->setVelocity(speed);
        }else{
          _left_wheel_motor->setVelocity(speed);
          _right_wheel_motor->setVelocity(-speed);
        }
      }else if(!onTheLine() || !leftTheLine){
        if(!onTheLine()) leftTheLine = true;
        cout<<"SSTESTS "<<sideWall()<<" "<<!frontInnerWall()<<endl;
        cout<<"SSTESTS2 "<<leftTurningMode<<endl;
        if( sideWall() && !frontInnerWall() ){
            speed = min(6.0,exp((1024-front_ir)/500.0)); // 3:30
            // cout<<"SPEED: "<<speed<<endl;
            // speed = 4;
            _left_wheel_motor->setVelocity(speed);
            _right_wheel_motor->setVelocity(speed);
        // cout<< "M: Forw"<<endl;
        }
        
        if( !sideWall() && !frontWall() ){
            speed = 3;
            // speed = 2;
            // _left_wheel_motor->setVelocity(speed+1.6);
            if(!leftTurningMode){
              _left_wheel_motor->setVelocity(speed*3.4/2);
              // _left_wheel_motor->setVelocity(speed*3.6/2);
              _right_wheel_motor->setVelocity(speed * 0.85);
              // _right_wheel_motor->setVelocity(speed);
            }else{
              _left_wheel_motor->setVelocity(speed * 0.85);
              _right_wheel_motor->setVelocity(speed*3.4/2);

            }

            // DONT UPDATE THE LEFT SIDE
            if(!leftTurningMode)
              noRightSide = true;
            else
              noLeftSide = true;
        // cout<< "M: L"<<endl;
        } 
      }
      else{
          if(leftTheLine){
              mode = STOP;
              leftTheLine = false;
          }
      }
    
        break;
  case STOP:
    stopCount++;
    if(stopCount == MAX_WAIT){
        if(frontWall()){
            mode = CIRCLE;
        }else if(!(abs(_theta-angle) < ANGLE_THRESHOLD || 2*M_PI-abs(_theta-angle) < ANGLE_THRESHOLD)){
            mode = TURN;
        }else{
            mode = FORWARD;
        }
        stopCount = 0;
    }
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    break;
    
    case TURN:
      if(navigate_angle_condition()){
      // if((abs(_theta-angle) < ANGLE_THRESHOLD || 2*M_PI-abs(_theta-angle) < ANGLE_THRESHOLD)){
          cout<<"STOPPED"<<endl;
          cout<<(abs(_theta-angle) < 0.1 )<<endl;
          cout<<(2*M_PI-abs(_theta-angle) < 0.1)<<endl;
          cout<<2*M_PI-abs(_theta-angle)<<endl;
          cout<<2*M_PI<<endl;
          cout<<abs(_theta-angle)<<endl;
          mode = STOP;
          break;
      }
      cout<<"Have to rotate"<<endl;
      // if have to flip
      int flip;
    //   if(abs(_theta-angle) < M_PI){ // no
    //       flip = 1;
    //     if(abs(_theta-angle) < slowDown){
    //         speed = 0.1;
    //     }else{
    //         speed = 2;
    //     }
    // }else{ // yes
    //     flip = -1;
    //     if(abs(_theta-angle) > 2*M_PI - slowDown){
    //         speed = 0.1;
    //     }else{
    //         speed = 2;
    //     }
    // }
    if(abs(_theta-angle) < M_PI){ // no
          flip = 1;
    }else{ // yes
        flip = -1;
    }
    speed = min(4.0,exp(1.0*(_theta-angle)*(_theta-angle)));
            
    
    cout<<"speed/theta/angle"<<speed<<" "<<_theta<<" "<<angle<<endl;
    // turning direction
    if(_theta < angle){  
        _left_wheel_motor->setVelocity(-speed*flip);
        _right_wheel_motor->setVelocity(speed*flip);
    }else{  
        _left_wheel_motor->setVelocity(speed*flip);
        _right_wheel_motor->setVelocity(-speed*flip);
    }
    break;
    
    case FORWARD:
    
    // to fix the angle
    if(!(abs(_theta-angle) < ANGLE_THRESHOLD ||2*M_PI-abs(_theta-angle) < ANGLE_THRESHOLD)){
          mode = STOP;
          break;
      }
      // check how close we are
      if((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal) > 0.2)
        speed = 8;
      else
          speed = 1;
    _left_wheel_motor->setVelocity(speed);
    _right_wheel_motor->setVelocity(speed);
    break;
  }
  return true;
}

unsigned int i=0;

enum MyRobot::MODE mode_r = MyRobot::MODE::STOP;
bool MyRobot::navigate_goal_reached()
{
  const double threshold = 0.01; // for 0.1 meter accuracy
  return ((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal) < threshold);
}
bool MyRobot::navigate_angle_condition()
{
  const double angle_threshold = 0.05;
  return (abs(_theta-angle) < angle_threshold ||2*M_PI-abs(_theta-angle) < angle_threshold);
}

// DIJKSTRA - return
bool MyRobot::navigate_points(){
    float speed;
    // cout<<i<<" i/s "<<points.size()<<endl;
    if(i >= points.size())
        return true;

    _x_goal = (float) (points[i][0]-x_off) / pixelsPerMeter;
    _y_goal = (float) (points[i][1]-y_off) / pixelsPerMeter;

    // cout<<_x_goal<<" "<<_y_goal<<" <- "<<_x<<" "<<_y<<endl;
    // myfile<<i<<" "<<_x_goal<<" "<<_y_goal<<" <- "<<_x<<" "<<_y<<endl;
    // myfile<<(_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal)<<" "<<abs(_theta-angle)<<" "<<2*M_PI-abs(_theta-angle) <<endl;

    angle = atan2((_y_goal-_y),(_x_goal-_x));
    if(angle < 0) angle += 2*M_PI;
    cout<<"MODE: ";
    switch(mode_r){
        case FORWARD:
            cout<<"FORWARD"; break;
        case TURN:
            cout<<"TURN"; break;
        case STOP:
            cout<<"STOP"; break;
        case CIRCLE:
            cout<<"CIRCLE"; break;
        }
    switch(mode_r){
          case STOP:
            stopCount++;
            if(stopCount == MAX_RETURN_WAIT){
                if(!navigate_angle_condition()){
                    mode_r = TURN;
                }else{
                    mode_r = FORWARD;
                }
                stopCount = 0;
            }
            _left_wheel_motor->setVelocity(0);
            _right_wheel_motor->setVelocity(0);
            break;
           
           case TURN:
           
             if(navigate_angle_condition()){
                //  cout<<"STOPPED"<<endl;
                //  cout<<(abs(_theta-angle) < 0.1 )<<endl;
                //  cout<<(2*M_PI-abs(_theta-angle) < 0.1)<<endl;
                //  cout<<2*M_PI-abs(_theta-angle)<<endl;
                //  cout<<2*M_PI<<endl;
                //  cout<<abs(_theta-angle)<<endl;
                 mode_r = STOP;
                 break;
             }
             cout<<"Rotating: goal_angle/theta "<<angle<<" "<<_theta<<endl;
              // if have to flip
             int flip;
             if(abs(_theta-angle) < M_PI){ // no
                 flip = 1;
                // if(abs(_theta-angle) < slowDown){
                //     speed = abs(_theta-angle);
                //     // speed = 0.3;
                // }else{
                //     speed = 5;
                // }
            }else{ // yes
                flip = -1;
                // if(abs(_theta-angle) > 2*M_PI - slowDown){
                //     speed = abs(_theta-angle);
                //     // speed = 0.3;
                // }else{
                //     speed = 5;
                // }
            }
                // speed = min(10,exp(abs(_theta-angle)* 3); TIME: 5:22
                speed = min(8.0,exp(1.2*(_theta-angle)*(_theta-angle)));
            
            cout<<"speed/theta/angle"<<speed<<" "<<_theta<<" "<<angle<<endl;
            // turning direction
            if(_theta < angle){  
                _left_wheel_motor->setVelocity(-speed*flip);
                _right_wheel_motor->setVelocity(speed*flip);
            }else{  
                _left_wheel_motor->setVelocity(speed*flip);
                _right_wheel_motor->setVelocity(-speed*flip);
            }
            break;
           
           case FORWARD:
             if(this->navigate_goal_reached()){
                cout<<"GOAL REACHED"<<endl;
                i+=1;
                if(i == points.size()){
                    cout<<"ROUTE FINISHED"<<endl;
                    _left_wheel_motor->setVelocity(0);
                    _right_wheel_motor->setVelocity(0);
                    return true;
                }
                // _x_goal = x_goals[goalsIndex];
                // _y_goal = y_goals[goalsIndex]; 
                // angle = atan((_x-_x_goal)/(_y-_y_goal)) - M_PI/2;
                 // angle = atan((_y-_y_goal)/(_x-_x_goal));
                // cout<<"X:  "<<_x<<endl;
                // cout<<"Y:  "<<_y<<endl;
                // cout<<"XG: "<<_x_goal<<endl;
                // cout<<"YG: "<<_y_goal<<endl;
                // cout<<"A:  "<<_theta<<endl;
                // cout<<"AG: "<<angle<<endl;
                mode_r = STOP;
                break;
            }
            // to fix the angle
            if(!navigate_angle_condition()){
                 mode_r = STOP;
                 break;
             }
             // check how close we are
             speed = min(10.0,exp(4.0*(sqrt((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal))))); // 3:30
            //  speed = min(10.0,exp(12.0*(((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal))))); // 3:30
            //  if((_x-_x_goal)*(_x-_x_goal)+(_y-_y_goal)*(_y-_y_goal) > 0.2) // 3:52
            //     speed = 10;
            //  else
            //      speed = 3;
            _left_wheel_motor->setVelocity(speed);
            _right_wheel_motor->setVelocity(speed);
            break;
        default:
            break;
        }
    return false;
}