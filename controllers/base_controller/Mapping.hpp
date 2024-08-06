#include <iostream>
#include <math.h>
#include "Dijkstra.hpp"

// #define mapSizeInMeters_X 22
// #define mapSizeInMeters_Y 10
// #define pixelsPerMeter 20

void printMap();

// bool Map[mapSizeInMeters_X * pixelsPerMeter][mapSizeInMeters_Y * pixelsPerMeter];

void init_map(){
  for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
    for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
        Map[i][j] = true;
        VisitedMap[i][j].timer = 0;
        VisitedMap[i][j].visits = 0;
    }
}

#define angleRange M_PI/10
#define maxSensorRange 0.25
// #define maxSensorRange 0.45
// #define maxSensorRange 0.55
#define maxSensorValue 1024

void initZeroMap(float x, float y){
  const float landing_spot_in_meters = 1.0;
  int c=0;
  for(int i = (x-landing_spot_in_meters)*pixelsPerMeter; i< (x+landing_spot_in_meters)*pixelsPerMeter; i++)
    for(int j = (y-landing_spot_in_meters)*pixelsPerMeter; j< (y+landing_spot_in_meters)*pixelsPerMeter; j++){
      Map[(int)i+ x_off][(int)j+y_off] = false;
      myfile<<(int)i+ x_off<<" "<<(int)j+y_off<<endl;
      c++;
    }


  // Map[5][5] = false;
  // myfile<<"\nAAAAAAAAAAAAAAAAAA\n"<<std::endl;
  // myfile<<x<<" "<<y<<" "<<landing_spot_in_meters<<std::endl;
  // myfile<<c<<" ff"<<std::endl;
  // printMap();
}
const float offset = 0.15;
float updateThresh(float sensorValue){
  // const float mod = 50.0;


  return offset+maxSensorRange * (maxSensorValue - sensorValue)/(maxSensorValue);
  // return maxSensorRange * (maxSensorValue - sensorValue - mod)/maxSensorValue;
}
// positive x -> toward the goal

  bool noRightSide = true;
  bool noLeftSide = true;
void updateMap(float x, float y, float angle, float leftSensor, float rightSensor, float frontSensor){//radians
  // const float mod = 100.0;
  // front camera
  for(float ang = angle - angleRange; ang < angle + angleRange; ang += 0.01)
    for(float r = offset/2; r < updateThresh(frontSensor); r += 0.01)
        Map[(int)((r*cos(ang)+x)*pixelsPerMeter+x_off)][(int)((r*sin(ang)+y)*pixelsPerMeter+y_off)] = false;

  // left camera
  if(!noLeftSide)
  for(float ang = angle - angleRange+M_PI/2; ang < angle + angleRange + M_PI/2; ang += 0.01)
    for(float r = offset; r < updateThresh(leftSensor); r += 0.01)
        Map[(int)((r*cos(ang)+x)*pixelsPerMeter+x_off)][(int)((r*sin(ang)+y)*pixelsPerMeter+y_off)] = false;

  // right camera
  if(!noRightSide)
  for(float ang = angle - angleRange-M_PI/2; ang < angle + angleRange - M_PI/2; ang += 0.01)
    for(float r = offset; r < updateThresh(rightSensor); r += 0.01)
        Map[(int)((r*cos(ang)+x)*pixelsPerMeter+x_off)][(int)((r*sin(ang)+y)*pixelsPerMeter+y_off)] = false;
    
  // also current robot position is explorable
  // for(int i = x*pixelsPerMeter-1; i<= x*pixelsPerMeter+1; i++)
  //   for(int j = y*pixelsPerMeter-1; j<= y*pixelsPerMeter+1; j++)
  //     Map[(int)i+ x_off][(int)j+y_off] = false;

  // int litsize = hsize * 0.5/2;
  // for(int i = x*pixelsPerMeter-litsize; i< x*pixelsPerMeter+litsize; i++)
  //   for(int j = y*pixelsPerMeter-litsize; j< y*pixelsPerMeter+litsize; j++)
  //     Map[(int)i+ x_off][(int)j+y_off] = false;

  unsigned int alreadyVisited = 0;
  for(int i = x*pixelsPerMeter-hsize; i< x*pixelsPerMeter+hsize; i++){
    for(int j = y*pixelsPerMeter-hsize; j< y*pixelsPerMeter+hsize; j++){
      if(VisitedMap[i][j].timer == 0){
        VisitedMap[i][j].visits++;
        VisitedMap[i][j].timer = VisitedTimerCooldown;
        if(VisitedMap[i][j].visits > 1)
          alreadyVisited++;
      }
        
    }
  }
  for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
    for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
        if(VisitedMap[i][j].timer > 0)
            VisitedMap[i][j].timer--;
    }
  cout<<"------------------------------------\n";
  cout<<"ALREADY VISITED: "<<alreadyVisited<<endl;
  cout<<"------------------------------------\n";
  if(alreadyVisited > 0) leftTurningMode = true;
  
}

void printMap(){
    for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
        for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
            if(Map[i][mapSizeInMeters_Y * pixelsPerMeter-1-j]) myfile<<"##";
            else myfile<<"  ";
            
        myfile<<std::endl;
    }
    myfile<<std::endl;
  // myfile.close();
}

void printMap(int a, int b, int c, int d){
    for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
        for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
            if(i == a && j == b) myfile<<"XX";
            else if(i == c && j == d) myfile<<"SS";
            else if(Map[i][mapSizeInMeters_Y * pixelsPerMeter-1-j]) myfile<<"##";
            else myfile<<"  ";
            
        myfile<<std::endl;
    }
    myfile<<std::endl;
  // myfile.close();
}

void printMapWithPos(float _x, float _y){
  int x = _x*pixelsPerMeter + x_off;
  int y = _y*pixelsPerMeter + y_off;
  myfile<<"\n\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n";
  for(int j=0;j<mapSizeInMeters_Y * pixelsPerMeter;j++){
      for(int i=0;i<mapSizeInMeters_X * pixelsPerMeter;i++)
          if(i ==  x && j ==  y) myfile<<"XX";
          else switch(Dij_Map[i][j]){
            // else switch(Dij_Map[i][mapSizeInMeters_Y * pixelsPerMeter-1-j]){
                case 0:
                    myfile<<"  ";
                    break;
                case 1:
                    myfile<<"##";
                    break;
                case 2:
                    myfile<<"++";
                    break;
                case 3:
                    myfile<<"--";
                    break;
                case 4:
                    myfile<<"><";
                    break;
                default:
                    break;
            }
      myfile<<std::endl;
  }
  myfile<<std::endl;
  myfile<<"\n\n$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n";
}