//
//  ball.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef ball_hpp
#define ball_hpp
#include <Eigen/Dense>
#include <vector>
#include <GLUT/GLUT.h>
#include <stdio.h>
#include <string>
#include "readShape.hpp"
#include <sstream>
#include <utility>
#include <iomanip>
#include <fstream>
#include <stdlib.h>

 

class Ball{
public:
    GLfloat momentX = 0.0, momentY = 0.0, momentZ = 0.0;
       GLfloat Vx = 0.0, Vy = 0.0, Vz = 0.0;
       GLfloat ax = 0.0, ay = 0.0, az = 0.0;
       //GLfloat g = 1.0;
    GLfloat Vz_copy = 0.0;
       int index;
    double t = 0.0;
    
    GLfloat vmWeight = 0.4;
    GLfloat fWeight = 0.35;
    GLfloat lWeight = 0.25;
       
       //starting position
       GLfloat x = 0.0,y = 0.0 ,z = -5.0;
       
       Eigen::Matrix4f m;
    
       std::vector<std::vector<double> > vertices;
       std::vector<std::vector<int> > vertptrs;

    Ball(GLfloat x, GLfloat y, GLfloat z, int index);
    void getPosition(GLfloat time);
    void getVelocity(GLfloat time);
    void initializeShape(std::string type);
    void display(GLfloat time);
    void collision(Ball cball, GLfloat& ax, GLfloat& ay, GLfloat& az);
    void setVelocityMatching(Ball ball, GLfloat& sumX, GLfloat& sumY, GLfloat& sumZ, int& count);
    void getVelocityMatching(GLfloat& sumX, GLfloat& sumY, GLfloat& sumZ, int& count, GLfloat& ax, GLfloat& ay, GLfloat& az);
    void transform(double t);
    void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
    void flockCentering(GLfloat& ax, GLfloat& ay, GLfloat& az);
    Eigen::Vector3f getOrtho(Eigen::Vector3f& l);
    void getAcceleration();
    void setMoment(Ball ball);
    void resetState();
    void followLeader(GLfloat& ax, GLfloat& ay, GLfloat& az);
    void getCoefficents(GLfloat dist, GLfloat &a, GLfloat &c, Ball &ball2, Eigen::Vector3f proj1, Eigen::Vector3f proj2);
};


extern std::vector<Ball> ballVec;


#endif /* ball_hpp */
