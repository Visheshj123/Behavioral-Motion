//
//  ball.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "ball.hpp"
#include <GLUT/GLUT.h>
#include <iostream>
#include <math.h>
#include <Eigen/Geometry>
//#include <fstream>


    
    
    Ball::Ball(GLfloat x, GLfloat y, GLfloat z, int index){
        this->index = index;
        initializeShape("ball.d");
        
        
        
        this->x = x;
        this->y = y;
        this->z = z;
        
        
        //sets intital velocities
        Vx = 0.01 * ax;
        Vy = 0.01 * ay;
        Vz = 0.01 * az;
        
    }
    
    //takes in delta t which will always be 0.01
    void Ball::getPosition(GLfloat time){
        if(time < 0.006 && index == 1){
            std::cout << "ball" << index << " x + Vx*time = " << x << " + " << Vx << " * " << time << " " << x + time * Vx << std::endl;
        }
        x = x + time * Vx;
        y = y + time * Vy;
        // std::cout << "Ball is at " << y << " with Vy at " << Vy << std::endl;
        //std::cout << "y is now " << y << std::endl;
        z = z + time * Vz;
        return;
    }
    
    //takes in delta t which will always be 0.01
    void Ball::getVelocity(GLfloat time){
          //std::cout << " with Vy at " << Vy << std::endl;
        Vy = Vy - time * ay;
        if(Vy > 1){
            Vy = 1;
        }else if(Vy < -1){
            Vy = -1;
        }
           
        Vx = Vx + time * ax;
        Vz = Vz + time * az;
         if(Vz > 1){
                   Vz = 1;
               }else if(Vz < -1){
                   Vz = -1;
               }
        std::cout << "Acceleration is " << ballVec[2].Vz << std::endl;
        std::cout << "Vz is " << ballVec[2].az << std::endl;
        std::cout << std::endl;
       
    }

void Ball::getCoefficents(GLfloat dist, GLfloat &a, GLfloat &c, Ball &ball2, Eigen::Vector3f proj1, Eigen::Vector3f proj2){
    

    //if they're not in same direction
    if(proj1(0) * proj2(0) < 0 ||proj1(1) * proj2(1) < 0 || proj1(2) * proj2(2) < 0){
        a = -2;
        c = -2;
        return;
    }
        
         Eigen::Vector3f l((x - ball2.x),(y - ball2.y), (z - ball2.z));
     
     //get next position of ball1
        GLfloat x1 =  x + 0.01*proj1(0);
        GLfloat y1 = y + 0.01*proj1(1);
        GLfloat z1 = z + 0.01*proj1(2);
        //Eigen Vector3f v1
        //dist between this point and one endpoint + dist between this point and the other endpoint
         GLfloat dist1 = sqrt( pow(x-x1, 2.0) + pow(y-y1, 2.0) + pow(z-z1,2.0));
         dist1 += sqrt(  pow((x1 - ball2.x), 2.0) + pow((y1 - ball2.y), 2.0) + pow((z1 - ball2.z), 2.0)  );

    //get next position of ball2
        GLfloat x2 =ball2.x + 0.01*proj2(0);
        GLfloat y2 = ball2.y - 0.01*proj2(1);
        GLfloat z2 = ball2.z + 0.01*proj2(2);
        GLfloat dist2 = sqrt( pow(x-x2, 2.0) + pow(y-y2, 2.0) + pow(z-z2,2.0));
        dist2 += sqrt(  pow((x2 - ball2.x), 2.0) + pow((y2 - ball2.y), 2.0) + pow((z2 - ball2.z), 2.0)  );

    //if next position is NOT between endpoints, additive
        if(dist1 - dist > 0.0000000001){
            a = 1.10;
            c = -1.90;
        }
        else if(dist2 - dist > 0.00000000001){
             a = -1.90;
            c = 1.10;
        }
    std::cout << "dist1 was " << dist1 << ", dist2 was: " << dist2 << " dist was " << dist << std::endl;
    return;
}

    void Ball::collision(Ball cball,  GLfloat& ax, GLfloat& ay, GLfloat& az){
        
        //with other object
        
            GLfloat dist = sqrt(  pow((x - cball.x), 2.0) + pow((y - cball.y), 2.0) + pow((z - cball.z), 2.0)  );
        if(dist - 0.2 < 0 && index < cball.index){
            
            std::cout << "collision between ball " << index << " and " << cball.index << std::endl;
      
            //create vector from centroids
                Eigen::Vector3f l((x - cball.x),(y - cball.y), (z - cball.z));

                //velocity vector from on ball
                Eigen::Vector3f ball1(Vx,Vy,Vz); //hypotenuos
                Eigen::Vector3f ball2(cball.Vx, cball.Vy, cball.Vz); //hypotenuos

                //create projection vector onto vector connecting centroids
                Eigen::Vector3f projL1;
                Eigen::Vector3f projL2;

                projL1 << (ball1.dot(l))/(l.dot(l)) * l; //leg
                projL2 << (ball2.dot(l))/(l.dot(l)) * l; //leg
                GLfloat a = 0.0, c = 0.0;
                getCoefficents(dist, a, c, cball, projL1, projL2);
                Eigen::Vector3f result1;
                result1 << a*projL1 + ball1;
                Eigen::Vector3f result2;
                result2 << c*projL2 + ball2;

               // resetState();

            std::cout << "before collision ball" << index << " velocity is: " << Vx << ", " << Vy << ", " << Vz << std::endl;
            std::cout << "before collision ball" << cball.index << " velocity is: " << cball.Vx << ", " << cball.Vy << ", " << cball.Vz << std::endl;
            Vx = result1(0);
            Vy = result1(1);
            Vz = result1(2);
      
            
                vmWeight = 0;
                fWeight = 0;
                lWeight = 0;

            

            cball.Vx = result2(0);
            cball.Vy = result2(1);
            cball.Vz = result2(2);
            
            std::cout << "moving ball" << index <<  " in this direction: " << Vx << ", " << Vy << ", " << Vz << std::endl;
            std::cout << "moving ball" << cball.index << " in this direction: " << cball.Vx << ", " << cball.Vy << ", " << cball.Vz << std::endl;
            std::cout << "coefficents were " << a << ", " << c << std::endl;
            std::cout << std::endl;
            
             
           
            
            getPosition(0.05);

            cball.vmWeight = 0;
            cball.fWeight = 0;
            cball.lWeight = 0;
            cball.getPosition(0.05);
            
             GLfloat newDist = sqrt(  pow((x - cball.x), 2.0) + pow((y - cball.y), 2.0) + pow((z - cball.z), 2.0)  );
            std::cout << newDist << std::endl;
            
            
        
               
        }
            

    }
    
    void Ball::setVelocityMatching(Ball ball, GLfloat& sumX, GLfloat& sumY, GLfloat& sumZ, int& count){
        if(ball.index != index){
            sumX += ball.Vx;
            sumY += ball.Vy;
            sumZ += ball.Vz;
            count++;
        }

    }

    void Ball::getVelocityMatching(GLfloat &sumX, GLfloat &sumY, GLfloat& sumZ, int &count, GLfloat& ax, GLfloat& ay, GLfloat& az){
        if(count == 0) return;

        double avgVx = sumX/count;
        double avgVy = sumY/count;
        double avgVz = sumZ/count;

        ax += vmWeight*((avgVx - Vx)/1);
        ay += -1*vmWeight*((avgVy - Vy)/1);
        az += vmWeight*((avgVz - Vz)/1);


    }

    void Ball::setMoment(Ball ball){
        momentX += ball.x;
        momentY += ball.y;
        momentZ += ball.z;
    }




    void Ball::flockCentering(GLfloat& ax, GLfloat& ay, GLfloat& az){
        
        double x_cm = (momentX + ballVec[0].x)/ballVec.size();
        double y_cm = (momentY + ballVec[0].y)/ballVec.size();
        double z_cm = (momentZ + ballVec[0].z)/ballVec.size();
    
        
        GLfloat Vfx = (x_cm - x)/1;
        GLfloat Vfy = (y_cm - y)/1;
        GLfloat Vfz = (z_cm - z)/1;
        
        ax += fWeight*((Vfx - Vx)/1);
        ay += -1*fWeight*((Vfy - Vy)/1);
        az += fWeight*((Vfz - Vz)/1);
        
    
    }

    void Ball::resetState(){
        
        
        momentX = 0;
        momentY = 0;
        momentZ = 0;
        
    }
    
//TODO:: Follow Leader
    void Ball::followLeader(GLfloat& ax, GLfloat& ay, GLfloat& az){
        
      
        GLfloat Vfx = (ballVec[0].x - x)/1;
        GLfloat Vfy = (ballVec[0].y - y)/1;
        GLfloat Vfz = (ballVec[0].z - z)/1;
    
            
        ax += lWeight*((Vfx - Vx)/1);
        ay += -1*lWeight*((Vfy - Vy)/1);
        az += lWeight*((Vfz - Vz)/1);
        
    
    }

    void Ball::getAcceleration(){
        
        resetState();
    
        
        //floor ceiling
              if(y > 1|| y < -1){
                  Vy *= -0.90;
                  getPosition(0.05);
                  while(y > 1 || y < -1){
                      getPosition(0.05);
                  }
              }

              //Left, right Wall
              if((int)x == -1 || (int)x ==1){
                  Vx = -0.90 * Vx;
                  getPosition(0.05);
                  while((int)x == -1 || (int)x ==1){
                      getPosition(0.05);
                  }
              }

              //back, front wall
              if(z > -4.0 || z < -6.0){
                  Vz *= -0.90;
                  getPosition(0.05);
                  while(z > -4.0 || z < -6.0){
                      getPosition(0.05);
                  }
              }
        
        
        GLfloat sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        int count = 0;
        
        GLfloat ax = 0.0;
        GLfloat ay=0.0;
        GLfloat az = 0.0;
        
        for(int i=1;i<ballVec.size();i++){
            setMoment(ballVec[i]);
            GLfloat dist = sqrt(  pow((x - ballVec[i].x), 2.0) + pow((y - ballVec[i].y), 2.0) + pow((z - ballVec[i].z), 2.0)  );
            
            collision(ballVec[i], ax, ay, az);
           
            
            if(dist - 0.4 < 0){
                setVelocityMatching(ballVec[i], sumX, sumY, sumZ, count);
            }
        }
        getVelocityMatching(sumX, sumY, sumZ, count, ax, ay, az);
   
        flockCentering(ax,ay,az); //TODO: Figure out a way in which we do NOT have to set velocity vector to 0 before running flockCentering
        followLeader(ax,ay,az);
        
        
        
        this->ax = ax;
        this->ay = ay;
        this->az = az;
        
        //reset default weights
           vmWeight = 0.40;
           fWeight = 0.35;
           lWeight = 0.25;
    }

    void Ball::initializeShape(std::string type){
          std::ifstream myfile;
                       std::string line;
                       int lineCount = 0;
                    std::string filename = "/Users/vishesh.javangula@ibm.com/Downloads/D-files/better-ball.d";
                           myfile.open(filename);
                       if(myfile.is_open()){
                           while(getline(myfile, line)){
                               
                               if(lineCount <= 267){
                                    storeVerticies(line, vertices);
                               }else if(lineCount >= 268){
                                   storePointers(line,vertptrs);
                               }
                                   
                               lineCount++;



                           }
                       }
                        myfile.close();
    }

void Ball::matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]) {
    int count = 0;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            rotationMatrix[count++] = m.coeffRef(j, i);
        }
    }
}


    void Ball::display(GLfloat time){
     
        
        glPushMatrix();
      
       
    
         getVelocity(time);
        getPosition(time);
        //std::cout << "y is " << y << std::endl;
        
        glTranslatef(x, y, z);
         
        
        if(index == 0){
            glPopMatrix();
            return;
        }
          
        glBegin(GL_POLYGON);
        for(int i=1;i<vertptrs.size();i++){
                 
                 for(int j=1;j<vertptrs[i].size();j++){
                     int ptr = vertptrs[i][j];
                     
                     Eigen::Vector3f m (vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);
                     GLfloat length = sqrt(m.dot(m));
                     glNormal3f(vertices[ptr][0]/length, vertices[ptr][1]/length, vertices[ptr][2]/length); //unit vector
                     
                      glVertex3f(vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);         

                 }
                

             }
          glEnd();
        glPopMatrix();
        
        
    }

