#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"

/**
 * @author  Aaron Parker
 * @date    10.22.2012
 * @file    RandomMapGenerator.cpp
 * @project CS 590 - final project
 * @brief   Generates and publishes a randomly created map in many different 
 *          formats (OccupancyGrid, ...)
 **/

//      Redistribution and use in source and binary forms, with or without
//      modification, are permitted provided that the following conditions are
//      met:
//      
//      * Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//      * Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the following disclaimer
//        in the documentation and/or other materials provided with the
//        distribution.
//      * Neither the name of the SIUE nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//      
//      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#define MAX_NUM_OBSTACLES_DEFAULT   30
#define MIN_NUM_OBSTACLES_DEFAULT   0
#define MAX_OBSTACLE_WIDTH_DEFAULT  20
#define MIN_OBSTACLE_WIDTH_DEFAULT  1
#define MAX_OBSTACLE_HEIGHT_DEFAULT 20 
#define MIN_OBSTACLE_HEIGHT_DEFAULT 1
#define MAP_HEIGHT_DEFAULT          100
#define MAP_WIDTH_DEFAULT           100

class RandomMapGenerator
{
public:
    RandomMapGenerator() : n("~") {
        
        map_pub = n.advertise<nav_msgs::GridCells>("rand_map",1);
        
        if(!n.getParam("max_num_obstacles", max_num_obstacles) || max_num_obstacles<0)
            max_num_obstacles=MAX_NUM_OBSTACLES_DEFAULT;
        if(!n.getParam("min_num_obstacles", min_num_obstacles) || min_num_obstacles<0)
            min_num_obstacles=MIN_NUM_OBSTACLES_DEFAULT;
        if(!n.getParam("max_obstacle_width", max_obstacle_width) || max_obstacle_width<0)
            max_obstacle_width=MAX_OBSTACLE_WIDTH_DEFAULT   ;
        if(!n.getParam("min_obstacle_width", min_obstacle_width) || min_obstacle_width<0)
            min_obstacle_width=MIN_OBSTACLE_WIDTH_DEFAULT;
        if(!n.getParam("max_obstacle_height", max_obstacle_height) || max_obstacle_height<0)
            max_obstacle_height=MAX_OBSTACLE_HEIGHT_DEFAULT;
        if(!n.getParam("min_obstacle_height", min_obstacle_height) || min_obstacle_height<0)
            min_obstacle_height=MIN_OBSTACLE_HEIGHT_DEFAULT;
        if(!n.getParam("map_height", map_height) || map_height<0)
            map_height=MAP_HEIGHT_DEFAULT;
        if(!n.getParam("map_width", map_width) || map_width<0)
            map_width=MAP_WIDTH_DEFAULT;

        
        ROS_INFO("  max_num_obstacles: %d",max_num_obstacles);
        ROS_INFO("  min_num_obstacles: %d",min_num_obstacles);
        ROS_INFO(" max_obstacle_width: %d",max_obstacle_width);
        ROS_INFO(" min_obstacle_width: %d",min_obstacle_width);
        ROS_INFO("max_obstacle_height: %d",max_obstacle_height);
        ROS_INFO("min_obstacle_height: %d",min_obstacle_height);
        ROS_INFO("         map_height: %d",map_height);
        ROS_INFO("          map_width: %d",map_width);
    }
    ~RandomMapGenerator() { ros::shutdown(); }

private:
    ros::NodeHandle n;
    
    ros::Publisher map_pub;

    int max_num_obstacles;
    int min_num_obstacles;
    int max_obstacle_width;
    int min_obstacle_width;
    int max_obstacle_height;
    int min_obstacle_height;
    int map_height;
    int map_width;

public:
    void seed(unsigned int seed){
        srand(seed);
    }
    
    void generateMap(){
        unsigned int num_obstacles = (rand()%(max_num_obstacles+1-min_num_obstacles)) + min_num_obstacles;
        ROS_INFO("Randomly adding %d obstacles", num_obstacles);
        
        int x;
        int y;
        int width;
        int height;
        
        nav_msgs::GridCells grid;
        
        geometry_msgs::Point origin;
        origin.x=0;
        origin.y=0;
        origin.z=0;
        grid.cells.push_back(origin);
        
        grid.cell_width=1;
        grid.cell_height=1;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "/global";
        
        for(unsigned int i=0;i<num_obstacles;i++)
        {
            //geometry_msgs::Point centroid;
            width = (rand()%(max_obstacle_width+1-min_obstacle_width));
            height = (rand()%(max_obstacle_height+1-min_obstacle_height));
            
            x = rand()%(map_height+1);
            y = rand()%(map_width+1);
            
            for(unsigned int h=0;h<height;h++){
                for(unsigned int w=0;w<width;w++)
                {
                    geometry_msgs::Point p;
                    p.x=x+w;
                    p.y=y+h;
                    p.z=0;
                    
                    grid.cells.push_back(p);
                }
            }
        }
        
        map_pub.publish(grid);
        return;
    }
    
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "random_map_generator");
    RandomMapGenerator rmg;
    //rmg.seed(time(NULL));
    
    ros::Rate r(1);
    while(ros::ok()){
        rmg.generateMap();
        r.sleep();
    }
    //ros::spin();
    return 0;
}
