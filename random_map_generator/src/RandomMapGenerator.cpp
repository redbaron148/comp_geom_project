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

#define MAX_NUM_OBSTACLES_DEFAULT           30
#define MIN_NUM_OBSTACLES_DEFAULT           0
#define LARGEST_ALLOWED_OBSTACLE_DEFAULT    5
#define SMALLEST_ALLOWED_OBSTACLE_DEFAULT   1
#define MAP_HEIGHT_DEFAULT                  100
#define MAP_WIDTH_DEFAULT                   100

class RandomMapGenerator
{
public:
    RandomMapGenerator() : n("~") {
        
        map_pub = n.advertise<nav_msgs::GridCells>("rand_map",1);
        
        if(!n.getParam("max_num_obstacles", max_num_obstacles) || max_num_obstacles<0)
            max_num_obstacles=MAX_NUM_OBSTACLES_DEFAULT;
        if(!n.getParam("min_num_obstacles", min_num_obstacles) || min_num_obstacles<0)
            min_num_obstacles=MIN_NUM_OBSTACLES_DEFAULT;
        if(!n.getParam("largest_allowed_obstacle", largest_allowed_obstacle) || largest_allowed_obstacle<0)
            largest_allowed_obstacle=LARGEST_ALLOWED_OBSTACLE_DEFAULT;
        if(!n.getParam("smallest_allowed_obstacle", smallest_allowed_obstacle) || smallest_allowed_obstacle<0)
            smallest_allowed_obstacle=SMALLEST_ALLOWED_OBSTACLE_DEFAULT;
        if(!n.getParam("map_height", map_height) || map_height<0)
            map_height=MAP_HEIGHT_DEFAULT;
        if(!n.getParam("map_width", map_width) || map_width<0)
            map_width=MAP_WIDTH_DEFAULT;

        
        ROS_INFO("        max_num_obstacles: %d",max_num_obstacles);
        ROS_INFO("        min_num_obstacles: %d",min_num_obstacles);
        ROS_INFO(" largest_allowed_obstacle: %d",largest_allowed_obstacle);
        ROS_INFO("smallest_allowed_obstacle: %d",smallest_allowed_obstacle);
        ROS_INFO("               map_height: %d",map_height);
        ROS_INFO("                map_width: %d",map_width);
    }
    ~RandomMapGenerator() { ros::shutdown(); }

private:
    ros::NodeHandle n;
    
    ros::Publisher map_pub;

    int max_num_obstacles;
    int min_num_obstacles;
    int largest_allowed_obstacle;
    int smallest_allowed_obstacle;
    int map_height;
    int map_width;
    
    
    //nav_msgs::GridCells* grid;

public:
    void seed(unsigned int seed){
        srand(seed);
    }
    
    void generateMap(){
        unsigned int num_obstacles = (rand()%(max_num_obstacles+1-min_num_obstacles)) + min_num_obstacles;
        ROS_INFO("Randomly adding %d obstacles", num_obstacles);
        
        nav_msgs::GridCells grid;
        grid.cell_width=0.5;
        grid.cell_height=0.5;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "/global";
        
        for(unsigned int i=0;i<num_obstacles;i++)
        {
            geometry_msgs::Point p;
            
            p.x = rand()%(map_height+1);
            p.y = rand()%(map_width+1);
            p.z = 0;
            
            grid.cells.push_back(p);
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
