#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include "comp_geom_msgs/RandomMapParameters.h"

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
#define MIN_NUM_OBSTACLES_DEFAULT   20
#define MAX_OBSTACLE_WIDTH_DEFAULT  20
#define MIN_OBSTACLE_WIDTH_DEFAULT  1
#define MAX_OBSTACLE_HEIGHT_DEFAULT 20 
#define MIN_OBSTACLE_HEIGHT_DEFAULT 1
#define MAP_HEIGHT_DEFAULT          100
#define MAP_WIDTH_DEFAULT           100
#define PUBLISH_RATE_DEFAULT		1
#define PUBLISH_MAP_DEFAULT			true
#define START_DEFAULT_DEFAULT		false

class RandomMapGenerator
{
public:
    RandomMapGenerator() : n("~") {
        init();
        fetch_params();
        
        ROS_INFO("publish_rate: %d Hz",publish_rate);
        ROS_INFO(" publish_map: %s",(publish_map==true?"true":"false"));
        ROS_INFO("  always_new: %s",(start_default==true?"true":"false"));
    }
    ~RandomMapGenerator() { ros::shutdown(); }

private:
    ros::NodeHandle n;
    
    ros::Publisher map_pub;
    ros::Subscriber new_map_sub;

    int publish_rate;
    bool publish_map;
	bool start_default;
	
	nav_msgs::GridCells* grid;
	
	void init(){
		publish_rate=PUBLISH_RATE_DEFAULT;
		publish_map=PUBLISH_MAP_DEFAULT;
		start_default=START_DEFAULT_DEFAULT;
		
		map_pub = n.advertise<nav_msgs::GridCells>("map",1);
		new_map_sub = n.subscribe("new_map_parameters",1,&RandomMapGenerator::newMapCallback,this);
		
		grid = NULL;
	}
	
	void fetch_params(){
        if(!n.getParam("publish_rate", publish_rate))
            ROS_DEBUG("No value set for parameter map_publish_rate");
        if(!n.getParam("publish_map", publish_map))
			ROS_DEBUG("No value set for parameter publish_map");
		if(!n.getParam("start_default", start_default))
			ROS_DEBUG("No value set for parameter start_default");
		if(publish_rate<=0){
            publish_rate=PUBLISH_RATE_DEFAULT;
            ROS_WARN("Invalid value, setting map_publish_rate to default.");
		}
	}
	
	void validate_params(comp_geom_msgs::RandomMapParameters& params){
		if(params.max_num_obstacles<params.min_num_obstacles){
			ROS_WARN("Invalid request, using default max_num_obstacles.");
			params.max_num_obstacles=MAX_NUM_OBSTACLES_DEFAULT;
		}
		if(params.min_num_obstacles>params.max_num_obstacles){
			ROS_WARN("Invalid request, using default min_num_obstacles.");
			params.min_num_obstacles=MIN_NUM_OBSTACLES_DEFAULT;
		}
		if(params.max_obstacle_width<params.min_obstacle_width){
			ROS_WARN("Invalid request, using default max_obstacle_width.");
			params.max_obstacle_width=MAX_OBSTACLE_WIDTH_DEFAULT;
		}
		if(params.min_obstacle_width>params.max_obstacle_width){
			ROS_WARN("Invalid request, using default min_obstacle_width.");
			params.min_obstacle_width=MIN_OBSTACLE_WIDTH_DEFAULT;
		}
		if(params.max_obstacle_height<params.min_obstacle_height){
			ROS_WARN("Invalid request, using default max_obstacle_height.");
			params.max_obstacle_height=MAX_OBSTACLE_HEIGHT_DEFAULT;
		}
		if(params.min_obstacle_height>params.max_obstacle_height){
			ROS_WARN("Invalid request, using default min_obstacle_height.");
			params.min_obstacle_height=MIN_OBSTACLE_HEIGHT_DEFAULT;
		}
	}
	
	void newMapCallback(comp_geom_msgs::RandomMapParameters msg){
		if(grid != NULL) delete grid;
		
		validate_params(msg);
		
		grid = generateMap(msg.max_num_obstacles,msg.min_num_obstacles,
					msg.max_obstacle_width,msg.min_obstacle_width,msg.max_obstacle_height,
					msg.min_obstacle_height,msg.map_height,msg.map_width);
		return;
	}
	
	nav_msgs::GridCells* generateMap(const unsigned int &max_num_obstacles, 
		const unsigned int &min_num_obstacles, const unsigned int &max_obstacle_width, 
		const unsigned int &min_obstacle_width, const unsigned int &max_obstacle_height, 
		const unsigned int &min_obstacle_height, const unsigned int &map_height, 
		const unsigned int &map_width){
			
        unsigned int num_obstacles = (rand()%(max_num_obstacles+1-min_num_obstacles)) + min_num_obstacles;
        ROS_INFO("Generating a new map.\n\tmax_num_obstacles:\t%d\n\tmin_num_obstacles:\t%d\n\tmax_obstacle_height:\t%d\n\tmin_obstacle_height:\t%d\n\tmax_obstacle_width:\t%d\n\tmin_obstacle_width:\t%d\n\tmap_width:\t\t%d\n\tmap_height:\t\t%d",
				max_num_obstacles, min_num_obstacles, max_obstacle_height, min_obstacle_height, max_obstacle_width, min_obstacle_width, map_width, map_height);
        
        int x;
        int y;
        unsigned int width;
        unsigned int height;
        
        nav_msgs::GridCells* _grid = new nav_msgs::GridCells();
        
        geometry_msgs::Point origin;
        origin.x=0;
        origin.y=0;
        origin.z=0;
        _grid->cells.push_back(origin);
        
        _grid->cell_width=1;
        _grid->cell_height=1;
        _grid->header.stamp = ros::Time::now();
        _grid->header.frame_id = "/map";
        
        for(unsigned int i=0;i<num_obstacles;i++)
        {
            width = (rand()%(max_obstacle_width+1-min_obstacle_width));
            height = (rand()%(max_obstacle_height+1-min_obstacle_height));
            
            x = rand()%(map_height);
            y = rand()%(map_width);
            
            for(unsigned int h=0;h<height && (h+y)<map_height;h++){
                for(unsigned int w=0;w<width && (w+x)<map_width;w++)
                {
                    geometry_msgs::Point p;
                    p.x=x+w;
                    p.y=y+h;
                    p.z=0;
                    
                    _grid->cells.push_back(p);
                }
            }
        }
        return _grid;
    }

public:
    void seed(unsigned int seed){
        srand(seed);
    }
    
    void spin(){
		ros::Rate r(publish_rate);
		if(start_default) grid = generateMap(MAX_NUM_OBSTACLES_DEFAULT,
									MIN_NUM_OBSTACLES_DEFAULT,MAX_OBSTACLE_WIDTH_DEFAULT,
									MIN_OBSTACLE_WIDTH_DEFAULT,MAX_OBSTACLE_HEIGHT_DEFAULT,
									MIN_OBSTACLE_HEIGHT_DEFAULT,MAP_HEIGHT_DEFAULT,MAP_WIDTH_DEFAULT);
		while(ros::ok()){
			if(publish_map) {
				if(grid!=NULL){
					ROS_DEBUG("Publishing map");
					map_pub.publish(*grid);
				}
				else ROS_DEBUG("No map to publish");
			}
			r.sleep();
			ros::spinOnce();
		}
	}
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_map_generator");
    RandomMapGenerator rmg;
    
    rmg.spin();
    
    return 0;
}
