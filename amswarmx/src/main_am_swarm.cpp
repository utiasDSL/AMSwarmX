#define _USE_MATH_DEFINES
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>

#include "runner/run_am_swarm.hpp"


void waiting(){ while(1);}

int main(int argc, char **argv)
{
    
    std :: thread idle_thread;
    idle_thread = std :: thread(waiting);
    idle_thread.detach();

    
    ros::init(argc, argv, "swarm_am_nav");
    
    int success_trials = 0, total_trials = 1;

    for(int i = 0; i < total_trials; i++){
        ROS_INFO_STREAM("================== Configuration " << i << " =====================");
        AMSwarmX sim = AMSwarmX();
        total_trials = sim.trials;
        if(total_trials != 1){
            sim.json_name = sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"/jsons/"+sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"_drone_"+std::to_string(sim.num_drone)+"_"+std::to_string(i);
            if(sim.sfcc)
                sim.result_name = sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"/ours/amswarmED/"+sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"_drone_"+std::to_string(sim.num_drone)+"_"+std::to_string(i);          
            else
                sim.result_name = sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"/ours/amswarmX/"+sim.world_file_name.substr(0, sim.world_file_name.length()-3)+"_drone_"+std::to_string(sim.num_drone)+"_"+std::to_string(i);
        }
        else{
            sim.json_name = "config";
            sim.result_name = "config";
        }
        if(sim.write_files){
            // sim.writeLAUNCH();
            sim.writeJSON();
        }
        if(!sim.visualize_am)
            sim.runSimulation();
        else
            sim.runIteration();
        
        if(sim.success){
            success_trials++;
        }
    }
    
    
    ROS_INFO_STREAM("________ RUNS FINISHED! _________");
    ROS_INFO_STREAM("Success Rate = " << success_trials);
   
    return 0;
}