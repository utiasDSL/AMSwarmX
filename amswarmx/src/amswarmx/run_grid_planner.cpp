#include "amswarmx/run_grid_planner.hpp"

using namespace JPS;

void getGrid2DPath(probData &prob_data, int VERBOSE){

	
	prob_data.grid_x.setLinSpaced((int)((prob_data.world_dimension[0][1] - prob_data.world_dimension[0][0])/prob_data.grid_resolution), prob_data.world_dimension[0][0]+prob_data.grid_resolution/2, prob_data.world_dimension[0][1]-prob_data.grid_resolution/2);
	prob_data.grid_y.setLinSpaced((int)((prob_data.world_dimension[1][1] - prob_data.world_dimension[1][0])/prob_data.grid_resolution), prob_data.world_dimension[1][0]+prob_data.grid_resolution/2, prob_data.world_dimension[1][1]-prob_data.grid_resolution/2);
	prob_data.grid_z.setLinSpaced(1, prob_data.z_init, prob_data.z_init);	
	
	// Update map
	if(prob_data.mpc_step == 0){
		int temp = 0;
		
		prob_data.grid_data.resize(prob_data.grid_x.rows()*prob_data.grid_y.rows());
		for(int i = 0; i < prob_data.grid_y.rows(); i++){
			for(int j = 0; j < prob_data.grid_x.rows(); j++){
				float dist = prob_data.distmap_obj->getDistance(octomap::point3d(prob_data.grid_x(j), prob_data.grid_y(i), prob_data.z_init));
				
				if (dist <= (2*prob_data.lx_drone + prob_data.buffer)/2.0 + prob_data.grid_margin){
					prob_data.grid_data[temp++] = 1;
					if (dist <= (2*prob_data.lx_drone + prob_data.buffer)/2.0 + sqrt(2)*prob_data.world_resolution/2.0)
						prob_data.decomp_obs2.push_back(Vec2f(prob_data.grid_x(j), prob_data.grid_y(i)));
				}
				else
					prob_data.grid_data[temp++] = 0;
			}
		}
	}
	

	Eigen :: MatrixXd origin_jps(2, 1), start_jps(2, 1), goal_jps(2, 1);
	origin_jps << prob_data.grid_x(0), prob_data.grid_y(0);
	start_jps << prob_data.x_init, prob_data.y_init;
	goal_jps << prob_data.xf_goal, prob_data.yf_goal;


	Eigen :: MatrixXi dim_jps(2, 1), start_int(2, 1), goal_int(2, 1);
	dim_jps << prob_data.grid_x.rows(), prob_data.grid_y.rows();

	prob_data.map_util = std::make_shared<OccMapUtil>();
	prob_data.map_util->setMap(origin_jps, dim_jps, prob_data.grid_data, prob_data.grid_resolution);

	if(prob_data.mpc_step == 0){
		start_int = prob_data.map_util->floatToInt(start_jps);
		prob_data.map_util->setFreeVoxelAndSurroundings2D(start_int, 0.15);
	}
	goal_int = prob_data.map_util->floatToInt(goal_jps);
	prob_data.map_util->setFreeVoxelAndSurroundings2D(goal_int, 0.05);
	std::unique_ptr<JPSPlanner2D> planner_ptr(new JPSPlanner2D(VERBOSE == 6 ? true : false)); 
	planner_ptr->setMapUtil(prob_data.map_util); 
	planner_ptr->updateMap();

	bool valid_jps = planner_ptr->plan(start_jps, goal_jps, 1, prob_data.jps);
	if(valid_jps){
		auto path_jps = planner_ptr->getRawPath();
		prob_data.grid_path.clear();
		prob_data.anchor_points.clear();
		
		// if(prob_data.sfcc)
		{
			// Set up DMP planner
			DMPlanner2D dmp(false);
			dmp.setPotentialRadius(Vec2f(1.0, 1.0)); // Set 2D potential field radius
			dmp.setSearchRadius(Vec2f(0.5, 0.5)); // Set the valid search region around given path
			dmp.setMap(prob_data.map_util, start_jps); // Set map util for collision checking, must be called before planning

			// Run DMP planner
			bool valid_dist = dmp.computePath(start_jps, goal_jps, path_jps); // Compute the path given the jps path
			path_jps = dmp.getRawPath();
		}
		for(const auto& it: path_jps){
			prob_data.grid_path.push_back({it.x(), it.y(), prob_data.z_init});
			prob_data.anchor_points.push_back({it.x(), it.y(), prob_data.z_init});
		}
	}
}

void getGrid3DPath(probData &prob_data, int VERBOSE){

	prob_data.grid_x.setLinSpaced((int)((prob_data.world_dimension[0][1] - prob_data.world_dimension[0][0])/prob_data.grid_resolution), prob_data.world_dimension[0][0]+prob_data.grid_resolution/2, prob_data.world_dimension[0][1]-prob_data.grid_resolution/2);
	prob_data.grid_y.setLinSpaced((int)((prob_data.world_dimension[1][1] - prob_data.world_dimension[1][0])/prob_data.grid_resolution), prob_data.world_dimension[1][0]+prob_data.grid_resolution/2, prob_data.world_dimension[1][1]-prob_data.grid_resolution/2);
	prob_data.grid_z.setLinSpaced((int)((prob_data.world_dimension[2][1] - prob_data.world_dimension[2][0])/prob_data.grid_resolution), prob_data.world_dimension[2][0]+prob_data.grid_resolution/2, prob_data.world_dimension[2][1]-prob_data.grid_resolution/2);	
	
	// Update map
	if(prob_data.mpc_step == 0){
		int temp = 0;
		
		prob_data.grid_data.resize(prob_data.grid_x.rows()*prob_data.grid_y.rows()*prob_data.grid_z.rows());
		
		for(int k = 0; k < prob_data.grid_z.rows(); k++){
			for(int i = 0; i < prob_data.grid_y.rows(); i++){
				for(int j = 0; j < prob_data.grid_x.rows(); j++){
					float dist = prob_data.distmap_obj->getDistance(octomap::point3d(prob_data.grid_x(j), prob_data.grid_y(i), prob_data.grid_z(k)));
					
					if (dist <= (2*prob_data.lx_drone + prob_data.buffer)/2.0 + prob_data.grid_margin){ 
						prob_data.grid_data[temp++] = 1;
						// if (dist <= (2*prob_data.lx_drone + prob_data.buffer)/2.0 + sqrt(3)*prob_data.world_resolution/3.0){
							prob_data.decomp_obs.push_back(Vec3f(prob_data.grid_x(j), prob_data.grid_y(i), prob_data.grid_z(k)));
						// }
					}
					else
						prob_data.grid_data[temp++] = 0;
				}
			}
		}
	}
	Eigen :: MatrixXd origin_jps(3, 1), start_jps(3, 1), goal_jps(3, 1);
	origin_jps << prob_data.grid_x(0), prob_data.grid_y(0), prob_data.grid_z(0);
	start_jps << prob_data.x_init, prob_data.y_init, prob_data.z_init;
	goal_jps << prob_data.xf_goal, prob_data.yf_goal, prob_data.zf_goal;


	Eigen :: MatrixXi dim_jps(3, 1), start_int(3, 1), goal_int(3, 1);
	dim_jps << prob_data.grid_x.rows(), prob_data.grid_y.rows(), prob_data.grid_z.rows();
	
	prob_data.map_util_ = std::make_shared<VoxelMapUtil>();
	prob_data.map_util_->setMap(origin_jps, dim_jps, prob_data.grid_data, prob_data.grid_resolution);

	if(prob_data.mpc_step == 0){
		start_int = prob_data.map_util_->floatToInt(start_jps);
		prob_data.map_util_->setFreeVoxelAndSurroundings3D(start_int, 0.05);
	}
	goal_int = prob_data.map_util_->floatToInt(goal_jps);
	prob_data.map_util_->setFreeVoxelAndSurroundings3D(goal_int, 0.05);

	std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(VERBOSE == 6 ? true : false)); 
	planner_ptr->setMapUtil(prob_data.map_util_); 
	planner_ptr->updateMap();

	bool valid_jps = planner_ptr->plan(start_jps, goal_jps, 1, prob_data.jps);
	if(valid_jps){
		auto path_jps = planner_ptr->getRawPath();
		prob_data.grid_path.clear();
		prob_data.anchor_points.clear();
		
		if(prob_data.sfcc){
			// Set up DMP planner
			DMPlanner3D dmp(false);
			dmp.setPotentialRadius(Vec3f(1.0, 1.0, 1.0)); // Set 2D potential field radius
			dmp.setSearchRadius(Vec3f(0.5, 0.5, 0.5)); // Set the valid search region around given path
			dmp.setMap(prob_data.map_util_, start_jps); // Set map util for collision checking, must be called before planning

			// Run DMP planner
			bool valid_dist = dmp.computePath(start_jps, goal_jps, path_jps); // Compute the path given the jps path
			path_jps = dmp.getRawPath();
		}

		for(const auto& it: path_jps){
			prob_data.grid_path.push_back({it.x(), it.y(), it.z()});
			prob_data.anchor_points.push_back({it.x(), it.y(), it.z()});
		}
	}
}