#include "amswarmx/select_anchor_goal.hpp"

void setGoal(probData &prob_data, int VERBOSE){

	
	if(prob_data.sfcc){
		prob_data.anchor_index = 0;
		// prob_data.x_goal = prob_data.x_anchor(0);
		// prob_data.y_goal = prob_data.y_anchor(0);
		// prob_data.z_goal = prob_data.z_anchor(0);					

		// prob_data.x_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.x_goal;
		// prob_data.y_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.y_goal;
		// prob_data.z_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.z_goal;
		for(int i = prob_data.anchor_index; i < prob_data.anchor_points.size(); i++){
			float x1 = prob_data.x_init;
			float y1 = prob_data.y_init;
			float z1 = prob_data.z_init;

			float x2 = prob_data.anchor_points[i][0];
			float y2 = prob_data.anchor_points[i][1];
			float z2 = prob_data.anchor_points[i][2];
			
			bool visible = false;
			if(prob_data.world == 2){
				Vec2f p1;
				p1 << x1, y1;

				Vec2f p2;
				p2 << x2, y2;
			
				visible = !prob_data.map_util->isBlocked(p1, p2, 1);
			}
			else{
				Vec3f p1;
				p1 << x1, y1, z1;

				Vec3f p2;
				p2 << x2, y2, z2;
			
				visible = !prob_data.map_util_->isBlocked(p1, p2, 1);
			}

			if(visible){
				
			
				prob_data.x_goal = prob_data.anchor_points[i][0];
				prob_data.y_goal = prob_data.anchor_points[i][1];
				prob_data.z_goal = prob_data.anchor_points[i][2];					

			
				prob_data.x_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.x_goal;
				prob_data.y_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.y_goal;
				prob_data.z_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.z_goal;
			
				prob_data.anchor_index = i;

				// double dist = sqrt(pow(prob_data.x_init - prob_data.anchor_points[i][0], 2) 
				// 		+ pow(prob_data.y_init - prob_data.anchor_points[i][1], 2)
				// 		+ pow(prob_data.z_init - prob_data.anchor_points[i][2], 2)); 
				// if(dist >= 1.0)
				// 	break;
			}
		}		
	}
	else{
		for(int i = prob_data.anchor_index; i < prob_data.anchor_points.size(); i++){
			bool visible = true;
			
			float x1 = prob_data.x_anchor(prob_data.num-1);
			float y1 = prob_data.y_anchor(prob_data.num-1);
			float z1 = prob_data.z_anchor(prob_data.num-1);

			float x2 = prob_data.anchor_points[i][0];
			float y2 = prob_data.anchor_points[i][1];
			float z2 = prob_data.anchor_points[i][2];

			float radius_xy = (prob_data.lx_drone+prob_data.buffer/2) + prob_data.visibility_margin;
			float radius_z = (prob_data.lx_drone+prob_data.buffer/2) + prob_data.visibility_margin;

			float alpha = atan2(y2-y1, x2-x1);
			float beta = (x2 == x1) ? M_PI_2 : atan2((x2 - x1)/cos(alpha), z2 - z1);
			float range = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
			
			float theta_xy = asin(radius_xy/range);
			float range_xy_lr = radius_xy/tan(theta_xy);

			float theta_z = asin(radius_z/range);
			float range_z_up = radius_z/tan(theta_z);
			
			if(!isnan(range_xy_lr) && !isnan(range_z_up)){
				bool success1 = false, success2 = false, success3 = false, success4 = false, 
						success5 = false, success6 = false, success7 = false, success8 = false;

				octomap::point3d origin(x1, y1, z1);
				octomap::point3d direction1(cos(alpha+theta_xy)*sin(beta), sin(alpha+theta_xy)*sin(beta), cos(beta));
				octomap::point3d direction2(cos(alpha-theta_xy)*sin(beta), sin(alpha-theta_xy)*sin(beta), cos(beta));
				octomap::point3d ray_end;    
				
				octomap::point3d origin2(x2, y2, z2);
				octomap::point3d direction3(cos(M_PI+alpha+theta_xy)*sin(beta), sin(M_PI+alpha+theta_xy)*sin(beta), cos(beta));
				octomap::point3d direction4(cos(M_PI+alpha-theta_xy)*sin(beta), sin(M_PI+alpha-theta_xy)*sin(beta), cos(beta));

				success1 = prob_data.octree_ptr->castRay(origin, direction1, ray_end, false, range_xy_lr);
				success2 = prob_data.octree_ptr->castRay(origin, direction2, ray_end, false, range_xy_lr);
				success3 = prob_data.octree_ptr->castRay(origin2, direction3, ray_end, false, range_xy_lr);
				success4 = prob_data.octree_ptr->castRay(origin2, direction4, ray_end, false, range_xy_lr);

				if(prob_data.world == 3){
					octomap::point3d direction5(cos(alpha)*sin(beta+theta_z), sin(alpha)*sin(beta+theta_z), cos(beta+theta_z));
					octomap::point3d direction6(cos(alpha)*sin(beta-theta_z), sin(alpha)*sin(beta-theta_z), cos(beta-theta_z));
					octomap::point3d direction7(cos(alpha)*sin(M_PI+beta+theta_z), sin(alpha)*sin(M_PI+beta+theta_z), cos(M_PI+beta+theta_z));
					octomap::point3d direction8(cos(alpha)*sin(M_PI+beta-theta_z), sin(alpha)*sin(M_PI+beta-theta_z), cos(M_PI+beta-theta_z));

					success5 = prob_data.octree_ptr->castRay(origin, direction5, ray_end, false, range_z_up);
					success6 = prob_data.octree_ptr->castRay(origin, direction6, ray_end, false, range_z_up);			
					success7 = prob_data.octree_ptr->castRay(origin2, direction7, ray_end, false, range_z_up);
					success8 = prob_data.octree_ptr->castRay(origin2, direction8, ray_end, false, range_z_up);
				}

				if(success1 || success2 || success3 || success4 || success5 || success6 || success7 || success8){
					visible = false;
					break;
				}
				
			}
			else{
				if(i != prob_data.anchor_points.size() - 1){
					continue;
				}
			}
			if(visible){
				
				prob_data.x_goal = prob_data.anchor_points[i][0];
				prob_data.y_goal = prob_data.anchor_points[i][1];
				prob_data.z_goal = prob_data.anchor_points[i][2];					

				prob_data.x_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.x_goal;
				prob_data.y_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.y_goal;
				prob_data.z_ref = Eigen :: ArrayXXd :: Ones(prob_data.kappa, 1) * prob_data.z_goal;

				prob_data.waypoint_index = i;
			}
		}

		prob_data.prev_waypoint_index = prob_data.waypoint_index;
	}
}


void assignAnchorPoints(probData &prob_data, int VERBOSE){
	
	int visible_index = -1;

	for(int i = prob_data.anchor_index; i < prob_data.anchor_points.size(); i++){				
		bool visible = true;
		for(int m = 0; m < prob_data.num; m++){
			
			float x1 = prob_data.anchor_points[i][0];
			float y1 = prob_data.anchor_points[i][1];
			float z1 = prob_data.anchor_points[i][2];

			float x2 = prob_data.x_init;//prob_data.x(m);
			float y2 = prob_data.y_init;//prob_data.y(m);
			float z2 = prob_data.z_init;//prob_data.z(m);

			float radius_xy = (prob_data.lx_drone+prob_data.buffer/2) + prob_data.visibility_margin;
			float radius_z = (prob_data.lx_drone+prob_data.buffer/2) + prob_data.visibility_margin;

			float alpha = atan2(y2-y1, x2-x1);
			float beta = (x2 == x1) ? M_PI_2 : atan2((x2 - x1)/cos(alpha), z2 - z1);
			float range = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

			float theta_xy = asin(radius_xy/range);
			float range_xy_lr = radius_xy/tan(theta_xy);

			float theta_z = asin(radius_z/range);
			float range_z_up = radius_z/tan(theta_z);
			
			if(!isnan(range_xy_lr) && !isnan(range_z_up)){
				bool success1 = false, success2 = false, success3 = false, success4 = false, 
					success5 = false, success6 = false, success7 = false, success8 = false;

				octomap::point3d origin(x1, y1, z1);
				octomap::point3d direction1(cos(alpha+theta_xy)*sin(beta), sin(alpha+theta_xy)*sin(beta), cos(beta));
				octomap::point3d direction2(cos(alpha-theta_xy)*sin(beta), sin(alpha-theta_xy)*sin(beta), cos(beta));
				octomap::point3d ray_end;    
				
				octomap::point3d origin2(x2, y2, z2);
				octomap::point3d direction3(cos(M_PI+alpha+theta_xy)*sin(beta), sin(M_PI+alpha+theta_xy)*sin(beta), cos(beta));
				octomap::point3d direction4(cos(M_PI+alpha-theta_xy)*sin(beta), sin(M_PI+alpha-theta_xy)*sin(beta), cos(beta));

				success1 = prob_data.octree_ptr->castRay(origin, direction1, ray_end, false, range_xy_lr);
				success2 = prob_data.octree_ptr->castRay(origin, direction2, ray_end, false, range_xy_lr);
				success3 = prob_data.octree_ptr->castRay(origin2, direction3, ray_end, false, range_xy_lr);
				success4 = prob_data.octree_ptr->castRay(origin2, direction4, ray_end, false, range_xy_lr);

				if(prob_data.world == 3){
					octomap::point3d direction5(cos(alpha)*sin(beta+theta_z), sin(alpha)*sin(beta+theta_z), cos(beta+theta_z));
					octomap::point3d direction6(cos(alpha)*sin(beta-theta_z), sin(alpha)*sin(beta-theta_z), cos(beta-theta_z));
					octomap::point3d direction7(cos(alpha)*sin(M_PI+beta+theta_z), sin(alpha)*sin(M_PI+beta+theta_z), cos(M_PI+beta+theta_z));
					octomap::point3d direction8(cos(alpha)*sin(M_PI+beta-theta_z), sin(alpha)*sin(M_PI+beta-theta_z), cos(M_PI+beta-theta_z));

					success5 = prob_data.octree_ptr->castRay(origin, direction5, ray_end, false, range_z_up);
					success6 = prob_data.octree_ptr->castRay(origin, direction6, ray_end, false, range_z_up);			
					success7 = prob_data.octree_ptr->castRay(origin2, direction7, ray_end, false, range_z_up);
					success8 = prob_data.octree_ptr->castRay(origin2, direction8, ray_end, false, range_z_up);
				}
				
				if(success1 || success2 || success3 || success4 || success5 || success6 || success7 || success8){
					visible = false;
					break;
				}	
			}
			else{
				continue;
			}	

			if(!visible)
				break;
			
			break;
		}

		if(visible){ 
			visible_index = i;
		}
	}

	for(int m = 0; m < prob_data.num; m++){
		if(visible_index != -1){
			prob_data.x_anchor(m) = prob_data.anchor_points[visible_index][0];
			prob_data.y_anchor(m) = prob_data.anchor_points[visible_index][1];
			prob_data.z_anchor(m) = prob_data.anchor_points[visible_index][2];
			prob_data.anchor_index = visible_index;
			// ROS_INFO_STREAM(prob_data.x_anchor(0) << " " << prob_data.y_anchor(0));
			// ROS_INFO_STREAM(prob_data.x_anchor(0) << " " << prob_data.y_init(0));
		}
		else{
			prob_data.x_anchor(m) = prob_data.x_init;
			prob_data.y_anchor(m) = prob_data.y_init;
			prob_data.z_anchor(m) = prob_data.z_init;
			prob_data.anchor_index = 0;
		}
	}

	
}