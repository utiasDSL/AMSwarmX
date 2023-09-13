#include "amswarmx/obstacles_utils.hpp"


void initObstacles(probData &prob_data, int VERBOSE)
{
	// prob_data.agent_obs_dist.clear();

	Eigen :: ArrayXXd val = pow(((-prob_data.x_static_obs_og).rowwise() + prob_data.x.transpose().row(0))/(prob_data.a_static_obs_og + prob_data.prox_obs), 2) 
								+ pow(((-prob_data.y_static_obs_og).rowwise() + prob_data.y.transpose().row(0))/(prob_data.b_static_obs_og + prob_data.prox_obs), 2);

	prob_data.x_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num); 
	prob_data.y_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
	prob_data.z_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);

	prob_data.a_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
	prob_data.b_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
	prob_data.c_static_obs = Eigen :: ArrayXXd :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);


	Eigen :: ArrayXd dist = -prob_data.a_static_obs_og.col(0) + prob_data.lx_drone + prob_data.buffer + (sqrt(pow((prob_data.agents_x(prob_data.id_badge, 0) - prob_data.x_static_obs_og.col(0)),2) 
								+ pow((prob_data.agents_y(prob_data.id_badge, 0) - prob_data.y_static_obs_og.col(0)),2)));
	

	int k = 0;
	prob_data.unify_obs = 0;
	for(int i = 0; i < prob_data.x_static_obs_og.rows(); i++){	
		if((val.row(i) <= 1.0).any()){
			prob_data.x_static_obs.row(k) = prob_data.x_static_obs_og.row(i);
			prob_data.y_static_obs.row(k) = prob_data.y_static_obs_og.row(i);
			prob_data.z_static_obs.row(k) = prob_data.z_static_obs_og.row(i);

			prob_data.a_static_obs.row(k) = prob_data.a_static_obs_og.row(i);
			prob_data.b_static_obs.row(k) = prob_data.b_static_obs_og.row(i);
			prob_data.c_static_obs.row(k) = prob_data.c_static_obs_og.row(i);

			
			if(prob_data.c_static_obs(k, 0) < prob_data.z_max) 
				prob_data.unify_obs++;
			
			k++;
		}	

		// prob_data.agent_obs_dist.push_back(dist(i));
	}

	if(k!=0 && prob_data.x_static_obs_og.rows() > 1){
		prob_data.x_static_obs.conservativeResize(k, prob_data.num);
		prob_data.y_static_obs.conservativeResize(k, prob_data.num);
		prob_data.z_static_obs.conservativeResize(k, prob_data.num);

		prob_data.a_static_obs.conservativeResize(k, prob_data.num);
		prob_data.b_static_obs.conservativeResize(k, prob_data.num);
		prob_data.c_static_obs.conservativeResize(k, prob_data.num);
	}
	prob_data.num_static_obs = k;

	prob_data.A_static_obs = prob_data.P;
	for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs =  stack(prob_data.A_static_obs, prob_data.P, 'v');
}

void neigbhoringAgents(probData &prob_data, int VERBOSE)
{
	// prob_data.inter_agent_dist.clear();

	Eigen :: ArrayXXd agents_x, agents_y, agents_z;

	agents_x = prob_data.agents_x;
	agents_y = prob_data.agents_y;
	agents_z = prob_data.agents_z;

	prob_data.x_drone = Eigen :: ArrayXXd :: Ones(agents_x.rows(), prob_data.num);
	prob_data.y_drone = Eigen :: ArrayXXd :: Ones(agents_y.rows(), prob_data.num);
	prob_data.z_drone = Eigen :: ArrayXXd :: Ones(agents_z.rows(), prob_data.num);

	prob_data.a_drone = Eigen :: ArrayXXd :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.lx_drone + prob_data.buffer);
	prob_data.b_drone = Eigen :: ArrayXXd :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.ly_drone + prob_data.buffer);
	prob_data.c_drone = Eigen :: ArrayXXd :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.lz_drone + prob_data.buffer);

	float del = (abs(prob_data.world - 2.0001)/(prob_data.world - 2.0001) + 1)/2;

	int k = 0;
	Eigen :: ArrayXXd val = pow(((-agents_x).rowwise() + agents_x.row(prob_data.id_badge))/(2*prob_data.lx_drone + prob_data.prox_agent), 2) 
								+ pow(((-agents_y).rowwise() + agents_y.row(prob_data.id_badge))/(2*prob_data.ly_drone + prob_data.prox_agent), 2)
								+ del * pow(((-agents_z).rowwise() + agents_z.row(prob_data.id_badge))/(2*prob_data.lz_drone + prob_data.prox_agent), 2);

	Eigen :: ArrayXd dist = (sqrt(pow((agents_x(prob_data.id_badge, 0) - agents_x.col(0)),2) 
								+ pow((agents_y(prob_data.id_badge, 0) - agents_y.col(0)),2)
								+ pow((agents_z(prob_data.id_badge, 0) - agents_z.col(0)),2))).max(0.0);

	
	for(int i = 0; i < agents_x.rows(); i++)
	{
		if(i == prob_data.id_badge)
			continue;
		if((val.row(i) <= 1.0).any()){
			prob_data.x_drone.row(k) = agents_x.row(i).rightCols(prob_data.num);
			prob_data.y_drone.row(k) = agents_y.row(i).rightCols(prob_data.num);
			prob_data.z_drone.row(k) = agents_z.row(i).rightCols(prob_data.num);
			k++; 
		}

		// prob_data.inter_agent_dist.push_back(dist(i));
	}

	
	if(k!=0 && agents_x.rows() > 1){
		prob_data.x_drone.conservativeResize(k, prob_data.num);
		prob_data.y_drone.conservativeResize(k, prob_data.num);
		prob_data.z_drone.conservativeResize(k, prob_data.num);

		prob_data.a_drone.conservativeResize(k, prob_data.num);
		prob_data.b_drone.conservativeResize(k, prob_data.num);
		prob_data.c_drone.conservativeResize(k, prob_data.num);
	}
	prob_data.num_drone = k;

	prob_data.A_drone = prob_data.P;    
	for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone =  stack(prob_data.A_drone, prob_data.P, 'v');

	if(prob_data.unify_obs!=0 && prob_data.num_static_obs!=0){
		
		if(k!=0){
			prob_data.x_drone = stack(prob_data.x_drone, prob_data.x_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
			prob_data.y_drone = stack(prob_data.y_drone, prob_data.y_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
			prob_data.z_drone = stack(prob_data.z_drone, prob_data.z_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v'); 

			prob_data.a_drone = stack(prob_data.a_drone, prob_data.a_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
			prob_data.b_drone = stack(prob_data.b_drone, prob_data.b_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
			prob_data.c_drone = stack(prob_data.c_drone, prob_data.c_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
			
			prob_data.A_drone = stack(prob_data.A_drone, prob_data.A_static_obs.bottomRows((prob_data.num_static_obs - prob_data.unify_obs)*prob_data.num), 'v'); 
		}
		else{
			prob_data.x_drone = prob_data.x_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
			prob_data.y_drone = prob_data.y_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
			prob_data.z_drone = prob_data.z_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs); 

			prob_data.a_drone = prob_data.a_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
			prob_data.b_drone = prob_data.b_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
			prob_data.c_drone = prob_data.c_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
			
			prob_data.A_drone = prob_data.A_static_obs.bottomRows((prob_data.num_static_obs - prob_data.unify_obs)*prob_data.num); 
		}
		prob_data.num_drone = prob_data.a_drone.rows();
		prob_data.num_static_obs = prob_data.unify_obs;

		if(prob_data.num_static_obs!=0){
			Eigen :: ArrayXXd temp;
			temp = prob_data.x_static_obs.topRows(prob_data.num_static_obs);
			prob_data.x_static_obs = temp;

			temp = prob_data.y_static_obs.topRows(prob_data.num_static_obs);
			prob_data.y_static_obs = temp;

			temp = prob_data.z_static_obs.topRows(prob_data.num_static_obs);
			prob_data.z_static_obs = temp;

			temp = prob_data.a_static_obs.topRows(prob_data.num_static_obs);
			prob_data.a_static_obs = temp;

			temp = prob_data.b_static_obs.topRows(prob_data.num_static_obs);
			prob_data.b_static_obs = temp;

			temp = prob_data.c_static_obs.topRows(prob_data.num_static_obs);
			prob_data.c_static_obs = temp;

			temp = prob_data.A_static_obs.topRows(prob_data.num_static_obs*prob_data.num);
			prob_data.A_static_obs = temp;
		}

	}
}

Eigen :: ArrayXXd queryDistances(probData &prob_data, int VERBOSE){

	Eigen :: ArrayXXd d_sfc = 10000.0 * Eigen :: ArrayXXd :: Ones(prob_data.d_sfc.rows(), prob_data.d_sfc.cols());

	for(int i = 0; i < prob_data.alpha_sfc.rows(); i++){
		
		double temp_dist = prob_data.distmap_obj->getDistance(octomap::point3d(prob_data.x(i), prob_data.y(i), prob_data.z(i))); 
		if(temp_dist > (2*prob_data.lx_drone + prob_data.buffer)/2.0 + prob_data.distance_to_obs_margin){
			continue;
		}

		if(!prob_data.castray){
			octomap::point3d origin(prob_data.x_anchor(i), prob_data.y_anchor(i), prob_data.z_anchor(i));
			octomap::point3d direction(cos(prob_data.alpha_sfc(i))*sin(prob_data.beta_sfc(i)), sin(prob_data.alpha_sfc(i))*sin(prob_data.beta_sfc(i)), cos(prob_data.beta_sfc(i)));
			octomap::point3d ray_end;    
			
			/////////
			castRobot(prob_data, origin, direction, ray_end, d_sfc(i));
		}
		else{
			octomap::point3d origin(prob_data.x_anchor(i), prob_data.y_anchor(i), prob_data.z_anchor(i));
			octomap::point3d direction(cos(prob_data.alpha_sfc(i))*sin(prob_data.beta_sfc(i)), sin(prob_data.alpha_sfc(i))*sin(prob_data.beta_sfc(i)), cos(prob_data.beta_sfc(i)));
			octomap::point3d ray_end;    
			
			
			bool success = prob_data.octree_ptr->castRay(origin, direction, ray_end);

			if(success){
				d_sfc(i) = std::max((origin - ray_end).norm() - (2*prob_data.lx_drone + prob_data.buffer)/2.0 - prob_data.distance_to_obs_margin, 0.0);
			}
		}
	}

	return d_sfc;
}

void castRobot(probData &prob_data, octomap::point3d &origin, octomap::point3d &direction, octomap::point3d &ray_end, double &d_sfc){
	
	bool success = prob_data.octree_ptr->castRay(origin, direction, ray_end); 
			
	bool visible = true;
	
	float x1 = origin.x();
	float y1 = origin.y();
	float z1 = origin.z();

	float x2 = ray_end.x();
	float y2 = ray_end.y();
	float z2 = ray_end.z();

	float radius_xy = (prob_data.lx_drone+prob_data.buffer/2);
	float radius_z = (prob_data.lx_drone+prob_data.buffer/2);

	float alpha = atan2(y2-y1, x2-x1);
	
	float beta = (x2 == x1) ? M_PI_2 : atan2((x2 - x1)/cos(alpha), z2 - z1);
	float range = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

	float theta_xy = asin(radius_xy/range);
	float range_xy_lr = radius_xy/tan(theta_xy);

	float theta_z = asin(radius_z/range);
	float range_z_up = radius_z/tan(theta_z);
	
	if(!isnan(range_xy_lr) && !isnan(range_z_up)){
		bool success1 = false, success2 = false, 
			success3 = false, success4 = false;
		std :: vector<float> temp_dist;
		temp_dist.resize(4);

		octomap::point3d origin(x1, y1, z1);
		octomap::point3d direction1(cos(alpha+theta_xy)*sin(beta), sin(alpha+theta_xy)*sin(beta), cos(beta));
		octomap::point3d direction2(cos(alpha-theta_xy)*sin(beta), sin(alpha-theta_xy)*sin(beta), cos(beta));
		octomap::point3d_collection ray_end_;    
		ray_end_.resize(4);

		temp_dist[0] = 1000000;
		temp_dist[1] = 1000000;
		temp_dist[2] = 1000000;
		temp_dist[3] = 1000000;

		success1 = prob_data.octree_ptr->castRay(origin, direction1, ray_end_[0], false, range_xy_lr);
		if(success1)
			temp_dist[0] = ((ray_end_[0] - origin).norm());
			
		success2 = prob_data.octree_ptr->castRay(origin, direction2, ray_end_[1], false, range_xy_lr);
		if(success2)
			temp_dist[1] = ((ray_end_[1] - origin).norm());

		if(prob_data.world == 3){
			octomap::point3d direction3(cos(alpha)*sin(beta+theta_z), sin(alpha)*sin(beta+theta_z), cos(beta+theta_z));
			octomap::point3d direction4(cos(alpha)*sin(beta-theta_z), sin(alpha)*sin(beta-theta_z), cos(beta-theta_z));

			success3 = prob_data.octree_ptr->castRay(origin, direction3, ray_end_[2], false, range_z_up);
			if(success3)
				temp_dist[2] = ((ray_end_[2] - origin).norm());

			success4 = prob_data.octree_ptr->castRay(origin, direction4, ray_end_[3], false, range_z_up);			
			if(success4)
				temp_dist[3] = ((ray_end_[3] - origin).norm());
		
		}

		if(success1 || success2 || success3 || success4){
			visible = false;
			int index_min = 0;
			float min = temp_dist[0];
			for(int j = 0; j < temp_dist.size(); j++){
				if(min > temp_dist[j]){
					min = temp_dist[j];
					index_min = j;
				}
			}
			
			auto A = origin;
			auto C = ray_end;
			auto B = ray_end_[index_min];
			auto t = ((B - A).dot(C - A))/((C - A).dot(C - A));
			auto inter = A + (C - A) * t;
			d_sfc = (origin - inter).norm() - (2*prob_data.lx_drone + prob_data.buffer)/2.0 - prob_data.distance_to_obs_margin;
			ray_end = ray_end_[index_min];
		}
	}

	if(visible){
		if(success)
			d_sfc = std::max((origin - ray_end).norm() - (2*prob_data.lx_drone + prob_data.buffer)/2.0 - prob_data.distance_to_obs_margin, 0.0);
	}
}
void getConvexPolytope2D(probData &prob_data, int VERBOSE){

	// Path to dilate
	vec_Vec2f path;
	path.push_back(Vec2f(prob_data.x_init, prob_data.y_init));
	path.push_back(Vec2f(prob_data.x_goal, prob_data.y_goal));
	
	// Initialize EllipsoidDecomp2D
	
	prob_data.decomp_util2.set_obs(prob_data.decomp_obs2);
	prob_data.decomp_util2.set_local_bbox(Vec2f(5, 5));
	prob_data.decomp_util2.dilate(path);

	auto polys = prob_data.decomp_util2.get_polyhedrons();

	for(size_t i = 0; i < path.size() - 1; i++) {
		const auto pt_inside = (path[i] + path[i+1]) / 2;
		LinearConstraint2D cs(pt_inside, polys[i].hyperplanes());
		prob_data.A_x_cp = (cs.A()).col(0);
		prob_data.A_y_cp = (cs.A()).col(1);
		prob_data.b_cp = cs.b();
		for(int i = 1; i < prob_data.num; i++){
			prob_data.A_x_cp = block_diag(prob_data.A_x_cp, (cs.A()).col(0));
			prob_data.A_y_cp = block_diag(prob_data.A_y_cp, (cs.A()).col(1));
			prob_data.b_cp = stack(prob_data.b_cp, cs.b(), 'v');
		}
	}
}

void getConvexPolytope3D(probData &prob_data, int VERBOSE){
	
	// Path to dilate
	vec_Vec3f path;
	path.push_back(Vec3f(prob_data.x_init, prob_data.y_init, prob_data.z_init));
	path.push_back(Vec3f(prob_data.x_goal+0.0000001, prob_data.y_goal+0.0000001, prob_data.z_goal+0.0000001));

	
	// Initialize EllipsoidDecomp2D
	prob_data.decomp_util.set_obs(prob_data.decomp_obs);
	prob_data.decomp_util.set_local_bbox(Vec3f(5, 5, 2));
	prob_data.decomp_util.dilate(path);

	auto polys = prob_data.decomp_util.get_polyhedrons();	
	for(size_t i = 0; i < path.size() - 1; i++) {
		const auto pt_inside = (path[i] + path[i+1]) / 2;
		LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
		prob_data.A_x_cp = (cs.A()).col(0);
		prob_data.A_y_cp = (cs.A()).col(1);
		prob_data.A_z_cp = (cs.A()).col(2);
		prob_data.b_cp = cs.b();
		for(int i = 1; i < prob_data.num; i++){
			prob_data.A_x_cp = block_diag(prob_data.A_x_cp, (cs.A()).col(0));
			prob_data.A_y_cp = block_diag(prob_data.A_y_cp, (cs.A()).col(1));
			prob_data.A_z_cp = block_diag(prob_data.A_z_cp, (cs.A()).col(2));
			prob_data.b_cp = stack(prob_data.b_cp, cs.b(), 'v');
		}
	}
}

