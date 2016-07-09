#define INFLATED_OBSTACLE 200

#include "RoverPath.h"
using namespace Eigen;
using costmap_2d::LETHAL_OBSTACLE;


		
RoverPathClass::RoverPathClass(double b_,int sample_, costmap_2d::Costmap2D* grid_)
{
	Rover_b = b_;
	sample = sample_;
	master_grid_ = grid_;
	
	set_pso_params_default();
	Travel_cost_inc = 0.0;
	Lethal_cost_inc = 10.0;
	Inf_cost_inc = 3.0;
}
/*
RoverPathClass::~RoverPathClass()	
{

}*/
void RoverPathClass::set_path_params(double Travel_cost_inc_,double Lethal_cost_inc_,double Inf_cost_inc_)
{
	Travel_cost_inc = Travel_cost_inc_;
	Lethal_cost_inc = Lethal_cost_inc_;
	Inf_cost_inc = Inf_cost_inc_;

}	
void RoverPathClass::set_pso_params(double pso_inertia_,double c_1_,double c_2_,double Goal_gain_,double Cost_gain_,double Speed_gain_,int particle_no_,int iteration_)
{
	
	pso_inertia = pso_inertia_;
	c_1 = c_1_;
	c_2 = c_2_;
	Goal_gain = Goal_gain_;
	Cost_gain = Cost_gain_;
	Speed_gain = Speed_gain_;
	particle_no = particle_no_;
  	iteration = iteration_;
}
	
void RoverPathClass::set_pso_params_default()
{
	pso_inertia = 0.1;
	c_1 = 0.45;
	c_2 = 0.45;
	Goal_gain = 30.0;
	Cost_gain = 1.0;
	Speed_gain = 0.0;
	particle_no = 15;
  	iteration = 5;
}
	
void RoverPathClass::update_costmap(costmap_2d::Costmap2D* grid_)
{
	master_grid_ = grid_;
}
	
void RoverPathClass::traj_to_cloud(MatrixXf tra)
{
		
	for(size_t i = 0; i < tra.cols(); i++)
	{
		pcl::PointXYZ point;
		point.x = tra(0,i);
		point.y = tra(1,i);
		point.z = 0.000;
			
		path_trace_pcl.points.push_back(point);
	}
		
}
	
pcl::PointCloud<pcl::PointXYZ> RoverPathClass::get_path_trace_cloud()
{
	return path_trace_pcl;
}
	
PATH_COST RoverPathClass::Cost_of_path(MatrixXf path, costmap_2d::Costmap2D* grid)
{
	CELL prev_cell;
	CELL curr_cell;
	prev_cell.x = 0;
	prev_cell.y = 0;
	PATH_COST cost;
	cost.Lethal_cost = 0.0;
	cost.Travel_cost = Travel_cost_inc;
	cost.Inf_cost = 0.0;
	cost.collision = false;
	
	for(size_t i=0; i < path.cols(); i++)
	{
		grid->worldToMap((double) path(0,i),(double) path(1,i),curr_cell.x,curr_cell.y);
		
		if( (curr_cell.x != prev_cell.x) && (curr_cell.x != prev_cell.x) )
		{
			curr_cell.c = grid->getCost(curr_cell.x,curr_cell.y);
			if (curr_cell.c == LETHAL_OBSTACLE)
			{
				cost.Lethal_cost += Lethal_cost_inc;
				cost.collision = true;
			}
			if (curr_cell.c == INFLATED_OBSTACLE)
			{
				cost.Inf_cost += Inf_cost_inc; 
			}
			cost.Travel_cost +=  Travel_cost_inc;
			prev_cell = curr_cell;
		}
	}
	return cost;
	}
		
MatrixXf Rover_vw(VectorXf V_input, VectorXf Omega_input, double b, double Ts,Vector3f x_0,Vector3f x_dot_0 , int sample, Vector3f x_dot_f)
{

	MatrixXf x;
	x.setZero(3,sample);
    	MatrixXf x_dot;
    	MatrixXf NE_dot_temp;
    	MatrixXf Rot_temp;
    	MatrixXf V_temp;
        
    	double dt = Ts / ((double)sample); 

    	x_dot.setZero(3,sample);
    	
   	x.col(0) = x_0;
    	NE_dot_temp.setZero(2,sample);
   	NE_dot_temp.col(0) = x_dot_0.topRows(2);
    
    	Rot_temp.setIdentity(2,2);
    	V_temp.setZero(2,1);
        
    	for(size_t i=1; i < sample; i++)
    	  {
     		x(2,i) = x(2,i-1) + Omega_input(i);
     
     		Rot_temp(0,0) =    cos(x(2,i));
     		Rot_temp(0,1) = -b*sin(x(2,i));
     		Rot_temp(1,0) =    sin(x(2,i));
     		Rot_temp(1,1) =  b*cos(x(2,i));
     
     		V_temp(0,0)  = V_input(i);
     		V_temp(0,1)  = Omega_input(i);
     		NE_dot_temp.col(i) = Rot_temp * V_temp;
     		x_dot(0,i) = NE_dot_temp(0,i);
     		x_dot(1,i) = NE_dot_temp(1,i);
     		x_dot(2,i) = Omega_input(i);
     
     		x(0,i) = x(0,i-1)+x_dot(0,i)*dt;
     		x(1,i) = x(1,i-1)+x_dot(1,i)*dt;
        
    	   }
    	   
    	x_dot_f = x_dot.rightCols(sample-1);   
    	
    	
	return x;  
}
	
MatrixXf RoverPathClass::PSO_path_finder(Vector3f Goal,float D,Vector2f V_curr_c,VectorXf output, bool solution_found)
{
	ROS_INFO("PSO Starts!... GOAL:");
	std::cout << Goal << std::endl;
	
	/*       particle structure  
	       n Particle and m piece
	| v11 v12 ...particle N.O. ... v1n|
	| w11 w12 ...particle N.O. ... w1n|
	|	        ...	          |
	|	     Piece N.O.	          | 
	|	        ...	          |
	| vmn vmn ...particle N.O. ... vmn|
	| wm1 wm2 ...particle N.O. ... wmn|
	*/
	
	//Definition
	int piece_no = 3;
	MatrixXf x;  //patricle
	VectorXf x_best(2*piece_no);
	VectorXf G(2*piece_no);
	MatrixXf v;  //particle_increment
   	MatrixXf output_tra;
   	output_tra.setZero(3,sample);
   	
   	
	float G_cost = 1.0/0.0;
	float x_best_cost = 1.0/0.0;
	
	
   
        // Init particles Start
	x.setOnes(2*piece_no,particle_no);
	//x.setOnes(2,particle_no);
	
	for(size_t i=0;i < 2*piece_no ;i++)  // First element set
	{
	
		x(i,0) = V_curr_c(0);
		i++;
		x(i,0) = V_curr_c(1);
	}
	/*
	x(0,0) = V_curr_c(0);   // First element set
	x(1,0) = V_curr_c(1);*/
	
	
	ROS_INFO_STREAM("V_curr_c is -------->  "  << V_curr_c);

	float rand_v;
	float rand_w;
	for(size_t i=1;i < x.cols();i++)
	{
	    for(size_t j=0; j< 2*piece_no ; j++)
	    {
		rand_v = ((float) (rand() % 40))/100 + 0.8;
		rand_w  = ((float) (rand() % 200))/100 -1.0;
		x(j,i)  = rand_v * V_curr_c(0); //fixed linear speed
		j++;
		x(j,i)  = rand_w * V_curr_c(1);
	    }
	}

	v.setZero(2*piece_no,x.cols());
	

	// Init particle End
	       
	ROS_INFO("Initial particle");
	std::cout << x << "\n";

	        
	solution_found = false;
	
	for(size_t i=0;i< 2*piece_no; i++)
	{
	G(i) = x(i,0);
	}
	/*G(0) = x(0,0);
	G(1) = x(1,0);*/
	
	x_best = G;
  		
  	double Ts= 3.0;
  	Vector3f x_0;
  	x_0 << 0.0, 0.0, 0.0;
  	Vector3f x_dot_0;
  	x_dot_0 << 0.0, 0.0, 0.0;
  	
  	//outputs
  	Vector3f x_dot_f;
  	VectorXf V_in;
  	VectorXf Omega_in;
  	MatrixXf tra;
  	
  	V_in.setOnes(sample);
  	Omega_in.setOnes(sample);
  	
    	
	for (size_t k = 0; k < iteration; k++)
	{
		
    		
    		MatrixXf tra;
		tra.setZero(3,sample);
		for(size_t i=0; i < particle_no; i++)
		{
			float r_1  = ((float) (rand() % 200))/100 -1.0;
			float r_2  = ((float) (rand() % 200))/100 -1.0;	
		
			
			int sub_sample = floor(V_in.size()/piece_no);
			size_t row_it = 0;
			for(size_t jj=0;jj < V_in.size() ; jj++)
			{       //                          first_iteration    in case sample % piece_no is not 0                
				if ( (jj%sub_sample) == 0  &&    jj != 0 &&    (sub_sample*piece_no - row_it) > 1 ) row_it = row_it+2;
				V_in(jj)     = x(row_it,i);
				Omega_in(jj) = x(row_it+1,i);
			}
			
			
		 	
			//simulating the trajectory
			tra = Rover_vw(V_in, Omega_in, Rover_b, Ts,x_0,x_dot_0 , sample,x_dot_f);
			traj_to_cloud(tra);
			
		
			
			Vector3f tra_tail;
			tra_tail(0) = tra(0,tra.cols()-1);
			tra_tail(1) = tra(1,tra.cols()-1);
			tra_tail(2) = 0;
			
			
			//Calculating the cost of trajectory
			PATH_COST cost = Cost_of_path(tra, master_grid_);
			
		
			
			float prop_speed = fabs(x(0,i));
			
			//Defining the objective function
			float Ob_func_1 = sqrtf( pow((tra_tail(0)-Goal(0)), 2) + pow((tra_tail(1)-Goal(1)), 2) );    //effect of distance from the goal
			float Ob_func_2 = (cost.Lethal_cost + cost.Inf_cost);				     	     //path cost
			float Ob_func_3 = fabs(V_curr_c(0) - prop_speed);			      		     //speed effect
			
			float Ob_func = Goal_gain *Ob_func_1 + Cost_gain *Ob_func_2 + Speed_gain * Ob_func_3;
				      
				      
			if (Ob_func < x_best_cost)
			{
				x_best_cost = Ob_func;
				for (size_t jj=0; jj < x.rows();jj++) x_best(jj) = x(jj,i);
				ROS_INFO("new value for x_best_cost");	
			}
			if (Ob_func < G_cost)
			{
				G_cost = Ob_func;
				for (size_t jj=0; jj < x.rows();jj++) G(jj) = x(jj,i);
				output_tra = tra;
				if (cost.Lethal_cost < 1) solution_found = true;
				ROS_WARN(" ------>  new value for G_cost");
			}
			if(i==0) //Reseting X_best and its cost in each iteration
			{
				for (size_t jj=0; jj < x.rows();jj++) x_best(jj) = x(jj,i);
				
				x_best_cost = Ob_func;
			}
			
			for (size_t jj=0; jj < x.rows();jj++)
				v(jj,i) = pso_inertia * v(jj,i) + c_1 * r_1 * (x_best(jj) - x(jj,i)) + c_2 * r_2 * (G(jj) - x(jj,i));
			
			
		
		}
		x = x+v;
	
			
	}
    	
	output = G;
	return output_tra;
    
}

