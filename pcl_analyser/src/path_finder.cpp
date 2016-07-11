


class PSOpathfinderclass
{
	public:
	
	PSOpathfinderclass()
	{
	
	}
	
	
	void PSO_path_finder(Vector3f Goal,float D,Vector2f V_curr_c,size_t particle_no,size_t iteration,Matrix3f output, MatrixXf output_tra, bool solution_found)
	{
	ROS_INFO("PSO Starts!... GOAL:");
	std::cout << Goal << std::endl;
	
	
	//Definition
	MatrixXf x;  //patricle
	Vector2f x_best;
	Vector2f G;
	MatrixXf v;  //particle_increment
   	
	float G_cost = 1.0/0.0;
	float x_best_cost = 1.0/0.0;
	
	
	//PSO params
	/*
	double pso_inertia = 0.1;
	double c_1 = 0.45;
	double c_2 = 0.45;
	double Goal_gain = 30.0;
	double Cost_gain = 1.0;
	double Speed_gain = 0.0;*/
	
	
	
	
	//Objective function params
	
	
	bool Once_ = false; //debug
	
   
        // Init particles Start
	x.setOnes(2,particle_no);
	x(0,0) = V_curr_c(0);   // First element set
	x(1,0) = V_curr_c(1);
	ROS_INFO_STREAM("V_curr_c is -------->  "  << V_curr_c);

	//float rand_v;
	float rand_w;
	for(size_t i=1;i < x.cols();i++)
	{
		rand_w  = ((float) (rand() % 200))/100 -1.0;
		x(0,i)  = V_curr_c(0); //fixed linear speed
		x(1,i)  = rand_w * V_curr_c(1);
	}
	v.setZero(2,x.cols());


	// Init particle End
	       
	ROS_INFO("Initial particle");
	std::cout << x << "\n";

	        
	solution_found = false;
	G(0) = x(0,0);
	G(1) = x(1,0);

	x_best = G;
  		
  	double Ts;
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
  	
  	path_z_inc = 0.0;
    	
	for (size_t k = 0; k < iteration; k++)
	{
		
    		
    		MatrixXf tra;
		tra.setZero(3,sample);
		for(size_t i=0; i < particle_no; i++)
		{
			float r_1  = ((float) (rand() % 200))/100 -1.0;
			float r_2  = ((float) (rand() % 200))/100 -1.0;
		ROS_INFO_STREAM("Particle:" << "\n" << x);	
		ROS_INFO("r_1:%f, r_2:%f", r_1,r_2);	
		
			//first part of trajectory: tra_0
			Ts = 3.0;
			
			for(size_t jj=0;jj < V_in.size() ; jj++)
			{
				V_in(jj) = x(0,i);
				Omega_in(jj) = x(1,i);
			}
		
			
			
			/*//DEBUG -show
			if(!Once_){
				std::cout << "V_in" << "\n";
				std::cout <<  V_in  << "\n";
				std::cout << "Omega_in" << "\n";
				std::cout <<  Omega_in  << "\n";			
				Once_ = true;
			}
			//DEBUG - end*/
		 	
			//simulating the trajectory
			tra = Rover_vw(V_in, Omega_in, b, Ts,x_0,x_dot_0 , sample,x_dot_f);
			traj_to_cloud(tra);
			
		ROS_ERROR_STREAM("tra size  " << tra.cols() <<"  path trace size   "<< path_trace_pcl.points.size());
			
		//ROS_INFO_STREAM_ONCE("trajectory is " << tra);
		
			
			Vector3f tra_tail;
			tra_tail(0) = tra(0,tra.cols()-1);
			tra_tail(1) = tra(1,tra.cols()-1);
			tra_tail(2) = 0;
		ROS_WARN_STREAM_ONCE("tra length  " << tra.cols() << "   tra_tail :  " << tra_tail);
			
			
			//Calculating the cost of trajectory
			PATH_COST cost = Cost_of_path(tra, master_grid_);
			
		ROS_INFO_ONCE("cost of the path is %f",cost.Lethal_cost);
			
			float prop_speed = (fabs(x(0,0))+fabs(x(0,1))+fabs(x(0,2)))/3;
			
			//Defining the objective function
			float Ob_func = Goal_gain * sqrtf( pow((tra_tail(0)-Goal(0)), 2) + pow((tra_tail(1)-Goal(1)), 2) )    //effect of distance from the goal
				      + Cost_gain * (cost.Lethal_cost + cost.Inf_cost)				     	      //path cost
				      + Speed_gain * (V_curr_c(0) - prop_speed);			      		      //speed effect
		
		ROS_INFO("LETHAL COST: %f",cost.Lethal_cost);		      
		ROS_INFO("Ob_fun: %f",Ob_func);
				      
			if (Ob_func < x_best_cost)
			{
				x_best_cost = Ob_func;
				x_best(0) = x(0,i);
				x_best(1) = x(1,i);
				ROS_INFO("new value for x_best_cost");	
			}
			if (Ob_func < G_cost)
			{
				G_cost = Ob_func;
				G(0) = x(0,i);
				G(1) = x(1,i);
				output_tra = tra;
				if (cost.Lethal_cost < 1) solution_found = true;
				ROS_WARN(" ------>  new value for G_cost");
			}
			if(i==0) //Reseting X_best and its cost in each iteration
			{
				x_best(0) = x(0,i);
				x_best(1) = x(1,i);
				x_best_cost = Ob_func;
			}	  
			v(1,i) = pso_inertia * v(1,i) + c_1 * r_1 * (x_best(1) - x(1,i)) + c_2 * r_2 * (G(1) - x(1,i));
			
			
		//ROS_WARN("Speed terms");
		//std::cout << v << "\n";	
		/*	
		ROS_WARN("Speed terms");
		ROS_INFO("1st:");
		std::cout << pso_inertia *v << "\n";
		
		ROS_INFO("2nd:");
		std::cout << c_1 * r_1 * (x_best - x) << "\n";
		
		ROS_INFO("3rd:");
		std::cout << c_2 * r_2 * (G - x) << "\n";
		
		ROS_ERROR("v:      x_best - x:  G - x");
		std::cout << v << "\n";
		ROS_ERROR("////////");
		std::cout << (x_best - x) << "\n";
		ROS_ERROR("////////");
		std::cout << (G - x) << "\n";
		*/	
				 
		

	



		}
		x = x+v;
		// Publishing
			
		robot_opt_path.header.stamp = ros::Time::now();
			
 		robot_opt_path.header.frame_id = "laser";
   
   		
		robot_opt_path.poses = std::vector<geometry_msgs::PoseStamped> (sample);
		for(size_t i=0; i < sample; i++)
		{

			robot_opt_path.poses[i].pose.position.x = tra(0,i);
			robot_opt_path.poses[i].pose.position.y = tra(1,i);
			robot_opt_path.poses[i].pose.position.z = 0.0;
			
		}
	  	
	  	path_solution_pub_.publish(robot_opt_path);
	  	//ROS_WARN("V0:%f   V1:%f   V2:%f", G(0,0), G(0,1), G(0,2));
		//ROS_WARN("w0:%f   w1:%f   w2:%f", G(1,0), G(1,1), G(1,2));
		
		// end pub	
	}
    
	//output = G;
    
	}
};
