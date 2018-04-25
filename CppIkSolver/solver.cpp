#include "solver.h"

kdlSolver::kdlSolver()
{
	kdlSolver("/home/dkebude/catkin_ws/src/panda_sim/panda_sim_description/urdf/panda_arm_hand.urdf","panda_link0","panda_hand");
}

kdlSolver::~kdlSolver()
{
	delete fk_solver;
	delete ik_solver;
}

kdlSolver::kdlSolver(std::string filename, std::string root, std::string tip, unsigned int _max_iter, unsigned int _svd_iter, double _eps)
{
	max_iter = _max_iter;
	eps = _eps;

	fromFile(tree, filename);
	tree.getChain(root, tip, chain);
	nj = chain.getNrOfJoints();
	ns = chain.getNrOfSegments();
	jnt_min.resize(nj);
		jnt_max.resize(nj);

		printf("\nKinematic chain acquired!\nChain base: %s\nChain tip: %s\nNumber of joints in the chain: %d\n", root.c_str(), tip.c_str(), nj);

		getJointLimits(filename, jnt_min, jnt_max);
	
	fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
	ik_solver = new KDL::ChainIkSolverVel_wdls(chain, eps, max_iter);

	jnt2jac = new KDL::ChainJntToJacSolver(chain);
	jac = KDL::Jacobian(nj);
	U = Eigen::MatrixXd::Zero(3,nj);
	S = Eigen::VectorXd::Zero(nj);
	V = Eigen::MatrixXd::Zero(nj,nj);
	tmp = Eigen::VectorXd::Zero(nj);
	svd_iter = _svd_iter;
}

void kdlSolver::getJointLimits(std::string filename, KDL::JntArray &jnt_min, KDL::JntArray &jnt_max)
{
	urdf::Model robotModel;
		robotModel.initFile(filename);

		boost::shared_ptr<const urdf::Joint> joint;
		for(unsigned int joint_num = 0, segment_num=0; joint_num < nj && segment_num < ns; segment_num++)
		{
		std::string jointName = chain.getSegment(segment_num).getJoint().getName().c_str();
		joint = robotModel.getJoint(jointName);
		   	if(joint && joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
		{
    		double lower=0,upper=0;
    		if(joint->type != urdf::Joint::CONTINUOUS)
    		{
        		lower=joint->limits->lower;
        		upper=joint->limits->upper;
        		std::cout << "Loaded joint limits for " << jointName << "(" << joint_num<< ")" << ": " << lower << " to " << upper  << std::endl;
    		}
    		else
    		{
        		lower=-3.14;
        		upper= 3.14;
    		}
    		jnt_min(joint_num)=lower;
    		jnt_max(joint_num)=upper;
    		joint_num++;
		}
		else
		{
    		std::cout << "Didn't load joint limits for " << jointName << std::endl;
		}
	}
}

int kdlSolver::getNumJoints()
{
	return nj;
}

void kdlSolver::getJointLimits(KDL::JntArray & j_min, KDL::JntArray & j_max)
{
	j_min = jnt_min;
	j_max = jnt_max;
}

void kdlSolver::randInit(KDL::JntArray &randJnt)
{
	for(int i = 0; i < nj; i++)
	{
		double f = (double)std::rand()/RAND_MAX;
			randJnt(i) = jnt_min(i) + f*(jnt_max(i)-jnt_min(i));
	}
}

void kdlSolver::solveFkDouble(std::vector<double> &q_in, std::vector<double> &p_out)
{
	KDL::JntArray q_in_jnt_array(nj);
	KDL::Frame p_out_frame;
	doubleVecToJntArray(q_in, q_in_jnt_array);
	int status = fk_solver->JntToCart(q_in_jnt_array, p_out_frame);
	if(status >= 0)
	{
		if(p_out.size() == 12)
			frameToDoubleVec(p_out_frame, p_out, false);
		else if(p_out.size() == 7)
			frameToDoubleVec(p_out_frame, p_out, true);
	}
	else
	{
		printf("Something went wrong while calculating FK.\n");
	}
}

void kdlSolver::solveFkJntArray(KDL::JntArray &q_in, KDL::Frame &p_out)
{
	int status = fk_solver->JntToCart(q_in, p_out);
	if(status < 0)
	{
		printf("Something went wrong while calculating FK.\n");
	}
}

bool kdlSolver::solvePoseIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose)
{
	KDL::JntArray q_init_jnt_arr(nj);
	KDL::JntArray q_targ_jnt_arr(nj);
	KDL::JntArray q_delta(nj);
	KDL::Frame p_init_frame;
	KDL::Frame p_targ_frame;
	KDL::Twist p_delta;

	doubleVecToJntArray(q_init, q_init_jnt_arr);
	if(p_targ.size() == 7)
		doubleVecToFrame(p_targ, p_targ_frame, true);
	else if(p_targ.size() == 12)
		doubleVecToFrame(p_targ, p_targ_frame, false);

	double k2 = 0.01;
	bool found = false;

	int trialCnt = 0;
	while(!found && trialCnt < 100)
	{
		q_targ_jnt_arr = q_init_jnt_arr;
		for(int iter = 0; iter < max_iter; iter++)
		{
			solveFkJntArray(q_targ_jnt_arr, p_init_frame); 			// Get initial pose of EE
			
			p_delta = diff(p_init_frame, p_targ_frame);				// Calculate the twist between init and final
			
			ik_solver->CartToJnt(q_init_jnt_arr, p_delta, q_delta);	// Calculate the joint angle change rate
			Multiply(q_delta, k2, q_delta);
			Add(q_targ_jnt_arr, q_delta, q_targ_jnt_arr);

			if(Equal(p_delta,KDL::Twist::Zero(),eps))
			{
				jntArrayToDoubleVec(q_targ_jnt_arr, q_targ);
				found = clamp(q_targ);
				if(verbose && found)
				{
					printf("IK solution found at iteration %d.\n", iter);
					break;
				}
				else if(found)
					break;
			}
			else if(iter == max_iter-1 && verbose)
			{
				printf("Maximum iteration count reached. ");
			}
		}

		if(!found)
    	{
    		std::vector<double> q(nj);
    		jntArrayToDoubleVec(q_init_jnt_arr, q);
    		vecNormalRandom(q);
    		doubleVecToJntArray(q, q_init_jnt_arr);
    		trialCnt++;
    		if(verbose)
    		{
    			printf("Trial %d finished. Trying with new initial joint positions:\n", trialCnt);
    			printJntArray(q_init_jnt_arr);
    		}
    	}		    	
	}
	if(!found && verbose)
		printf("Could not find a valid IK solution.\n");

	return found;
}

bool kdlSolver::solvePosOnlyIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose)
{
	KDL::JntArray q_init_jnt_arr(nj);
	KDL::JntArray q_targ_jnt_arr(nj);
	KDL::JntArray q_delta(nj);
	KDL::Frame p_init_frame;
	KDL::Frame p_targ_frame;
	KDL::Twist p_delta;

	doubleVecToJntArray(q_init, q_init_jnt_arr);
	if(p_targ.size() == 7)
		doubleVecToFrame(p_targ, p_targ_frame, true);
	else if(p_targ.size() == 12)
		doubleVecToFrame(p_targ, p_targ_frame, false);

	double k2 = 0.01;
	bool found = false;

	int trialCnt = 0;
	while(!found && trialCnt < 100)
	{
		q_targ_jnt_arr = q_init_jnt_arr;
		for(int iter = 0; iter < max_iter; iter++)
		{
			solveFkJntArray(q_targ_jnt_arr, p_init_frame); 			// Get initial pose of EE
			
			p_init_frame.M = p_targ_frame.M;

			p_delta = diff(p_init_frame, p_targ_frame);				// Calculate the twist between init and final
			
			myCartToJntVel(q_init_jnt_arr, p_delta, q_delta);		// Calculate the joint angle change rate
			Multiply(q_delta, k2, q_delta);
			Add(q_targ_jnt_arr, q_delta, q_targ_jnt_arr);

			if(Equal(p_delta,KDL::Twist::Zero(),eps))
			{
				jntArrayToDoubleVec(q_targ_jnt_arr, q_targ);
				found = clamp(q_targ);
				if(verbose && found)
				{
					printf("IK solution found at iteration %d.\n", iter);
					break;
				}
				else if(found)
					break;
			}
			else if(iter == max_iter-1 && verbose)
			{
				printf("Maximum iteration count reached. ");
			}
		}

		if(found)
		{
			jntArrayToDoubleVec(q_targ_jnt_arr, q_targ);
			found = clamp(q_targ);
			if(!found && verbose)
				printf("However, something went wrong with IK calculation. Trying with some other random joint angle set.");
		}
    	if(!found)
    	{
    		std::vector<double> q(nj);
    		jntArrayToDoubleVec(q_init_jnt_arr, q);
    		vecNormalRandom(q);
    		doubleVecToJntArray(q, q_init_jnt_arr);
    		trialCnt++;
    		if(verbose)
    		{
    			printf("Trial %d finished. Trying with new initial joint positions:\n", trialCnt);
    			printJntArray(q_init_jnt_arr);
    		}
    	}		    	
	}
	if(!found && verbose)
		printf("Could not find a valid IK solution.\n");

	return found;
}

void kdlSolver::solveHybridIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, double threshold, bool verbose)
{
	bool found = false;
	bool found_pos_only = false;
	bool rejected = true;
	int trialWithPose = 0;
	int trialWithPosOnly = 0;
	printf("Hybrid solver, trying to solve with pose.");
	fflush(stdout);
	while(rejected && trialWithPose < 10)
	{
		found = solvePoseIk(q_init, q_targ, p_targ, false);
		if(verbose)
		{
			printf("\n------------------------\nFound a solution! Euclidean distance between initial and target angles: %.5f\n------------------------\n", e_dist(q_init, q_targ));
		}
		else
		{
			printf(".");
			fflush(stdout);
		}

		rejected = reject(q_init, q_targ, threshold);
		trialWithPose++;
	}
	if(!found)
	{
		printf("\nHybrid solver, trying to solve with position only.\n");
		fflush(stdout);
	}
	else
	{
		printf("\nFound a solution, quitting the loop!\n");
	}
	while(rejected && trialWithPosOnly < 10)
	{
		found_pos_only = solvePosOnlyIk(q_init, q_targ, p_targ, false);
		if(verbose)
		{
			printf("\n------------------------\nFound a solution! Euclidean distance between initial and target angles: %.5f\n------------------------\n", e_dist(q_init, q_targ));
		}
		else
		{
			printf(".");
			fflush(stdout);
		}

		rejected = reject(q_init, q_targ, threshold);
		trialWithPosOnly++;
	}
	if(rejected)
		printf("\nFound a solution with position only but over the threshold, be careful when using!\n");
	else if(found_pos_only)
		printf("\nFound a solution with position only IK!\n");	
}

bool kdlSolver::clamp(std::vector<double> &q_targ)
{
	bool found = true;
	for(int i = 0; i < q_targ.size(); i++)
	{
		if(jnt_min(i) <= std::fmod(q_targ[i],2*PI) && std::fmod(q_targ[i],2*PI) <= jnt_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI);
		else if(jnt_min(i) <= std::fmod(q_targ[i],2*PI)-2*PI && std::fmod(q_targ[i],2*PI)-2*PI <= jnt_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)-2*PI;
		else if(jnt_min(i) <= std::fmod(q_targ[i],2*PI)+2*PI && std::fmod(q_targ[i],2*PI)+2*PI <= jnt_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)+2*PI;
		else
		{
			if(q_targ[i] < jnt_min(i))
			{
				q_targ[i] = jnt_min(i);
			}
			else
			{
				q_targ[i] = jnt_max(i);
			}
			found = false;
		}
	}
	return found;		
}

int kdlSolver::myCartToJntVel(const KDL::JntArray& q_init_jnt_arr, const KDL::Twist& p_delta, KDL::JntArray& q_delta)
{

	jnt2jac->JntToJac(q_init_jnt_arr,jac);
        
	jacTop = jac.data.topRows(3);
	
	double sum;
	unsigned int i,j;
	
	// Compute the SVD of the weighted jacobian
	int ret = KDL::svd_eigen_HH(jacTop,U,S,V,tmp,svd_iter);
	        
	// We have to calculate qdot_out = jac_pinv*v_in
	// Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
	// qdot_out = V*S_pinv*Ut*v_in
			
	for (i=0;i<jacTop.cols();i++) {
	    sum = 0.0;
	    for (j=0;j<jacTop.rows();j++) {
	         if(i<3)
	             sum+= U(j,i)*p_delta(j);
	         else
	             sum+=0.0;
	     }
	
	     tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
	 }
	
	//tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
	//it with V to get qdot_out
	for (i=0;i<jacTop.cols();i++) {
	    sum = 0.0;
	    for (j=0;j<jacTop.cols();j++) {
	        sum+=V(i,j)*tmp(j);
	    }
	    //Put the result in qdot_out
	    q_delta(i)=sum;
	}
	
	return ret;  
}

bool kdlSolver::reject(std::vector<double> q_init, std::vector<double> q_targ, double threshold)
{
	if(e_dist(q_init, q_targ) > threshold)
		return true;
	else
		return false;
}

bool fromFile(KDL::Tree & my_tree, std::string filename)
{
	if(!kdl_parser::treeFromFile(filename, my_tree))
	{
		printf("Failed to construct kdl tree!\nNote: File must be *.urdf\n");
		return false;
	}
	return true;
}

void doubleVecToJntArray(std::vector<double> vec, KDL::JntArray &jnt_arr)
{
	for(int i = 0; i < vec.size(); i++)
	{
		jnt_arr.data(i) = vec[i];
	}
}

void doubleVecToFrame(std::vector<double> vec, KDL::Frame &frame, bool isQuat)
{
	for(int i = 0; i < 3; i++)
   		frame.p.data[i] = vec[i];

	if(isQuat)
	{
		frame.M = KDL::Rotation::Quaternion(vec[3],vec[4],vec[5],vec[6]);
	}
	else
	{
		for(int i = 0; i < 9; i++)
 			frame.M.data[i] = vec[i+3];
	}
}

void jntArrayToDoubleVec(KDL::JntArray jnt_arr, std::vector<double> &vec)
{
	int len = jnt_arr.data.size();
	for(int i = 0; i < len; i++)
	{
		vec[i] = jnt_arr.data(i);
	}
}

void frameToDoubleVec(KDL::Frame frame, std::vector<double> &vec, bool isQuat)
{

	for(int i = 0; i < 3; i++)
		vec[i] = frame.p.data[i];

	if(isQuat)
	{
		frame.M.GetQuaternion(vec[3], vec[4], vec[5], vec[6]);
	}
	else
	{
		for(int i = 0; i < 9; i++)
      		vec[i+3] = frame.M.data[i];
	}
}

void printDoubleVec(std::vector<double> vec)
{
	int len = vec.size();
	printf("[");
	for(int i = 0; i < len; i++)
		printf(" %.5f ", vec[i]);
	printf("]\n");
}

void printJntArray(KDL::JntArray j)
{
	std::vector<double> print_vec(j.data.size());
	jntArrayToDoubleVec(j, print_vec);
	printDoubleVec(print_vec);
}

double e_dist(std::vector<double> a, std::vector<double> b)
{
	double dist = 0.0;

	for(int i = 0; i < a.size(); i++)
		dist += std::sqrt(std::pow(a[i]-b[i],2));
	
	return dist;
}

void vecNormalRandom(std::vector<double> &q)
{
	for(int i = 0; i < q.size(); i++)
	{
		boost::mt19937 *rng = new boost::mt19937();
		rng->seed(time(NULL));
		
		boost::normal_distribution<> distribution(q[i], 0.001);
		boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dist(*rng, distribution);

		double new_angle = dist();
		q[i] = new_angle;
	}
}

void normalizeQuaternion(std::vector<double> &q)
{
	double denom = std::sqrt(std::pow(q[3],2)+std::pow(q[4],2)+std::pow(q[5],2)+std::pow(q[6],2));
	q[3] = q[3]/denom;
	q[4] = q[4]/denom;
	q[5] = q[5]/denom;
	q[6] = q[6]/denom;
}