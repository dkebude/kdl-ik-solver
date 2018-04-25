#include <kdl_parser/kdl_parser.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <eigen3/Eigen/Core>

#include <urdf/model.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

# define PI 3.14159265358979323846 

bool fromFile(KDL::Tree & my_tree, std::string filename);
void doubleVecToJntArray(std::vector<double> vec, KDL::JntArray &jnt_arr);
void doubleVecToFrame(std::vector<double> vec, KDL::Frame &frame, bool isQuat);
void jntArrayToDoubleVec(KDL::JntArray jnt_arr, std::vector<double> &vec);
void frameToDoubleVec(KDL::Frame frame, std::vector<double> &vec, bool isQuat);
void printDoubleVec(std::vector<double> vec);
void printJntArray(KDL::JntArray j);
double e_dist(std::vector<double> a, std::vector<double> b);
void vecNormalRandom(std::vector<double> &q);
void normalizeQuaternion(std::vector<double> &q);

class kdlSolver
{
	KDL::Tree tree;
	KDL::Chain chain;
	KDL::ChainFkSolverPos_recursive *fk_solver;
	KDL::ChainIkSolverVel_wdls *ik_solver;
	KDL::JntArray jnt_min, jnt_max;

	KDL::ChainJntToJacSolver *jnt2jac;
	KDL::Jacobian  jac;
	Eigen::MatrixXd jacTop;
	Eigen::MatrixXd U;
	Eigen::VectorXd S;
	Eigen::MatrixXd V;
	Eigen::VectorXd tmp;
	
	unsigned int svd_iter;
	unsigned int max_iter;
	unsigned int nj;
	unsigned int ns;
	double eps;

public:
	kdlSolver();
	~kdlSolver();
	kdlSolver(std::string filename, std::string root, std::string tip, unsigned int _max_iter = 10000, unsigned int _svd_iter = 125, double _eps = 1e-6);
	void getJointLimits(std::string filename, KDL::JntArray &jnt_min, KDL::JntArray &jnt_max);	
	int getNumJoints();
	void getJointLimits(KDL::JntArray & j_min, KDL::JntArray & j_max);
	void randInit(KDL::JntArray &randJnt);
	void solveFkDouble(std::vector<double> &q_in, std::vector<double> &p_out);
	void solveFkJntArray(KDL::JntArray &q_in, KDL::Frame &p_out);
	bool solvePoseIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose);
	bool solvePosOnlyIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, bool verbose);
	void solveHybridIk(std::vector<double> &q_init, std::vector<double> &q_targ, std::vector<double> &p_targ, double threshold, bool verbose);
	bool clamp(std::vector<double> &q_targ);
	int myCartToJntVel(const KDL::JntArray& q_init_jnt_arr, const KDL::Twist& p_delta, KDL::JntArray& q_delta);
	bool reject(std::vector<double> q_init, std::vector<double> q_targ, double threshold);
};