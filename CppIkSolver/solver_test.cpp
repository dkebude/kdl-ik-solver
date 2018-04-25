#include "solver.h"

double frand(double min, double max)
{
  double f = (double)std::rand()/RAND_MAX;
  return min + f*(max-min);
}

int main(int argc, char** argv)
{
	bool verbose = true;

	int seed = std::time(0);
	std::srand(seed);
	printf("IK Solver test starting with random seed %d\n", seed);

	kdlSolver solver("/home/dkebude/catkin_ws/src/panda_sim/panda_sim_description/urdf/panda_arm_hand.urdf","panda_link0","panda_hand", 100000, 125, 1e-6);

	int nj = solver.getNumJoints();

	KDL::JntArray j_min(nj);
	KDL::JntArray j_max(nj);
	solver.getJointLimits(j_min, j_max);

	std::vector<double> q_init (nj);
	std::vector<double> q_targ (nj);
	std::vector<double> p_targ (7); //quaternion

	q_init.at(0) = 1.15636;//(j_min(0)+j_max(0))/2;
	q_init.at(1) = 1.16243;//(j_min(1)+j_max(1))/2;
	q_init.at(2) = -0.61218;//(j_min(2)+j_max(2))/2;
	q_init.at(3) = -1.85039;//(j_min(3)+j_max(3))/2;
	q_init.at(4) = -2.88407;//(j_min(4)+j_max(4))/2;
	q_init.at(5) = 2.57014;//(j_min(5)+j_max(5))/2;
	q_init.at(6) = -2.53572;//(j_min(6)+j_max(6))/2;
	
	p_targ.at(0) = 0.485;//frand(j_min(0), j_max(0));
	p_targ.at(1) = 0.237;//frand(j_min(1), j_max(1));
	p_targ.at(2) = -0.028;//frand(j_min(2), j_max(2));
	p_targ.at(3) = 0.87;//frand(j_min(3), j_max(3));
	p_targ.at(4) = 0.117;//frand(j_min(4), j_max(4));
	p_targ.at(5) = 0.473;//frand(j_min(5), j_max(5));
	p_targ.at(6) = -0.076;//frand(j_min(6), j_max(6));
	normalizeQuaternion(p_targ);
	
	printf("\n------------------------\nTrying with pose IK Solver:\n------------------------\n");

	printf("\nInitial joint positions:\n------------------------\n");
	printDoubleVec(q_init);

	printf("\nTarget frame:\n------------------------\n");
	printDoubleVec(p_targ);

	printf("\nJoints as calculated by IK:\n------------------------\n");
	solver.solvePoseIk(q_init, q_targ, p_targ, verbose);
	printDoubleVec(q_targ);

	printf("\nFrame as calculated after IK:\n------------------------\n");
	solver.solveFkDouble(q_targ, p_targ);
	printDoubleVec(p_targ);

	printf("\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n", e_dist(q_init, q_targ));

	p_targ.at(0) = 0.485;//frand(j_min(0), j_max(0));
	p_targ.at(1) = 0.237;//frand(j_min(1), j_max(1));
	p_targ.at(2) = -0.028;//frand(j_min(2), j_max(2));
	p_targ.at(3) = 0.87;//frand(j_min(3), j_max(3));
	p_targ.at(4) = 0.117;//frand(j_min(4), j_max(4));
	p_targ.at(5) = 0.473;//frand(j_min(5), j_max(5));
	p_targ.at(6) = -0.076;//frand(j_min(6), j_max(6));
	normalizeQuaternion(p_targ);

	printf("\n------------------------\nTrying with position only IK Solver:\n------------------------\n");

	printf("\nInitial joint positions:\n------------------------\n");
	printDoubleVec(q_init);

	printf("\nTarget frame:\n------------------------\n");
	printDoubleVec(p_targ);

	printf("\nJoints as calculated by IK:\n------------------------\n");
	solver.solvePosOnlyIk(q_init, q_targ, p_targ, verbose);
	printDoubleVec(q_targ);

	printf("\nFrame as calculated after IK:\n------------------------\n");
	solver.solveFkDouble(q_targ, p_targ);
	printDoubleVec(p_targ);

	printf("\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n", e_dist(q_init, q_targ));

	p_targ.at(0) = 0.485;//frand(j_min(0), j_max(0));
	p_targ.at(1) = 0.237;//frand(j_min(1), j_max(1));
	p_targ.at(2) = -0.028;//frand(j_min(2), j_max(2));
	p_targ.at(3) = 0.87;//frand(j_min(3), j_max(3));
	p_targ.at(4) = 0.117;//frand(j_min(4), j_max(4));
	p_targ.at(5) = 0.473;//frand(j_min(5), j_max(5));
	p_targ.at(6) = -0.076;//frand(j_min(6), j_max(6));
	normalizeQuaternion(p_targ);

	printf("\n------------------------\nTrying with hybrid IK Solver:\n------------------------\n");

	double jnt_dist_threshold = 1.5;
	verbose = false;

	printf("\nInitial joint positions:\n------------------------\n");
	printDoubleVec(q_init);

	printf("\nTarget frame:\n------------------------\n");
	printDoubleVec(p_targ);

	printf("\nJoints as calculated by IK:\n------------------------\n");
	solver.solveHybridIk(q_init, q_targ, p_targ, jnt_dist_threshold, verbose);
	printDoubleVec(q_targ);

	printf("\nFrame as calculated after IK:\n------------------------\n");
	solver.solveFkDouble(q_targ, p_targ);
	printDoubleVec(p_targ);

	printf("\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n", e_dist(q_init, q_targ));

	return 0;
}