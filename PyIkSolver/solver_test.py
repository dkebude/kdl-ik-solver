#!/usr/bin/env python
import rospy
from solver import *

def main():
	verbose = False

	solver = kdlSolver("/home/dkebude/catkin_ws/src/panda_sim/panda_sim_description/urdf/panda_arm_hand.urdf","panda_link0","panda_hand", 100000, 125, 0.000001)

	num_joints = solver.getNumJoints()

	q_targ = DoubleVector(num_joints)

	q_init = DoubleVector(num_joints)
	q_init[0] = 1.15636
	q_init[1] = 1.16243
	q_init[2] = -0.61218
	q_init[3] = -1.85039
	q_init[4] = -2.88407
	q_init[5] = 2.57014
	q_init[6] = -2.53572

	p_targ = DoubleVector(7)
	p_targ[0] = 0.485
	p_targ[1] = 0.237
	p_targ[2] = -0.028
	p_targ[3] = 0.87
	p_targ[4] = 0.117
	p_targ[5] = 0.473
	p_targ[6] = -0.076
	normalizeQuaternion(p_targ)

	print "\n------------------------\nTrying with pose IK Solver:\n------------------------\n"
	print "\nInitial joint positions:\n------------------------\n"
	printDoubleVec(q_init)

	print "\nTarget frame:\n------------------------\n"
	printDoubleVec(p_targ)

	print "\nJoints as calculated by IK:\n------------------------\n"
	solver.solvePoseIk(q_init, q_targ, p_targ, verbose)
	printDoubleVec(q_targ)

	print "\nFrame as calculated after IK:\n------------------------\n"
	solver.solveFkDouble(q_targ, p_targ)
	printDoubleVec(p_targ)

	print "\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n"%e_dist(q_init, q_targ)

	p_targ[0] = 0.485
	p_targ[1] = 0.237
	p_targ[2] = -0.028
	p_targ[3] = 0.87
	p_targ[4] = 0.117
	p_targ[5] = 0.473
	p_targ[6] = -0.076
	normalizeQuaternion(p_targ)

	print "\n------------------------\nTrying with position only IK Solver:\n------------------------\n"
	print "\nInitial joint positions:\n------------------------\n"
	printDoubleVec(q_init)

	print "\nTarget frame:\n------------------------\n"
	printDoubleVec(p_targ)

	print "\nJoints as calculated by IK:\n------------------------\n"
	solver.solvePosOnlyIk(q_init, q_targ, p_targ, verbose)
	printDoubleVec(q_targ)

	print "\nFrame as calculated after IK:\n------------------------\n"
	solver.solveFkDouble(q_targ, p_targ)
	printDoubleVec(p_targ)

	print "\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n"%e_dist(q_init, q_targ)

	p_targ[0] = 0.485
	p_targ[1] = 0.237
	p_targ[2] = -0.028
	p_targ[3] = 0.87
	p_targ[4] = 0.117
	p_targ[5] = 0.473
	p_targ[6] = -0.076
	normalizeQuaternion(p_targ)

	jnt_dist_th = 1.5
	verbose = False

	print "\n------------------------\nTrying with hybrid IK Solver:\n------------------------\n"
	print "\nInitial joint positions:\n------------------------\n"
	printDoubleVec(q_init)

	print "\nTarget frame:\n------------------------\n"
	printDoubleVec(p_targ)

	print "\nJoints as calculated by IK:\n------------------------\n"
	solver.solveHybridIk(q_init, q_targ, p_targ, jnt_dist_th, verbose)
	printDoubleVec(q_targ)

	print "\nFrame as calculated after IK:\n------------------------\n"
	solver.solveFkDouble(q_targ, p_targ)
	printDoubleVec(p_targ)

	print "\n------------------------\nEuclidean distance between initial and target angles: %.5f\n------------------------\n"%e_dist(q_init, q_targ)

if __name__ == '__main__':
	try:
	    main()
	except rospy.ROSInterruptException:
	    pass