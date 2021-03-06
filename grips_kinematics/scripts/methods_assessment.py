#!/usr/bin/env python
#
# Poses are stored as:
# rotation     position
# [w x y z]    [x y z]

import roslib
import rospy, math, argparse, os
import random
import scipy.io as sio
import numpy as np
# Services
from grips_msgs.srv import GetStateMetrics, GetStateMetricsRequest, GetStateMetricsResponse
from grips_msgs.srv import GetJointLimits, GetJointLimitsRequest, GetJointLimitsResponse
from grips_msgs.srv import GetPoseMetrics, GetPoseMetricsRequest, GetPoseMetricsResponse
# Messages
from geometry_msgs.msg import Pose
#Dynamic Reconfigure
import dynamic_reconfigure.client

in_variables = ['joint_positions', 'poses']

out_variables = ['joint_positions', 'poses', 'error', 'iterations', 'time']

joint_names = ['SA', 'SE', 'linkage_tr', 'WP', 'WY', 'WR']
NUM_JOINTS = len(joint_names)

def generate_input_data(input_mat_file, NUM_TEST = 100):
  # Subscribe to the FK service
  fk_srv_name = rospy.get_param('metrics_service', '/grips/kinematic_services/get_fk_metrics')
  rospy.loginfo('Waiting for %s service' % fk_srv_name)
  rospy.wait_for_service(fk_srv_name)
  fk_srv = rospy.ServiceProxy(fk_srv_name, GetStateMetrics)
  # Subscribe to limits_service
  limits_srv_name = rospy.get_param('limits_service', '/grips/kinematic_services/get_joint_limits')
  rospy.loginfo('Waiting for %s service' % limits_srv_name)
  rospy.wait_for_service(limits_srv_name)
  limits_srv = rospy.ServiceProxy(limits_srv_name, GetJointLimits)
  # Get the joint limits
  req = GetJointLimitsRequest()
  req.header.stamp = rospy.Time.now()
  req.header.frame_id = '/world'
  req.name = joint_names
  try:
    res = limits_srv(req)
  except rospy.ServiceException, e:
    rospy.logwarn('Service did not process request: %s' % str(e))     
  min_positions = np.array(res.min_position)
  max_positions = np.array(res.max_position)
  rospy.loginfo('min_positions: %s' % str(min_positions))
  rospy.loginfo('max_positions: %s' % str(max_positions))
  rospy.loginfo('Generating test MAT file, [%d] tests' % NUM_TEST)
  angles_mat = np.array([]) 
  poses_mat = np.array([])
  req = GetStateMetricsRequest()
  for i in xrange(NUM_TEST):
    random_joints = []
    current_pose = [0]*7
    for joint in xrange(NUM_JOINTS):
      random_joints.append(random.uniform(min_positions[joint], max_positions[joint]))
    req.joint_states.header.stamp = rospy.Time.now()
    req.joint_states.header.frame_id = 'world'
    req.joint_states.name = joint_names
    req.joint_states.position = list(random_joints)
    try:
      res = fk_srv(req)
      if res.found_group:
        current_pose = pose2list(res.pose)
    except rospy.ServiceException, e:
      rospy.logwarn('Service did not process request: %s' % str(e))
    angles_mat = np.append(angles_mat, np.array(random_joints), 0)
    poses_mat = np.append(poses_mat, np.array(current_pose), 0)
    if (i % 1000 == 0):
      rospy.loginfo('[FK] Evaluated: %d/%d' % (i, NUM_TEST))
    # Check for shutdowns
    if rospy.is_shutdown():
      return
  # Prepares the data to save it in a .mat file
  angles_shape = (NUM_TEST, NUM_JOINTS)
  poses_shape = (NUM_TEST, 7)
  angles_mat = angles_mat.reshape(angles_shape)
  poses_mat = poses_mat.reshape(poses_shape)
  rospy.loginfo('[FK] Evaluated: %d/%d' % (i+1, NUM_TEST))
  rospy.loginfo('Writing %s file' % input_mat_file)
  sio.savemat(input_mat_file, {in_variables[0]:angles_mat, in_variables[1]:poses_mat}, oned_as='column')
  rospy.sleep(3.0)

def solve_ik(input_mat_file, out_filename):  
  # Subscribe to kinematic services
  ik_srv_name = rospy.get_param('metrics_service', '/grips/kinematic_services/get_ik_metrics')
  rospy.loginfo('Waiting for %s service' % ik_srv_name)
  rospy.wait_for_service(ik_srv_name)
  ik_srv = rospy.ServiceProxy(ik_srv_name, GetPoseMetrics)
  # Load data from the *.mat file  
  input_dict = sio.loadmat(input_mat_file)
  poses_mat = input_dict[in_variables[1]]
  estimated_poses = np.array([])
  calculated_mat = np.array([])
  estimation_error = np.array([])
  iterations = []
  time_list = []
  calculated = 0  
  req = GetPoseMetricsRequest()
  tests = poses_mat.shape[0]
  for i, pose in enumerate(poses_mat):
    req.header.stamp = rospy.Time.now()
    req.header.frame_id = 'world'
    req.link_name = 'end_effector'
    req.pose = list2pose(pose)
    try:
      res = ik_srv(req)
      time_list.append((res.duration).to_sec())
      if res.found_ik and res.found_group:
        calculated_mat = np.append(calculated_mat, np.array(res.joint_states.position), 0)
        estimated_poses = np.append(estimated_poses, np.array(pose2list(res.estimated_pose)), 0)
        estimation_error = np.append(estimation_error, np.array(pose2list(res.estimation_error)), 0)
        iterations.append(1)
        calculated += 1
      else:
        calculated_mat = np.append(calculated_mat, np.array([0]*NUM_JOINTS), 0)
        estimated_poses = np.append(estimated_poses, np.array([0]*7), 0)
        estimation_error = np.append(estimation_error, np.array([0]*7), 0)
        iterations.append(0)
    except rospy.ServiceException, e:
      rospy.logwarn('Service did not process request: %s' % str(e))
    if (i % 1000 == 0):
      rospy.loginfo('[IK] Evaluated: %d/%d' % (i, tests))
    # Check for shutdowns
    if rospy.is_shutdown():
      return
  # Show the result in the console
  rospy.loginfo('IK Done: %d/%d metrics were calculated' % (calculated, tests))
  # Prepare the data to save it in a .mat file
  calculated_mat = calculated_mat.reshape((tests, NUM_JOINTS))
  estimated_poses = estimated_poses.reshape((tests, 7))
  estimation_error = estimation_error.reshape((tests, 7))
  variables = [0] * len(out_variables)
  for i, name in enumerate(out_variables):
    variables[i] = out_filename + '_' + name
  data_dict = {variables[0]: calculated_mat, variables[1]:estimated_poses, variables[2]:estimation_error, variables[3]:iterations, variables[4]:time_list}
  # Save new .mat file
  path = os.path.dirname(input_mat_file)
  out_mat_file = out_filename + '.mat'
  if path:
    out_mat_file = path + '/' + out_mat_file
  rospy.loginfo(('\033[94m' + 'Writing %s file' + '\033[0m') % out_mat_file)
  sio.savemat(out_mat_file, data_dict, oned_as='column')

def pose2list(pose):
  data = [0] * 7
  data[0] = pose.orientation.w
  data[1] = pose.orientation.x
  data[2] = pose.orientation.y
  data[3] = pose.orientation.z
  data[4] = pose.position.x
  data[5] = pose.position.y
  data[6] = pose.position.z
  return data

def list2pose(data):
  pose = Pose()
  pose.orientation.w = data[0]
  pose.orientation.x = data[1]
  pose.orientation.y = data[2]
  pose.orientation.z = data[3]
  pose.position.x = data[4]
  pose.position.y = data[5]
  pose.position.z = data[6]
  return pose


if __name__ == '__main__':
  rospy.init_node('methods_assessment')
  # Parse the arguments
  parser = argparse.ArgumentParser(description='Performs the assessment of the current IK solver')
  parser.add_argument('--generate', action='store_true', 
                        help='If set, generates a new input MAT file')
  parser.add_argument('--input', dest='input_file', type=str, required=True,
                        help='The path of the input MAT file')
  parser.add_argument('--tests', dest='tests', type=int, default=1000,
                        help='Tests to perform')
  args = parser.parse_args()
  #Generate a new input data file
  if args.generate:
    generate_input_data(args.input_file, args.tests)
  # Test the available IK solvers
  client = dynamic_reconfigure.client.Client('/grips/kinematic_services', timeout=1)
  solvers = {0:'ikfast', 1:'lma', 2:'decoup', 3:'kdl'}
  for key, solver in solvers.iteritems():
    client.update_configuration({'kinematics_solver':key})
    solve_ik(args.input_file, solver)
    rospy.sleep(10.0) # Give it time to reset the solver
