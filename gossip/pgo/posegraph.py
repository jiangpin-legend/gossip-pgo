#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
import sys

import g2o
import numpy as np
from sophus import SO3,SE3

from .viewer import Viewer3D
from .multi_viewer import MultiViewer3D
from .multi_robot_tools import MultiRobotTools
from .g2o_tool import G2oTool


class PoseGraph3D(object):
  nodes = []
  edges = []
  nodes_optimized = []
  edges_optimized = []

  def __init__(self, verbose=False,robot_id = 0):
    self.solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    self.solver=  g2o.OptimizationAlgorithmLevenberg(self.solver)
    self.robot_id = robot_id

    self.optimizer = g2o.SparseOptimizer()
    self.optimizer.set_verbose(verbose)
    self.optimizer.set_algorithm(self.solver)
    self.g2o_tool = G2oTool()
    self.multi_robot_tools = MultiRobotTools()


  def load_file(self, fname):
    self.optimizer.load(fname)
    print("vertices: ", len(self.optimizer. vertices()))
    print("edges: ", len(self.optimizer.edges()))

    # self.edges_key_pairs = self.g2o_tool.read_edge_keys(fname)
    self.edges_key_pairs = []

    for edge in self.optimizer.edges():
      self.edges.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])
      self.edges_key_pairs.append([edge.vertices()[0].id(),edge.vertices()[1].id()])

    self.nodes = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_keys = [key for key in self.optimizer.vertices().keys()]

    self.nodes = np.array(self.nodes)
    self.edges = np.array(self.edges)
    self.nodes_keys = np.array(self.nodes_keys)
    self.edges_key_pairs = np.array(self.edges_key_pairs)
    self.init_separator()
    # print(self.separator_nodes.shape)

    # print(len(self.edges_key_pairs))
    
    # print(self.nodes_keys)
    # print(self.edges_key_pairs)

    # self.nodes_keys = np.array(self.nodes_keys)
    # print(self.nodes_keys)
    # print(self.edges_key_pairs)   

  def optimize(self, iterations=1):
    self.optimizer.initialize_optimization()
    self.optimizer.optimize(iterations)

    self.optimizer.save("data/out.g2o")
    self.edges_optimized = []
    for edge in self.optimizer.edges():
      self.edges_optimized.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    self.nodes_optimized = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_optimized = np.array(self.nodes_optimized)
    self.edges_optimized = np.array(self.edges_optimized)

  def init_separator(self):
    separator_key = np.array( [key_pair for key_pair in self.edges_key_pairs if (self.multi_robot_tools.key2robot_id_g2o(key_pair[0])!=self.multi_robot_tools.key2robot_id_g2o(key_pair[1])) ] )

    self.separator_edge_mask = np.array([ (self.multi_robot_tools.is_separator_g2o(key_pair)) for key_pair in self.edges_key_pairs])

    separator_node_key = []
    for key_pair in separator_key:
      separator_node_key.append(key_pair[0])
      separator_node_key.append(key_pair[1])
    separator_node_key = np.array(separator_node_key)
    separator_node_key = np.unique(separator_node_key)
    self.separator_node_key = separator_node_key

    self.separator_node_mask = np.isin(self.nodes_keys,separator_node_key)
    # print(separator_node_mask)
    self.separator_edges = np.array(self.edges[self.separator_edge_mask])
    self.separator_nodes = np.array(self.nodes[self.separator_node_mask])
    self.separator_nodes_se3 = []
    for i in range(self.separator_nodes.shape[0]):
      matrix = self.separator_nodes[i,:,:]
      se3_node = SE3(matrix).log()
      self.separator_nodes_se3.append(se3_node)
    self.separator_nodes_se3 = np.array(self.separator_nodes_se3)[:,:,0]
    print(self.separator_nodes_se3.shape)



class PoseGraphSE3(PoseGraph3D):
  def __init__(self, verbose=False, robot_id=0):
    super().__init__(verbose, robot_id)

  def load_file(self, fname):
    print("loading:"+fname)
    self.optimizer.load(fname)
    self.optimizer.set_verbose(True)
    print("vertices: ", len(self.optimizer.vertices()))
    print("edges: ", len(self.optimizer.edges()))

    # self.edges_key_pairs = self.g2o_tool.read_edge_keys(fname)
    self.edges_key_pairs = []

    for edge in self.optimizer.edges():
      self.edges.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])
      self.edges_key_pairs.append([edge.vertices()[0].id(),edge.vertices()[1].id()])

    self.nodes = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_keys = [key for key in self.optimizer.vertices().keys()]

    self.nodes = np.array(self.nodes)
    self.edges = np.array(self.edges)
    self.nodes_keys = np.array(self.nodes_keys)
    self.edges_key_pairs = np.array(self.edges_key_pairs)
    self.init_separator()
    self.optimizer.initialize_optimization()
    

  def rotationMatrixToQuaternion(self,m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[0] = 0.5 * t
        t = 0.5/t
        q[1] = (m[2,1] - m[1,2]) * t
        q[2] = (m[0,2] - m[2,0]) * t
        q[3] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[0] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

  def linearize_at(self,poses,linearized_point):
    #linearize se3 at linearizePoint
      #pose:se3 vector6
      #linearized_point: vector6

    linearized_poses = []
    r_ref = linearized_point[3:6]

    R_ref = SO3().exp(r_ref)
    R_ref_inv = R_ref.inverse()
   
    for pose in poses:
      pose_se3 = SE3().exp(pose)
      rot_se3 = pose_se3.rotationMatrix()
      rot_in_ref = np.dot(R_ref_inv,rot_se3)
      linearized_rot = SO3(rot_in_ref).log()
      linearized_pose = [0,0,0,0,0,0]
      linearized_pose[0:3] = pose[0:3]
      linearized_pose[3:6] = linearized_rot
      linearized_poses.append(linearized_pose)

    linearized_poses = np.array(linearized_poses)
    return linearized_poses

  def update_pose(self,pose,delta_pose):
    delta_trans = delta_pose[0:3]
    delta_rot = delta_pose[3:6]

    delta_Rot = SO3().exp(delta_rot)
    new_Rot = np.dot(pose.rotationMatrix(),delta_Rot)
    new_rot = SO3().log(new_Rot)

    new_trans = pose[0:3]+delta_trans
    new_pose = np.concatenate(new_trans,new_rot)

    return new_pose

  def update_separator(self,new_poses):
    for id in self.separator_node_key:
      new_pose = g2o.Isometry3d(SO3(new_poses[3:6]).matrix(), new_poses[0:3])
      vc = self.optimizer.vertices()[id]
      vc.set_estimate(new_pose)

  def average_at(self,new_poses,weights,local_pose,local_weight):
    linearized_poses = self.linearize_at(new_poses,local_pose)
    avg_pose = np.dot(local_weight,local_pose)+np.dot(weights,linearized_poses)
    return avg_pose

  def optimize(self, iterations=1):
    self.optimizer.optimize(iterations)
    separator_vec_list = []
    for key in self.separator_node_key:
      #return separator vec
      separator_vec = SE3(self.optimizer.vertices()[key].estimate().matrix()).log()
      separator_vec_list.append(separator_vec)
    #the key order might be different,should check carefully
    return separator_vec_list
    # self.optimizer.save("data/out.g2o")
    # self.edges_optimized = []
    # for edge in self.optimizer.edges():
      # self.edges_optimized.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    # self.nodes_optimized = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    # self.nodes_optimized = np.array(self.nodes_optimized)
    # self.edges_optimized = np.array(self.edges_optimized)

if __name__ == "__main__":
  if len(sys.argv) > 1:
    gfile = str(sys.argv[1])
  else:
    # gfile = "/home/jiangpin/dataset/example_4robots/3_renamed.g2o"
    gfile = "/home/jiangpin/dataset/example_4robots/full_graph_renamed.g2o"
    # gfile = "./data/sphere2500.g2o"

    


  graph = PoseGraph3D()
  graph.load_file(gfile)
  #graph.optimize()
  print("loaded")
  viewer = MultiViewer3D(graph,4)

  # graph.optimize()
  # viewer = Viewer3D(graph)




