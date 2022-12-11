#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
import sys

import g2o
import numpy as np
from sophus import *

from viewer import Viewer3D
from multi_viewer import MultiViewer3D
from multi_robot_tools import MultiRobotTools
from g2o_tool import G2oTool


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
    print(self.separator_nodes.shape)

    print(len(self.edges_key_pairs))
    
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
    separator_edge_mask = []

    separator_edge_mask = np.array([ (self.multi_robot_tools.is_separator_g2o(key_pair)) for key_pair in self.edges_key_pairs])

    separator_node_key = []
    for key_pair in separator_key:
      separator_node_key.append(key_pair[0])
      separator_node_key.append(key_pair[1])
    separator_node_key = np.array(separator_node_key)
    separator_node_key = np.unique(separator_node_key)

    separator_node_mask = np.isin(self.nodes_keys,separator_node_key)

    # print(separator_node_mask)
    self.separator_edges = np.array(self.edges[separator_edge_mask])
    self.separator_nodes = np.array(self.nodes[separator_node_mask])

  def linearize(self,poses,linearize_point):
    R_ref = np.matrix(linearize_point[0:3,0:3])
    R_ref_inv = R_ref.getI()
    linearize_pose = []
    for pose in poses:
      rot = np.matrix(pose[0:3,0:3])
      rot_ref = np.dot(rot,R_ref_inv)
      linearize_rot = np.array(SO3(rot_ref).log())
      linearize_pose.append(np.concatenate(linearize_point[0:2,3],linearize_rot))
    linearize_pose = np.array(linearize_pose)
    

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




