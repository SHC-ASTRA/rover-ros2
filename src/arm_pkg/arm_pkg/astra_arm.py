import numpy as np
import time, math, os
from math import sin, cos, pi
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
#import pygame as pyg
from scipy.spatial.transform import Rotation as R


# Misc
degree = pi / 180.0


def convert_angles(angles):
    # Converts angles to the format used for the urdf (contains some dummy joints)
    return [0.0, 0.0, angles[0], angles[1], angles[2], angles[3], angles[4], 0.0]


class arm:
    def __init__(self, urdf_name):
        self.ik_tolerance = 1e-3 #Tolerance (in meters) to determine if solution is valid
        # URDF file path
        self.urdf = os.path.join(get_package_share_directory('ik_pkg'), urdf_name)
        # IKpy Chain        
        self.chain = Chain.from_urdf_file(self.urdf)   


        # Arrays for joint states
        # Some links in the URDF are static (non-joints), these will remain zero for IK
        # Indexes: Ignore, Ignore, Ax_0, Ax_1, Ax2, Ax_3, Wrist, Ignore
        self.zero_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_angles = self.zero_angles
        self.last_angles = self.zero_angles
        self.ik_angles = self.zero_angles

        self.current_position = []
        self.target_position = [0.0, 0.0, 0.0] 
        self.target_orientation = [] # Effector orientation desired at target position. 
                                # Generally orientation for the effector is modified manually by the operator. 
        
        # Might not need, copied over from state_publisher.py in ik_test
        #self.step = 0.03 # Max movement increment


    # Get current orientation of the end effector and update target_orientation
    def update_orientation(self):

        # FK matrix for arm's current pose
        fk_matrix = self.chain.forward_kinematics(self.current_angles)
        
        # Update target_orientation to the effector's current orientation
        self.target_orientation = fk_matrix[:3, :3]

    # Update current angles to those provided
    # Resetting last_angles to the new angles
    #
    # Use: First call, or when angles are changed manually.
    def reset_angles(self, angles):
        # Update angles to the new angles
        self.current_angles = convert_angles(angles)
        self.last_angles = self.current_angles

    # Update current angles to those provided
    # Maintain previous angles in last_angles
    #
    # Use: Repeated calls during IK operation
    def update_angles(self, angles):
        # Update angles to the new angles
        self.last_angles = self.current_angles
        self.current_angles = convert_angles(angles)
    
    # Get current X,Y,Z position of end effector
    def get_position(self):
        # FK matrix for arm's current pose
        fk_matrix = self.chain.forward_kinematics(self.current_angles)
        
        # Get the position of the end effector from the FK matrix
        position = fk_matrix[:3, 3]
        
        return position
    
    def update_position(self):
        # FK matrix for arm's current pose
        fk_matrix = self.chain.forward_kinematics(self.current_angles)
        
        # Get the position of the end effector from the FK matrix and update current pos
        self.current_position = fk_matrix[:3, 3]


        
        