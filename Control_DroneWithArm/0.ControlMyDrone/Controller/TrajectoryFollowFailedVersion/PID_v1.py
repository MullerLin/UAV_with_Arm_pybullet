import os
import math
import numpy as np
import pybullet as p
from enum import Enum
import xml.etree.ElementTree as etxml
from scipy.spatial.transform import Rotation

class DroneModel(Enum):
    """Drone models enumeration class."""

    CF2X = "cf2x"   # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = "cf2p"   # Bitcraze Craziflie 2.0 in the + configuration
    HB = "hb"       # Generic quadrotor (with AscTec Hummingbird inertial properties)
    HBARM = "hbarm"

class DroneControl(object):
    def __init__(self,
                 drone_model: DroneModel,
                 g: float = 10
                 ):
        #### Set general use constants #############################
        self.DRONE_MODEL = drone_model
        """DroneModel: The type of drone to control."""
        self.GRAVITY = g * self._getURDFParameter('m')
        """float: The gravitational force (M*g) acting on each drone."""
        self.KF = self._getURDFParameter('kf')
        """float: The coefficient converting RPMs into thrust."""
        self.KM = self._getURDFParameter('km')
        """float: The coefficient converting RPMs into torque."""
        if self.DRONE_MODEL != DroneModel.HB and self.DRONE_MODEL != DroneModel.HBARM:
            print("[ERROR] in SimplePIDControl.__init__(), SimplePIDControl requires DroneModel.HB")
            exit()

        self.P_COEFF_FOR = np.array([200, 200, 200])
        self.I_COEFF_FOR = np.array([.01, .01, .01])
        self.D_COEFF_FOR = np.array([20, 20, 30])

        self.P_COEFF_TOR = np.array([100, 100, 100])
        self.I_COEFF_TOR = np.array([.5, .5, .5])
        self.D_COEFF_TOR = np.array([.1, .1, .25])

        self.MAX_ROLL_PITCH = np.pi/6
        self.L = self._getURDFParameter('arm')
        self.THRUST2WEIGHT_RATIO = self._getURDFParameter('thrust2weight')
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY) / (4*self.KF))
        self.MAX_THRUST = (3*4*self.KF*self.MAX_RPM**2)
        self.MAX_XY_TORQUE = (5*self.L*self.KF*self.MAX_RPM**2)
        self.MAX_Z_TORQUE = (5*2*self.KM*self.MAX_RPM**2)
        self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1]])
        self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([1/self.KF, 1/(self.KF*self.L), 1/(self.KF*self.L), 1/self.KM])
        self.reset()

    def reset(self):
        """Reset the control classes.

        A general use counter is set to zero.

        """
        self.control_counter = 0

        #### Initialized PID control variables #####################
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    def computeControlFromState(self,
                                control_timestep,
                                cur_pos,
                                cur_quat,
                                target_pos,
                                target_rpy=np.zeros(3),
                                    # target_vel=np.zeros(3),
                                    # target_rpy_rates=np.zeros(3)
                                ):

        control_timestep = control_timestep,
        cur_pos = cur_pos
        cur_quat = cur_quat
        # print(cur_pos)
        # print(cur_quat)
        target_pos = target_pos
        target_rpy = target_rpy
        # target_vel = target_vel
        # target_rpy_rates = target_rpy_rates
        # cur_vel = state[10:13]
        # cur_ang_vel = state[13:16]
        self.control_counter += 1
        if target_rpy[2]!=0:
            print("\n[WARNING] ctrl it", self.control_counter, "in SimplePIDControl.computeControl(), desired yaw={:.0f}deg but locked to 0. for DroneModel.HB".format(target_rpy[2]*(180/np.pi)))
        target_force, target_rpy = self._simplePIDPositionControl(control_timestep = control_timestep,
                                                                            cur_pos = cur_pos,
                                                                            cur_quat = cur_quat,
                                                                            target_pos = target_pos
                                                                            )
        target_torques, rpy_e = self._simplePIDAttitudeControl(control_timestep=control_timestep,
                                             cur_quat=cur_quat,
                                             target_rpy=target_rpy,
                                             )
        if abs(target_torques[0]) > self.MAX_XY_TORQUE:
            print("[WARNING] iter", self.control_counter,
              "in utils.nnlsRPM(), unfeasible roll torque {:.2f} outside range [{:.2f}, {:.2f}]".format(target_torques[0] ,
                                                                                                        -self.MAX_XY_TORQUE,
                                                                                                        self.MAX_XY_TORQUE))
            #target_torques[0] = self.MAX_XY_TORQUE

        if abs(target_torques[1]) > self.MAX_XY_TORQUE:
            print("[WARNING] iter", self.control_counter,
              "in utils.nnlsRPM(), unfeasible pitch torque {:.2f} outside range [{:.2f}, {:.2f}]".format(target_torques[1] ,
                                                                                                        -self.MAX_XY_TORQUE,
                                                                                                        self.MAX_XY_TORQUE))
            #target_torques[1] = self.MAX_XY_TORQUE
        if abs(target_torques[2]) > self.MAX_Z_TORQUE:
            print("[WARNING] iter", self.control_counter,
              "in utils.nnlsRPM(), unfeasible yaw torque {:.2f} outside range [{:.2f}, {:.2f}]".format(target_torques[2] ,
                                                                                                        -self.MAX_Z_TORQUE,
                                                                                                       self.MAX_Z_TORQUE))
            #target_torques[2] = self.MAX_Z_TORQUE

        if np.linalg.norm(target_force) > self.MAX_THRUST:
            print("[WARNING] iter", self.control_counter,
                  "in utils.nnlsRPM(), unfeasible thrust {:.2f} outside range [0, {:.2f}]".format(np.linalg.norm(target_force), self.MAX_THRUST))

            #target_force = target_force / np.linalg.norm(target_force) * self.MAX_THRUST

        #print("[INFO] target_force : ", target_force, "target_torques : ", target_torques)
        return target_force, target_torques, target_rpy

    def _simplePIDPositionControl(self,
                                  control_timestep,
                                  cur_pos,
                                  cur_quat,
                                  target_pos
                                  ):

        pos_e = target_pos - np.array(cur_pos).reshape(3)
        print("[INFO] ERROR in Position: ", pos_e)
        d_pos_e = (pos_e - self.last_pos_e) / control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        print("[Gravity] Gravity: ", np.array([0, 0, self.GRAVITY]))
        matrix = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        gravity = np.dot(np.array([0, 0, self.GRAVITY]), matrix)
        print("[Gravity] Gravity: ", gravity)
        print("FORCE : ", np.multiply(self.P_COEFF_FOR, pos_e))
        #### PID target thrust #####################################
        target_force = np.array([0, 0, self.GRAVITY]) \
                       + np.multiply(self.P_COEFF_FOR, pos_e) \
                       + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                       + np.multiply(self.D_COEFF_FOR, d_pos_e)
        # target_force = np.array(gravity)\
        #                + np.multiply(self.P_COEFF_FOR, pos_e) \
        #                + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
        #                + np.multiply(self.D_COEFF_FOR, d_pos_e)

        target_rpy = np.zeros(3)
        sign_z =  np.sign(target_force[2])
        if sign_z == 0:
            sign_z = 1
        #### Target rotation #######################################
        target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
        target_rpy[1] = np.arctan2(sign_z*target_force[0], sign_z*target_force[2])
        target_rpy[2] = 0.
        target_rpy[0] = np.clip(target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        target_rpy[1] = np.clip(target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
        #thrust = np.dot(cur_rotation, target_force)
        return  target_force, target_rpy

    ################################################################################

    def _simplePIDAttitudeControl(self,
                                  control_timestep,
                                  cur_quat,
                                  target_rpy
                                  ):
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        rpy_e = target_rpy - np.array(cur_rpy).reshape(3,)
        print("[INFO] ERROR in RPY: ", rpy_e)
        print("[INFO] TARGET RPY  : X:{:+06.4f}, Y:{:+06.4f}, Z:{:+06.4f}".format(target_rpy[0],
                                                                                       target_rpy[1],
                                                                                       target_rpy[2]))
        if rpy_e[2] > np.pi:
            rpy_e[2] = rpy_e[2] - 2*np.pi
        if rpy_e[2] < -np.pi:
            rpy_e[2] = rpy_e[2] + 2*np.pi
        d_rpy_e = (rpy_e - self.last_rpy_e) / control_timestep
        self.last_rpy_e = rpy_e
        self.integral_rpy_e = self.integral_rpy_e + rpy_e*control_timestep
        #### PID target torques ####################################
        target_torques = np.multiply(self.P_COEFF_TOR, rpy_e) \
                         + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e) \
                         + np.multiply(self.D_COEFF_TOR, d_rpy_e)
        return target_torques, rpy_e

    ################################################################################

    def setPIDCoefficients(self,
                           p_coeff_pos=None,
                           i_coeff_pos=None,
                           d_coeff_pos=None,
                           p_coeff_att=None,
                           i_coeff_att=None,
                           d_coeff_att=None
                           ):
        """Sets the coefficients of a PID controller.

        This method throws an error message and exist is the coefficients
        were not initialized (e.g. when the controller is not a PID one).

        Parameters
        ----------
        p_coeff_pos : ndarray, optional
            (3,1)-shaped array of floats containing the position control proportional coefficients.
        i_coeff_pos : ndarray, optional
            (3,1)-shaped array of floats containing the position control integral coefficients.
        d_coeff_pos : ndarray, optional
            (3,1)-shaped array of floats containing the position control derivative coefficients.
        p_coeff_att : ndarray, optional
            (3,1)-shaped array of floats containing the attitude control proportional coefficients.
        i_coeff_att : ndarray, optional
            (3,1)-shaped array of floats containing the attitude control integral coefficients.
        d_coeff_att : ndarray, optional
            (3,1)-shaped array of floats containing the attitude control derivative coefficients.

        """
        ATTR_LIST = ['P_COEFF_FOR', 'I_COEFF_FOR', 'D_COEFF_FOR', 'P_COEFF_TOR', 'I_COEFF_TOR', 'D_COEFF_TOR']
        if not all(hasattr(self, attr) for attr in ATTR_LIST):
            print(
                "[ERROR] in BaseControl.setPIDCoefficients(), not all PID coefficients exist as attributes in the instantiated control class.")
            exit()
        else:
            self.P_COEFF_FOR = self.P_COEFF_FOR if p_coeff_pos is None else p_coeff_pos
            self.I_COEFF_FOR = self.I_COEFF_FOR if i_coeff_pos is None else i_coeff_pos
            self.D_COEFF_FOR = self.D_COEFF_FOR if d_coeff_pos is None else d_coeff_pos
            self.P_COEFF_TOR = self.P_COEFF_TOR if p_coeff_att is None else p_coeff_att
            self.I_COEFF_TOR = self.I_COEFF_TOR if i_coeff_att is None else i_coeff_att
            self.D_COEFF_TOR = self.D_COEFF_TOR if d_coeff_att is None else d_coeff_att

    ################################################################################



    ################################################################################

    def _getURDFParameter(self,
                          parameter_name: str
                          ):
        """Reads a parameter from a drone's URDF file.

        This method is nothing more than a custom XML parser for the .urdf
        files in folder `asset/`.

        Parameters
        ----------
        parameter_name : str
            The name of the parameter to read.

        Returns
        -------
        float
            The value of the parameter.

        """
        #### Get the XML tree of the drone model to control ########
        URDF = self.DRONE_MODEL.value + ".urdf"
        URDF_TREE = etxml.parse(os.path.dirname(os.path.abspath(__file__)) + "/../asset/" + URDF).getroot()
        #### Find and return the desired parameter #################
        if parameter_name == 'm':
            return float(URDF_TREE[1][0][1].attrib['value'])\
                   + float(URDF_TREE[12][0][1].attrib['value']) \
                   + float(URDF_TREE[14][0][1].attrib['value'])  \
                   + float(URDF_TREE[16][0][1].attrib['value'])  \
                   + float(URDF_TREE[18][0][1].attrib['value'])
                 # [0] mass of drone, [12] link1, [14] link2, [16] cameralink, [18] gripper
        elif parameter_name in ['ixx', 'iyy', 'izz']:
            return float(URDF_TREE[1][0][2].attrib[parameter_name])
        elif parameter_name in ['arm', 'thrust2weight', 'kf', 'km', 'max_speed_kmh', 'gnd_eff_coeff' 'prop_radius', \
                                'drag_coeff_xy', 'drag_coeff_z', 'dw_coeff_1', 'dw_coeff_2', 'dw_coeff_3']:
            return float(URDF_TREE[0].attrib[parameter_name])
        elif parameter_name in ['length', 'radius']:
            return float(URDF_TREE[1][2][1][0].attrib[parameter_name])
        elif parameter_name == 'collision_z_offset':
            COLLISION_SHAPE_OFFSETS = [float(s) for s in URDF_TREE[1][2][0].attrib['xyz'].split(' ')]
            return COLLISION_SHAPE_OFFSETS[2]

