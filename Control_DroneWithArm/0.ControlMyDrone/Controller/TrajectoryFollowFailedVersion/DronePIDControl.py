import os
import math
import numpy as np
import pybullet as p
from enum import Enum
import xml.etree.ElementTree as etxml
from scipy.spatial.transform import Rotation
# from gym_pybullet_drones.utils.utils import nnlsRPM
# from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary
from scipy.optimize import nnls

class DroneModel(Enum):
    """Drone models enumeration class."""

    CF2X = "cf2x"   # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = "cf2p"   # Bitcraze Craziflie 2.0 in the + configuration
    HB = "hb"       # Generic quadrotor (with AscTec Hummingbird inertial properties)
    HBARM = "hbarm"

class BaseControl(object):
    """Base class for control.

    Implements `__init__()`, `reset(), and interface `computeControlFromState()`,
    the main method `computeControl()` should be implemented by its subclasses.

    """

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel,
                 g: float = 9.8
                 ):
        """Common control classes __init__ method.

        Parameters
        ----------
        drone_model : DroneModel
            The type of drone to control (detailed in an .urdf file in folder `asset`).
        g : float, optional
            The gravitational acceleration in m/s^2.

        """
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
        self.P_COEFF_FOR = np.array([.1, .1, .2])
        self.I_COEFF_FOR = np.array([.0001, .0001, .0001])
        self.D_COEFF_FOR = np.array([.3, .3, .4])
        self.P_COEFF_TOR = np.array([.1, .1, .02])
        self.I_COEFF_TOR = np.array([.0001, .0001, .0001])
        self.D_COEFF_TOR = np.array([.1, .1, .25])
        # self.P_COEFF_FOR = np.array([.5, .5, 1.0])
        # self.I_COEFF_FOR = np.array([.0005, .0005, .0005])
        # self.D_COEFF_FOR = np.array([1.5, 1.5, 2.0])
        # self.P_COEFF_TOR = np.array([.4, .4, .05])
        # self.I_COEFF_TOR = np.array([.0002, .0002, .0002])
        # self.D_COEFF_TOR = np.array([.4, .4, .8])
        self.MAX_ROLL_PITCH = np.pi/6
        self.L = self._getURDFParameter('arm')
        self.THRUST2WEIGHT_RATIO = self._getURDFParameter('thrust2weight')
        self.MAX_RPM = np.sqrt((self.THRUST2WEIGHT_RATIO*self.GRAVITY) / (4*self.KF))
        self.MAX_THRUST = (4*self.KF*self.MAX_RPM**2)
        self.MAX_XY_TORQUE = (self.L*self.KF*self.MAX_RPM**2)
        self.MAX_Z_TORQUE = (2*self.KM*self.MAX_RPM**2)
        self.A = np.array([ [1, 1, 1, 1], [0, 1, 0, -1], [-1, 0, 1, 0], [-1, 1, -1, 1] ])
        self.INV_A = np.linalg.inv(self.A)
        self.B_COEFF = np.array([1/self.KF, 1/(self.KF*self.L), 1/(self.KF*self.L), 1/self.KM])
        self.reset()

    ################################################################################

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

    ################################################################################

    def computeControlFromState(self,
                                control_timestep,
                                state,
                                target_pos,
                                target_rpy=np.zeros(3),
                                target_vel=np.zeros(3),
                                target_rpy_rates=np.zeros(3)
                                ):
        """Interface method using `computeControl`.

        It can be used to compute a control action directly from the value of key "state"
        in the `obs` returned by a call to BaseAviary.step().

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        state : ndarray
            (20,)-shaped array of floats containing the current state of the drone.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_rpy_rates : ndarray, optional
            (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

        """
        return self.computeControl(control_timestep=control_timestep,
                                   cur_pos=state[0:3],
                                   cur_quat=state[3:7],
                                   cur_vel=state[10:13],
                                   cur_ang_vel=state[13:16],
                                   target_pos=target_pos,
                                   target_rpy=target_rpy,
                                   target_vel=target_vel,
                                   target_rpy_rates=target_rpy_rates
                                   )

    ################################################################################

    def computeControl(self,
                       control_timestep,
                       cur_pos,
                       cur_quat,
                       cur_vel,
                       cur_ang_vel,
                       target_pos,
                       target_rpy=np.zeros(3),
                       target_vel=np.zeros(3),
                       target_rpy_rates=np.zeros(3)
                       ):
        """Abstract method to compute the control action for a single drone.

        It must be implemented by each subclass of `BaseControl`.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_rpy_rates : ndarray, optional
            (3,1)-shaped array of floats containing the desired roll, pitch, and yaw rates.

        """
        """Computes the PID control action (as RPMs) for a single drone.

        This methods sequentially calls `_simplePIDPositionControl()` and `_simplePIDAttitudeControl()`.
        Parameters `cur_ang_vel`, `target_rpy`, `target_vel`, and `target_rpy_rates` are unused.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_rpy_rates : ndarray, optional
            (3,1)-shaped array of floats containing the the desired roll, pitch, and yaw rates.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.
        ndarray
            (3,1)-shaped array of floats containing the current XYZ position error.
        float
            The current yaw error.

        """
        self.control_counter += 1
        if target_rpy[2]!=0:
            print("\n[WARNING] ctrl it", self.control_counter, "in SimplePIDControl.computeControl(), desired yaw={:.0f}deg but locked to 0. for DroneModel.HB".format(target_rpy[2]*(180/np.pi)))
        thrust, computed_target_rpy, pos_e = self._simplePIDPositionControl(control_timestep,
                                                                            cur_pos,
                                                                            cur_quat,
                                                                            target_pos
                                                                            )
        rpm = self._simplePIDAttitudeControl(control_timestep,
                                             thrust,
                                             cur_quat,
                                             computed_target_rpy
                                             )
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]


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
            return float(URDF_TREE[1][0][1].attrib['value'])
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

    def _simplePIDPositionControl(self,
                                  control_timestep,
                                  cur_pos,
                                  cur_quat,
                                  target_pos
                                  ):
        """Simple PID position control (with yaw fixed to 0).

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.

        Returns
        -------
        float
            The target thrust along the drone z-axis.
        ndarray
            (3,1)-shaped array of floats containing the target roll, pitch, and yaw.
        float
            The current position error.

        """
        pos_e = target_pos - np.array(cur_pos).reshape(3)
        d_pos_e = (pos_e - self.last_pos_e) / control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*control_timestep
        #### PID target thrust #####################################
        target_force = np.array([0, 0, self.GRAVITY]) \
                       + np.multiply(self.P_COEFF_FOR, pos_e) \
                       + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                       + np.multiply(self.D_COEFF_FOR, d_pos_e)
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
        thrust = np.dot(cur_rotation, target_force)
        return thrust[2], target_rpy, pos_e

    ################################################################################

    def _simplePIDAttitudeControl(self,
                                  control_timestep,
                                  thrust,
                                  cur_quat,
                                  target_rpy
                                  ):
        """Simple PID attitude control (with yaw fixed to 0).

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        thrust : float
            The target thrust along the drone z-axis.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        target_rpy : ndarray
            (3,1)-shaped array of floats containing the computed the target roll, pitch, and yaw.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.

        """
        cur_rpy = p.getEulerFromQuaternion(cur_quat)
        rpy_e = target_rpy - np.array(cur_rpy).reshape(3,)
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
        return nnlsRPM(thrust=thrust,
                       x_torque=target_torques[0],
                       y_torque=target_torques[1],
                       z_torque=target_torques[2],
                       counter=self.control_counter,
                       max_thrust=self.MAX_THRUST,
                       max_xy_torque=self.MAX_XY_TORQUE,
                       max_z_torque=self.MAX_Z_TORQUE,
                       a=self.A,
                       inv_a=self.INV_A,
                       b_coeff=self.B_COEFF,
                       gui=True
                       )

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

def nnlsRPM(thrust,
            x_torque,
            y_torque,
            z_torque,
            counter,
            max_thrust,
            max_xy_torque,
            max_z_torque,
            a,
            inv_a,
            b_coeff,
            gui=False
            ):
    """Non-negative Least Squares (NNLS) RPMs from desired thrust and torques.

    This function uses the NNLS implementation in `scipy.optimize`.

    Parameters
    ----------
    thrust : float
        Desired thrust along the drone's z-axis.
    x_torque : float
        Desired drone's x-axis torque.
    y_torque : float
        Desired drone's y-axis torque.
    z_torque : float
        Desired drone's z-axis torque.
    counter : int
        Simulation or control iteration, only used for printouts.
    max_thrust : float
        Maximum thrust of the quadcopter.
    max_xy_torque : float
        Maximum torque around the x and y axes of the quadcopter.
    max_z_torque : float
        Maximum torque around the z axis of the quadcopter.
    a : ndarray
        (4, 4)-shaped array of floats containing the motors configuration.
    inv_a : ndarray
        (4, 4)-shaped array of floats, inverse of a.
    b_coeff : ndarray
        (4,1)-shaped array of floats containing the coefficients to re-scale thrust and torques.
    gui : boolean, optional
        Whether a GUI is active or not, only used for printouts.

    Returns
    -------
    ndarray
        (4,)-shaped array of ints containing the desired RPMs of each propeller.

    """
    #### Check the feasibility of thrust and torques ###########
    if gui and thrust < 0 or thrust > max_thrust:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible thrust {:.2f} outside range [0, {:.2f}]".format(thrust, max_thrust))
    if gui and np.abs(x_torque) > max_xy_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible roll torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -max_xy_torque, max_xy_torque))
    if gui and np.abs(y_torque) > max_xy_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible pitch torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -max_xy_torque, max_xy_torque))
    if gui and np.abs(z_torque) > max_z_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible yaw torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -max_z_torque, max_z_torque))
    B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), b_coeff)
    sq_rpm = np.dot(inv_a, B)
    #### NNLS if any of the desired ang vel is negative ########
    if np.min(sq_rpm) < 0:
        sol, res = nnls(a,
                        B,
                        maxiter=3*a.shape[1]
                        )
        if gui:
            print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible squared rotor speeds, using NNLS")
            print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                   "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
            print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                  "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                  "\t\tResidual: {:.2f}".format(res))
        sq_rpm = sol
    return np.sqrt(sq_rpm)