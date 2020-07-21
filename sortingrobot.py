from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageOps
import libry as ry

from utils import *


class SortingRobot:
    """Creates a simulation in which robot arms are sorting objects to baskets based on their color."""

    def __init__(self, scene_path, n_objects, spawn_special=False, save_video=False):
        """Initializes simulation, model config and objects containing information about the scene.

        Args:
            scene_path (str): path to .g file which contains the scene
            n_objects (int): number of objects to spawn randomly in a predefined area of the scene
            spawn_special (bool): spawn an additional sphere which has to be thrown to put in the basket if true
            save_video (bool): export frames for every timestep in the simulation if true
        """
        self.pause = .002
        self.tau = .01

        # create simulation
        self.RealWorld = None
        self.sim_objects = []
        self.S = None
        self.setup_simulation(scene_path, n_objects, spawn_special)

        # create state dict of objects
        self.obj_info = [
            {
                'status': 'dropped',
                'grasp_attempts': 0,
                'pos': None,
                'quat': None,
                'color_code': None,
                'color': None,
                'shape': None,
                'size': None
            } for _ in range(len(self.sim_objects))
        ]
        self.get_obj_info()

        # create model world
        self.C, self.V, self.config_objects = self.setup_config(scene_path)

        # get initial joint configuration
        self.q_init = self.C.getJointState([])

        # option to save single frames for a video
        self.save_video = save_video
        if self.save_video:
            self.video_counter = 0
            self.save_path = './images/{}'.format(time.strftime('%Y%m%d-%H%M%S'))
            Path(self.save_path).mkdir(parents=True, exist_ok=True)

    def close_hand2_open_hand1(self, arm1, arm2):
        """Moves the grippers to handover an object.

        Args:
            arm1 (str): prefix of arm that currently holds the object
            arm2 (str): prefix of arm that will receiver the object

        Returns:
            bool: true if handover was successful, false if not
        """
        status = False

        self.S.closeGripper('{}_gripper'.format(arm2))

        # keep simulation running until handover was successful or not
        while True:
            time.sleep(self.pause)

            self.S.step([], self.tau, ry.ControlMode.none)

            if self.save_video:
                self.save_video_frame()

            if self.S.getGripperIsGrasping('{}_gripper'.format(arm2)):
                status = True
                break

            if self.S.getGripperWidth('{}_gripper'.format(arm2)) < -0.04:
                break

        self.S.openGripper('{}_gripper'.format(arm1))

        return status

    def destroy(self):
        """Destroys simulation, model config and viewer."""
        self.RealWorld = 0
        self.S = 0
        self.C = 0
        self.V = 0

    def drop(self, arm, duration):
        """Opens the gripper of the specified arm.

        Args:
            arm (str): prefix of arm whose gripper will be opened
            duration (int): time to run the simulation in order to move the gripper fingers (in milliseconds)
        """
        self.S.openGripper('{}_gripper'.format(arm))
        self.run_simulation(duration)

    def get_obj_info(self):
        """Updates the state dict of objects in the simulation."""
        for i, obj in enumerate(self.obj_info):
            obj['pos'] = self.sim_objects[i].getPosition()
            obj['quat'] = self.sim_objects[i].getQuaternion()
            obj['color_code'] = self.sim_objects[i].info()['color']
            obj['color'] = get_color_from_color_code(obj['color_code'])
            obj['shape'] = self.sim_objects[i].info()['shape']
            obj['size'] = self.sim_objects[i].info()['size']

    def grasp(self, object_id, arm):
        """Grasps an object with the end effector of the specified arm if well aligned.

        Args:
            object_id (int): ID of the object to be grasped
            arm (str): prefix of the arm to grasp the object

        Returns:
            bool: true if grasp was successful, false if not
        """
        self.S.closeGripper('{}_gripper'.format(arm))

        # keep the simulation running until gripper grasps
        # or until gripper failed to grasp
        while True:
            time.sleep(self.pause)

            self.S.step([], self.tau, ry.ControlMode.none)

            if self.save_video:
                self.save_video_frame()

            # check if grasp was successful
            if self.S.getGripperIsGrasping('{}_gripper'.format(arm)):
                self.C.setJointState(self.S.get_q())
                self.V.setConfiguration(self.C)
                self.obj_info[object_id]['status'] = '{}_hand'.format(arm)
                return True

            # check if grasp was not successful
            if self.S.getGripperWidth('{}_gripper'.format(arm)) < -0.04:
                self.C.setJointState(self.S.get_q())
                self.V.setConfiguration(self.C)
                self.obj_info[object_id]['status'] = 'dropped'
                return False

    def handover(self, arm1, arm2, object_id, steps):
        """Performs a handover of an object from arm1 to arm2 based on object's shape.

        Args:
            arm1 (str): prefix of first robot arm (which currently holds the object)
            arm2 (str): prefix of second robot arm (which will receive the object)
            object_id (int): ID of object to be transferred
            steps (int): number of interpolation steps
        """
        if self.obj_info[object_id]['shape'] == 'sphere':
            return self.handover_sphere(arm1, arm2, steps)
        elif self.obj_info[object_id]['shape'] == 'capsule':
            return self.handover_capsule(arm1, arm2, object_id, steps)
        else:
            raise NotImplementedError

    def handover_capsule(self, arm1, arm2, object_id, steps):
        """Does a handover of a grasped capsule from arm1 to arm2.

        Args:
            arm1 (str): prefix of arm which is holding the capsule at the beginning
            arm2 (str): prefix of arm which will receive the capsule at the end
            object_id (int): ID of object to be transferred
            steps (int): number of interpolation steps

        Returns:
            bool: true if handover was successful, false if not
        """
        self.C.setJointState(self.S.get_q())

        # get distance from center
        length = self.obj_info[object_id]['size'][0]
        dist = length / 2 - 0.05

        # calculate robot trajectories for handover
        komo = self.C.komo_path(1, steps, 5, True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e2])
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, scale=[1e2], order=1)
        komo.addObjective(time=[1.], feature=ry.FS.positionRel, type=ry.OT.eq,
                          frames=['{}_gripperCenter'.format(arm1), '{}_gripperCenter'.format(arm2)],
                          target=[0, 2 * dist, 0])
        komo.addObjective(time=[1.], feature=ry.FS.scalarProductYY, type=ry.OT.eq,
                          frames=['{}_gripperCenter'.format(arm1), '{}_gripperCenter'.format(arm2)], target=[1])
        komo.addObjective(time=[1.], feature=ry.FS.scalarProductZZ, type=ry.OT.eq,
                          frames=['{}_gripperCenter'.format(arm1), '{}_gripperCenter'.format(arm2)], target=[-1])
        komo.optimize()

        # execute calculated trajectories
        for s in range(steps):
            time.sleep(self.pause)

            self.C.setFrameState(komo.getConfiguration(s))
            q = self.C.getJointState([])
            self.S.step(q, self.tau, ry.ControlMode.position)

            if self.save_video:
                self.save_video_frame()

        # update viewer
        self.V.setConfiguration(self.C)

        # close gripper of second arm, open gripper of first arm
        status = self.close_hand2_open_hand1(arm1, arm2)

        # move back to initial pose
        self.move_from_handover_to_init(steps)

        return status

    def handover_sphere(self, arm1, arm2, steps):
        """Does a handover of a grasped sphere from arm1 to arm2.

        Args:
            arm1 (str): prefix of arm which is holding the sphere at the beginning
            arm2 (str): prefix of arm which will receive the sphere at the end
            steps (int): number of interpolation steps

        Returns:
            bool: true if handover was successful, false if not
        """
        # get initial state
        self.C.setJointState(self.S.get_q())

        # calculate robot trajectories for handover
        komo = self.C.komo_path(1, steps, 5, True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e2])
        komo.addObjective(time=[1.], feature=ry.FS.distance, frames=['R_gripperCenter', 'L_gripperCenter'],
                          type=ry.OT.eq, scale=[10])
        komo.addObjective(time=[1.], feature=ry.FS.scalarProductXX, frames=['R_gripperCenter', 'L_gripperCenter'],
                          type=ry.OT.eq)
        komo.addObjective(time=[1.], feature=ry.FS.scalarProductZZ, frames=['R_gripperCenter', 'L_gripperCenter'],
                          type=ry.OT.eq, target=[-1])
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, scale=[1e2], order=1)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=2)
        komo.optimize()

        # execute calculated trajectories
        for s in range(steps):
            time.sleep(self.pause)

            self.C.setFrameState(komo.getConfiguration(s))
            q = self.C.getJointState([])
            self.S.step(q, self.tau, ry.ControlMode.position)

            if self.save_video:
                self.save_video_frame()

        # update viewer
        self.V.setConfiguration(self.C)

        # close gripper of second arm, open gripper of first arm
        status = self.close_hand2_open_hand1(arm1, arm2)

        # move back to initial pose
        self.move_from_handover_to_init(steps)

        return status

    def joint_space_interpolation(self, q0, q1, steps, open_gripper=False, arm=None):
        """Does joint space interpolation with a sine motion profile.

        Args:
            q0 (numpy.ndarray): angles from all joints for initial pose
            q1 (numpy.ndarray): angles from all joints for target pose
            steps (int): number of interpolation steps
            open_gripper (bool): opens gripper of specified arm halfway through the motion if true
            arm (str): prefix of robot arm, necessary only if that arm's gripper should be opened
        """
        diff = q1 - q0

        # keep all joint angles in a range between -pi and pi
        for i in range(diff.shape[0]):
            while diff[i] < -np.pi:
                diff[i] += 2 * np.pi
            while diff[i] > np.pi:
                diff[i] -= 2 * np.pi

        # interpolation
        for t in range(steps):
            time.sleep(self.pause)
            theta = -np.pi / 2 + t / (steps - 1) * np.pi
            q = q0 + diff * (0.5 + 0.5 * np.sin(theta))
            self.S.step(q, self.tau, ry.ControlMode.position)

            if open_gripper:
                assert arm is not None
                if t == steps // 2:
                    self.S.openGripper('{}_gripper'.format(arm))

            if self.save_video:
                self.save_video_frame()

    def move_from_handover_to_init(self, steps):
        """Moves robot arms back to their initial pose.

        Args:
            steps (int): number of interpolation steps
        """
        self.C.setJointState(self.S.get_q())

        # calculate robot trajectories for collision free movement
        komo = self.C.komo_path(1, steps, 5, True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq, scale=[1e2])
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, target=self.q_init)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, scale=[1e2], order=1)
        komo.addObjective(time=[1.], feature=ry.FS.qItself, type=ry.OT.eq, order=2)
        komo.optimize()

        # execute calculated trajectories
        for s in range(steps):
            time.sleep(self.pause)

            self.C.setFrameState(komo.getConfiguration(s))
            q = self.C.getJointState([])
            self.S.step(q, self.tau, ry.ControlMode.position)

            if self.save_video:
                self.save_video_frame()

        # update the viewer
        self.V.setConfiguration(self.C)

    def move_to(self, object_id, arm, steps):
        """Aligns gripper and object based on object's shape.

        Args:
            object_id (int): ID of object to be sorted
            arm (str): prefix of robot arm that will be aligned
            steps (int): number of interpolation steps
        """
        if self.obj_info[object_id]['shape'] == 'sphere':
            return self.move_to_sphere(object_id, arm, steps)
        elif self.obj_info[object_id]['shape'] == 'capsule':
            return self.move_to_capsule(object_id, arm, steps)
        else:
            raise NotImplementedError

    def move_to_basket(self, object_id, arm, steps):
        """Moves robot arm with object in hand to matching basket.

        Args:
            object_id (int): ID of the object to sort
            arm (str): prefix of the robot arm which holds the object
            steps (int): number of interpolation steps
        """
        # get initial state
        self.C.setJointState(self.S.get_q())
        q0 = self.C.getJointState([])

        # calculate robot pose for dropping the object in the basket
        komo = self.C.komo_IK(True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq)
        komo.addObjective(feature=ry.FS.qItself, frames=['L_finger1', 'R_finger1'], type=ry.OT.eq)
        komo.addObjective(feature=ry.FS.scalarProductYZ, frames=['{}_gripperCenter'.format(arm), 'world'],
                          type=ry.OT.eq, target=[1])
        komo.addObjective(feature=ry.FS.positionDiff,
                          frames=['{}_gripperCenter'.format(arm),
                                  '{}_basket'.format(self.obj_info[object_id]['color'])],
                          type=ry.OT.eq, target=[0, 0, 0.8])
        komo.optimize()

        # get robot pose for dropping
        self.C.setFrameState(komo.getConfiguration(0))
        self.V.setConfiguration(self.C)
        q1 = self.C.getJointState([])

        # interpolate in joint space between initial and calculated pose
        self.joint_space_interpolation(q0, q1, steps)

    def move_to_capsule(self, object_id, arm, steps):
        """Aligns the specified robot arm with the specified object (which should be a capsule).

        Args:
            object_id (int): ID of the object with which the arm will be aligned for grasping
            arm (str): prefix of robot arm that will be aligned with the object
            steps (int): number of interpolation steps
        """
        # get initial state
        self.C.setJointState(self.S.get_q())
        q0 = self.C.getJointState([])

        # synchronize pose of objects in config and simulation
        self.get_obj_info()
        self.config_objects[object_id].setPosition(self.obj_info[object_id]['pos'])
        self.config_objects[object_id].setQuaternion(self.obj_info[object_id]['quat'])

        # get offset from center for grasping
        length = self.obj_info[object_id]['size'][0]
        dist = length / 2.0 - 0.05

        # calculate robot pose for grasping
        komo = self.C.komo_IK(True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq)
        komo.addObjective(feature=ry.FS.qItself, frames=['L_finger1', 'R_finger1'], type=ry.OT.eq)
        komo.addObjective(feature=ry.FS.positionRel,
                          frames=['{}_gripperCenter'.format(arm), 'object_{}'.format(object_id)], type=ry.OT.eq,
                          target=[0, 0, dist])
        komo.addObjective(feature=ry.FS.scalarProductYZ,
                          frames=['{}_gripperCenter'.format(arm), 'object_{}'.format(object_id)], type=ry.OT.eq,
                          scale=[1e2], target=[1])
        komo.optimize()

        # get robot pose for grasping
        self.C.setFrameState(komo.getConfiguration(0))
        self.V.setConfiguration(self.C)
        q1 = self.C.getJointState([])

        # interpolate in joint space between initial and calculated pose
        self.joint_space_interpolation(q0, q1, steps)

    def move_to_init(self, steps):
        """Moves robot arms to their initial pose.

        Args:
            steps (int): number of interpolation steps
        """
        # get current state
        self.C.setJointState(self.S.get_q())
        q_curr = self.C.getJointState([])

        # set initial pose in config
        self.C.setJointState(self.q_init)
        self.V.setConfiguration(self.C)

        # interpolate in joint space between current and initial pose
        self.joint_space_interpolation(q_curr, self.q_init, steps)

    def move_to_sphere(self, object_id, arm, steps):
        """Aligns the specified robot arm with the specified object (which should be a sphere).

        Args:
            object_id (int): ID of the object with which the arm will be aligned for grasping
            arm (str): prefix of robot arm that will be aligned with the object
            steps (int): number of interpolation steps
        """
        # get initial state
        self.C.setJointState(self.S.get_q())
        q0 = self.C.getJointState([])

        # synchronize pose of objects in config and simulation
        self.get_obj_info()
        self.config_objects[object_id].setPosition(self.obj_info[object_id]['pos'])
        self.config_objects[object_id].setQuaternion(self.obj_info[object_id]['quat'])

        # calculate robot pose for grasping
        komo = self.C.komo_IK(True)
        komo.addObjective(feature=ry.FS.accumulatedCollisions, type=ry.OT.ineq)
        komo.addObjective(feature=ry.FS.qItself, frames=['L_finger1', 'R_finger1'], type=ry.OT.eq)
        komo.addObjective(feature=ry.FS.positionDiff,
                          frames=['{}_gripperCenter'.format(arm), 'object_{}'.format(object_id)], type=ry.OT.eq)
        komo.optimize()

        # get robot pose for grasping
        self.C.setFrameState(komo.getConfiguration(0))
        self.V.setConfiguration(self.C)
        q1 = self.C.getJointState([])

        # interpolate in joint space between initial and calculated pose
        self.joint_space_interpolation(q0, q1, steps)

    def run_simulation(self, duration):
        """Lets time pass by in simulation without further instructions.

        Args:
            duration (int): time in milliseconds
        """
        for t in range(duration // 10):
            time.sleep(self.pause)
            self.S.step([], self.tau, ry.ControlMode.none)

            if self.save_video:
                self.save_video_frame()

    def save_video_frame(self):
        """Saves image from current simulation time step."""
        img = self.S.getScreenshot()
        img = ImageOps.flip(Image.fromarray(img))
        img.save(Path(self.save_path).joinpath('{}.png'.format(self.video_counter)))
        self.video_counter += 1

    def setup_config(self, scene_path):
        """Creates model config that mirrors the simulation.

        Args:
            scene_path (str): path to .g file which contains the scene

        Returns:
            c (libry.Config): model config object
            v (libry.ConfigurationViewer): model config viewer
            objects (list): list of frames that match the objects in the simulation
        """
        # internal robot configuration
        c = ry.Config()
        c.addFile(scene_path)

        # configuration viewer
        v = ry.ConfigurationViewer()
        v.setConfiguration(c)

        # frames for objects in scene
        objects = []
        for i in range(len(self.sim_objects)):
            obj = c.addFrame('object_{}'.format(i))
            obj.setPosition(self.obj_info[i]['pos'])
            obj.setQuaternion(self.obj_info[i]['quat'])
            obj.setShape(ry.ST.sphere, [.03])
            obj.setColor(self.obj_info[i]['color_code'])
            v.setConfiguration(c)
            objects.append(obj)

        return c, v, objects

    def setup_simulation(self, scene_path, n_objects, spawn_special=False):
        """Loads and populates the scene.

        Args:
            scene_path (str): path to .g file which contains the scene
            n_objects (int): number of objects to spawn randomly in a predefined area of the scene
            spawn_special (bool): spawn an additional sphere which has to be thrown to put in the basket if true
        """
        # real world configuration
        self.RealWorld = ry.Config()
        self.RealWorld.addFile(scene_path)

        # add objects to scene
        for i in range(n_objects):
            obj = self.RealWorld.addFrame('object_{}'.format(i))
            obj.setPosition(get_random_position())
            obj.setQuaternion(get_random_quaternion())
            shape, params = get_random_shape()
            obj.setShape(shape, params)
            obj.setColor(get_random_color())
            obj.setMass(0.2)
            obj.setContact(1)
            self.sim_objects.append(obj)

        # spawn special sphere
        if spawn_special:
            obj = self.RealWorld.addFrame('object_{}'.format(n_objects))
            obj.setPosition(get_random_position())
            obj.setQuaternion(get_random_quaternion())
            obj.setShape(ry.ST.sphere, [.05])
            obj.setColor([0, 0, 0])
            obj.setMass(0.2)
            obj.setContact(1)
            self.sim_objects.append(obj)

        # turn real world configuration into simulation
        self.S = self.RealWorld.simulation(ry.SimulatorEngine.physx, True)

    def sort(self, object_ids):
        """Sorts objects to their matching bins (by color).

        Args:
            object_ids (list): list of IDs of those objects that will be sorted
        """
        # iterate over all objects in the given list
        for oid in object_ids:
            # update state dict of objects
            self.get_obj_info()

            # skip the iteration if object is already in the correct basket
            if self.obj_info[oid]['status'] == 'basket':
                continue

            # choose the robot arm for grasping based on the position of the object
            if self.obj_info[oid]['pos'][0] >= 0:
                arm1 = 'R'
                arm2 = 'L'
            else:
                arm1 = 'L'
                arm2 = 'R'

            # align the chosen arm with the object
            self.move_to(oid, arm1, 300)

            # try to grasp the object and increase the grasp attempts counter
            grasp = self.grasp(oid, arm1)
            self.obj_info[oid]['grasp_attempts'] += 1

            # if the grasp was successful, continue sorting
            if grasp:
                # move back to initial pose
                self.move_to_init(300)

                # get baskets which are reachable for that arm (from prior knowledge)
                baskets = get_reachable_baskets(arm1)

                # if matching basket in not in the arm's range, do a handover or throw the object
                if self.obj_info[oid]['color'] not in baskets:
                    # for the special sphere which should be thrown do the throwing
                    if self.obj_info[oid]['color'] == 'black':
                        self.throw_ball(oid, arm1, 300)
                        self.obj_info[oid]['status'] = 'basket'
                        self.move_to_init(300)
                        continue
                    # for every other object do a handover
                    else:
                        handover = self.handover(arm1, arm2, oid, 200)
                        # if handover was successful, update state dict
                        if handover:
                            arm2, arm1 = arm1, arm2
                            self.obj_info[oid]['status'] = '{}_hand'.format(arm1)
                        # else move back to initial pose
                        else:
                            self.move_to_init(300)
                            continue

                # move the robot arm with the object in hand to the basket and drop it
                self.move_to_basket(oid, arm1, 300)
                self.drop(arm1, 1000)
                self.obj_info[oid]['status'] = 'basket'

            # move back to initial pose
            self.move_to_init(300)

        # get all objects which are not already sorted to their matching baskets
        status = [i for i in object_ids if self.obj_info[i]['status'] != 'basket']

        # if there are still objects left that are not correctly sorted,
        # check if it was already tried multiple times to sort them
        if status:
            grasps = [self.obj_info[i]['grasp_attempts'] for i in status]
            missing = [s for i, s in enumerate(status) if grasps[i] < 3]
            failed = [s for i, s in enumerate(status) if grasps[i] >= 3]

            # if there are still objects where it was not tried to grasp them three times or more, try again
            if missing:
                self.sort(missing)
            else:
                # display which objects could not be sorted
                if failed:
                    print('Could not sort the following objects: {}'.format(failed))

    def throw_ball(self, object_id, arm, steps, joint_idx=5):
        """Throws ball towards unreachable basket.

        Args:
            object_id (int): ID of object to be thrown
            arm (str): prefix of arm that throws object
            steps (int): number of interpolation steps
            joint_idx (int): index of joint (of that arm) which will be used for throwing
        """
        # get as close to the basket as possible
        self.move_to_basket(object_id, arm, steps)

        # add total number of joints of one arm to the joint index
        # if it is the right arm that throws the object
        if arm == 'R':
            joint_idx += 8

        # get current joint configuration
        self.C.setJointState(self.S.get_q())
        q0 = self.C.getJointState([])

        # calculate a joint configuration where the specified joint
        # rotates back by 135°
        q1 = q0.copy()
        q1[joint_idx] -= np.pi / 4 * 3

        # interpolate in joint space between initial and calculated pose
        self.joint_space_interpolation(q0, q1, 100)

        # calculate a joint configuration where the specified joint
        # rotates forward by 45° (from the initial pose)
        q2 = q0.copy()
        q2[joint_idx] += np.pi / 4

        # interpolate in joint space between current pose and calculated pose
        # using fewer interpolation steps to have a higher velocity and acceleration
        self.joint_space_interpolation(q1, q2, 35, True, arm)

    @staticmethod
    def wait(duration):
        """Stops the execution for the given time (in milliseconds).

        Args:
            duration (int): time in milliseconds
        """
        time.sleep(duration / 1000)
