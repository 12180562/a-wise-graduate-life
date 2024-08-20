#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi, cos, sin, atan2, asin, sqrt, tan
import numpy as np
import rospy

class VO_module:
    """
    This class contains the computations regarding RVO (Receiprocal Velocity Obstacle) to generate the best velocity vector for the ship to avoid the obstacles.

    Attributes:

        min_targetSpeed:
            - Unit: m/s
            - The minimum speed that the ship can have.

        max_targetSpeed:
            - Unit: m/s
            - The maximum speed that the ship can have.

        num_targetSpeedCandidates:
            - Integer
            - The number of the candidates of the ship speed that the ship can have at the moment.
            - It is to generate the reachble range of the velocity candidates

        min_targetHeading_deg_local:
            - Unit: deg.
            - The minimum heading angle that the ship can have at the moment.
            - 0: Global heading angle of the ship
            - +: Counterclockwise in x-y coordinates, clockwise in y-x coordinates.
            - -: Colckwise in x-y coordinates, counterclockwise in y-x coordinates.

        max_targetHeading_deg_local:
            - Unit: deg.
            - The minimum heading angle that the ship can have at the moment.
            - 0: Global heading angle of the ship
            - +: Counterclockwise in x-y coordinates, clockwise in y-x coordinates.
            - -: Colckwise in x-y coordinates, counterclockwise in y-x coordinates.

        num_targetHeadingCandidates:
            - Integer
            - The number of the candidates of the heading angle that the ship can have at the moment.
            - It is to generate the reachble range of the velocity candidates.

        weight_alpha: 
            - Range: 0 < weight_alpha <= 1
            - The weight for the effort that agent A takes to avoid agent B.
            - The lower weight_alpha makes the later start of the avoidance action.
            - >> 0: Nothing action to avoid. Does not consider agent B at all.
            - 1: The same as VO. It considers the agnet B completely.
            - More detail, see the section "IV. RECIPROCAL VELOCITY OBSTACLE" in the paper "Reciprocal Velocity Obstacles for Real-time Multi-Agent Naviagation".

        weight_aggresiveness:
            - Range: 0 < weight_aggresiveness <= 1
            - This weight is activated when there's no avoidance velocities among the velocity candidates.
            - This weight represents the extent how much you want the OS having "avoiding behavior" when there's no collision avoidance velocity.
            - >> 0: Keep the course
            - 1: Give up your course and focus on the most dangerous obstacle.
            - More detail, see the section "IV. RECIPROCAL VELOCITY OBSTACLE" in the paper "Reciprocal Velocity Obstacles for Real-time Multi-Agent Naviagation"
    
    Private methods:

        __is_all_vels_collidable(slef, vel_all_annotated, shipID_all):
            - If all the velocity candiates are in collision cone it returns `True`. Otherwise, it returns `False`.

        __is_all_vels_avoidable(self, vel_all_annotated, shipID_all):
            - If all the velocity candidates are out of the collision cone it returns `True`. Otherwise, it returns `False`.

        __include_mappedTSradius(self, OS, TS):
            - It inserts the mapped radius of the TS (OS radius + TS radius) into the TS dictionary

        __remove_annotation(self, vel_annotated_dict_all)
            - It removes the annotations and takes only velocities from the dictionary of the set of velocities and annotations where the annotation is the indication of going left/right, in time horizon, and going into collision cone.

        __annotate_vels(self, reachableVel_global_all, RVOdata_all):
            - It gives the annotation (in left / in right / in time horizon / in collision cone) for each velocity.

        __take_vels(self, vel_all_annotated, annotation, shipID_all):
            - It takes the velocities that contains specific annotation (in left / in right / in time horizon / in collision cone).
        
        __is_in_between(self, theta_given, theta_left, theta_right):
            - It returns `True` if a given angle is between two specific angles. Otherwise, it returns `False`

        __is_in_left(
            self,  
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global, 
            LOSangle_rad_global,
            ): It returns `True` if the velocity vector is pointing out the left side that will not cause any collision. Otherwise, it returns `False`.

        __is_in_right(
            self, 
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global, 
            LOSangle_rad_global,
            ): It returns `True` if the velocity vector is pointing out the right side that will not cause any collision. Otherwise, it returns `False`.

        __is_within_time_horizon(
            self, 
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global, 
            velVecNorm,
            shortestRelativeDist, 
            timeHorizon,
            ): It returns `True` if the velocity vector is within time horizon. Otherwise, it returns `False`.

        __is_in_collision_cone(
            self, 
            velVecNorm,
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global, 
            shortestRelativeDist, 
            timeHorizon,
            ): It returns `True` if the velocity vector is within collision cone. Otherwise, it returns `False`.

        __generate_vel_candidates(self, targetSpeed_all, targetHeading_rad_global_all):
            - It generates the velocity candidates based on given speeds and heading angles.

        __select_vel_inside_RVOs(self, reachableCollisionVel_global_all, RVOdata_all, V_des):
            - It selects the best velocity based on the RVO formulation when all the velocity candidates are causing collisions. 

        __choose_velocity(self, V_des, RVOdata_all, OS, TS): 
            - It chooses the best velocity for the collision avoidance task.

        __extract_RVO_data(self, OS, TS, static_OB):
            - It generates a container that contains the data for RVO formulation.

    Public methods:

        VO_update(self, OS_original, TS_original, static_OB, V_des):   
            - It computes the velocity selection and returns selected velocity and collision cone information from the current state of the ships.
        
        vectorV_to_goal(self, OS, goal, V_max):
            - It returns the velocity towards the next waypoint regardless of the ship's current state. The velocity is limited by the given maximum speed (Target speed).

    """
    def __init__(self):
        """
        It loads the hyperparameters from `/params/main_parameter.yaml` for the computation in RVO formulation to select the velocity 

        params:

            min_targetSpeed: 
                - Unit: m/s
                - The assumed minimum speed that the ship can have at the moment.
            
            max_targetSpeed: 
                - Unit: m/s
                - The assumed maximum speed that the ship can have at the mement.
            
            num_targetSpeedCandidates:
                - Integer
                - The number of the candidates of the speed that the ship can have at the moment.
                - It is for the generation of a set of velocity candidates.

            min_targetHeading_deg_local:
                - Unit: deg.
                - The assumed minimum heading angle the the ship can have at the moment.
                - 0: Global heading angle of the ship.
                - +: Counterclockwise on x-y coordinates, clockwise on y-x coordinates.
                - -: Clockwise on x-y coordinates, counterclockwise on y-x coordinates.

            max_targetHeading_deg_local:
                - Unit: deg.
                - The assumed maximum heading angle the the ship can have at the moment.
                - 0: Global heading angle of the ship.
                - +: Counterclockwise on x-y coordinates, clockwise on y-x coordinates.
                - -: Clockwise on x-y coordinates, counterclockwise on y-x coordinates.
            
            num_targetHeadingCandidates:
                - Integer
                - The number of candidates of the heading angle that the ship can have at the moment.
                - It is for the generation of a set of velocity candidates.

            weight_alpha:
                - Range: 0 < weight_focusObs <= 1
                - The weight for the effort agent A takes to avoid agent B.
                - The lower alpha makes the later start of the avoidance action.
                - >> 0: Nothing action to avoid. Does not consider agent B at all.
                - 1: The same as VO. Consider the agnet B completely.
                - For more details, see the section "IV. RECIPROCAL VELOCITY OBSTACLE" in the paper "Reciprocal Velocity Obstacles for Real-time Multi-Agent Naviagation".

            weight_aggresivness:
                - Range: 0 < weight_agressivness <= 1
                - The 'weight_agressivness' is the extent how much you want the OS having "avoiding behavior" when there's no collision avoidance velocity.
                - >> 0: Keep the course.
                - 1: Give up your course and focus on the most dangerous obstacle.
                - For more details, see the section "IV. RECIPROCAL VELOCITY OBSTACLE" in the paper "Reciprocal Velocity Obstacles for Real-time Multi-Agent Naviagation".

            timeHorizon:
                - Unit: sec.
                - It allows for the ship going into the VO (Collision cone) as long as the expected collision time is larger than the `timeHorizon`.
                - Range: 0 < timeHorizon
                - >> 0: It removes all the area of the collision cone.
                - inf: It does nothing on the original collision cone. 
                - For More details, see the Eq.(5) in the paper "Motion Planning in Dynamic Environments using Velocity Obstacles".
        """

        # NOTE: It is not clear what min and max of speed could be.
        self.min_targetSpeed = rospy.get_param('min_targetSpeed')
        self.max_targetSpeed = rospy.get_param('max_targetSpeed')
        self.num_targetSpeedCandidates = int(rospy.get_param('num_targetSpeedCandidates'))

        # NOTE: It is not clear what min and max of heading angle could be.
        self.min_targetHeading_deg_local = rospy.get_param('min_targetHeading_deg_local')
        self.max_targetHeading_deg_local = rospy.get_param('max_targetHeading_deg_local')
        self.num_targetHeadingCandidates = int(rospy.get_param('num_targetHeadingCandidates'))

        self.weight_alpha = rospy.get_param('weight_focusObs')  
        self.weight_aggresiveness = rospy.get_param('weight_agressivness')
        self.cri_param = rospy.get_param('cri_param')
        self.time_horizon = rospy.get_param('timeHorizon')
        self.rule = rospy.get_param('Portside_rule')     

        
        
        
    def __is_all_vels_collidable(self, vel_all_annotated, shipID_all):
        """
        If all the velocity candiates are in collision cone it returns `True`. Otherwise, it returns `False`.

        Args:

            vel_all_annotated: 
                - A set of velocities annotated with the velocity's category('inLeft'/'inRight'/'inTimeHorizon'/'inCollisionCone') for each target ship.
                - Example:
                    [
                        {
                            'vel': ndarray([0.76, 0.87])
                            '2001': 'inLeft',
                            '2002': 'inTimeHorizon',
                            ...,
                        },
                        {
                            'vel': ndarray([0.23, 0.46])
                            '2001': 'inRight',
                            '2002': 'inCollisionCone',
                            ...,
                        },
                        ...,
                    ]

            shipID_all: 
                - All of the IDs of the ships to be simulated.
                - Example: [OS, TS1, TS2, ...]

        Return:

            `True`: 
                - If it is not that all the velociteis are causing a collision.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inLeft',
                            '2002': 'inRight',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inCollisionCone',
                            '2002': 'inCollisionCone',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inTimeHorizon',
                            '2002': 'inCollisionCone',
                        },
                    ]

            `False`: 
                - If all the velociteis are causing a collision.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inCollisionCone',
                            '2002': 'inRight',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inCollisionCone',
                            '2002': 'inCollisionCone',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inTimeHorizon',
                            '2002': 'inCollisionCone',
                        },
                    ]
        """
        for vel_annotated in vel_all_annotated:
            isVelCollidable = False
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   isVelCollidable = True
                   break
            if not isVelCollidable:
                return False

        return True

    def __is_all_vels_avoidable(self, vel_all_annotated, shipID_all):
        """
        If all the velocity candiates are out of the collision cone it returns `True`. Otherwise, it returns `False`.

        Args:

            vel_all_annotated: 
                - A set of velocities annotated with the velocity's category('inLeft'/'inRight'/'inTimeHorizon'/'inCollisionCone) for each other ship.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx])
                            '2001': 'inLeft',
                            '2002': 'inTimeHorizon',
                            ...,
                        },
                        {
                            'vel': ndarray([xx, xx])
                            '2001': 'inRight',
                            '2002': 'inCollisionCone',
                            ...,
                        },
                        ...,
                    ]

            shipID_all: 
                - All of the IDs of the ships to be simulated.
                - Example: [OS, TS1, TS2, ...]

        Return:

            `True`:
                - If all the velociteis are not causing a collision.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inLeft',
                            '2002': 'inRight',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inRight',
                            '2002': 'inLeft',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inTimeHorizon',
                            '2002': 'inLeft',
                        },
                    ]

            `False`:
                - If it has at least one the velocity causing a collision.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inCollisionCone',
                            '2002': 'inRight',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inRight',
                            '2002': 'inLeft',
                        },
                        {
                            'vel': ndarray([xx, xx]),
                            '2001': 'inTimeHorizon',
                            '2002': 'inLeft',
                        },
                    ]
        """
        for vel_annotated in vel_all_annotated:
            for shipID in shipID_all:
                if (vel_annotated[shipID] == 'inCollisionCone'):
                   return False

        return True

    
    def __remove_annotation(self, vel_annotated_all):
        """
        It removes the annotations and takes only velocities from the dictionary of the set of velocities and annotations where the annotation is the indication of going left/right, in time horizon, and going into collision cone.

        Example: 

            From: 
                [
                    {
                        'vel': array([0.20, 0.02]), 
                        2001: 'inRight', 
                        2002: 'inLeft,
                    },
                    {
                        'vel': array([0.18, 0.12]), 
                        2001: 'inLeft', 
                        2002: 'inRight,
                    },
                    ...,
                ]
            To:
                [
                    array([0.20, 0.02]),
                    array([0.18, 0.12]),
                    ...,
                ]

        Args:

            vel_annotated_dict_all: 
                - A list of annoated velocities
                - Example:
                    [
                        {
                            'vel': array([0.20, 0.02]), 
                            2001: 'inRight', 
                            2002: 'inLeft,
                        },
                        {
                            'vel': array([0.18, 0.12]), 
                            2001: 'inLeft', 
                            2002: 'inRight,
                        },
                        ...,
                    ]

        Returns:

            vels:
                - A list of velocities that does not incldue the annotations.
                - Example:
                    [
                        array([0.20, 0.02]),
                        array([0.18, 0.12]),
                        ...,
                    ]
        """
        vels = []
        
        for vel_annotated in vel_annotated_all:
            vels.append(vel_annotated['vel'])
            
        return vels

    def __annotate_vels(self, reachableVel_global_all, RVOdata_all, TS):
        """
        It gives the annotation (in left / in right / in time horizon / in collision cone) for each velocity.

        Example:

            From:
                [
                    ndarray([0.82, 0.77]),
                    ndarray([0.56, 0.45]),
                    ...,
                ]
            To:print
                [
                    {
                        'vel': ndarray([0.82, 0.77]),
                        2001: 'inLeft',
                        2002: 'inTimeHorizon',
                        ...,
                    },
                    {
                        'vel': ndarray([0.56, 0.45]),
                        2001: 'inRight',
                        2002: 'inCollisionCone',
                        ...,
                    }
                ]
            where 2001, 2002, ... are the ship IDs.

        Args:

            reachableVel_global_all: 
                - A set of reachable velocities in global coordinates
                - Example:
                    [
                        ndarray([0.82, 0.77]),
                        ndarray([0.56, 0.45]),
                        ...,
                    ]
            
            RVOdata_all: 
                - A set of dictionaries that contain the information to calculate RVO
                - Example:
                    [
                        {
                            "TS_ID": 2001,
                            "LOSdist": 16.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.85, 1.15]),
                            "vB": ndarray([-0.12, 0.52]),
                            "boundLineAngle_left_rad_global: 0.24", 
                            "boundLineAngle_right_rad_global: 0.31", 
                            "collisionConeShifted_local: ndarray([0.12, 0.32])",
                        },
                        {
                            "TS_ID": 2002,
                            "LOSdist": 12.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.25, 1.25]),
                            "vB": ndarray([-0.22, 0.42]),
                            "boundLineAngle_left_rad_global: 0.21", 
                            "boundLineAngle_right_rad_global: 0.35", 
                            "collisionConeShifted_local: ndarray([0.32, 0.12])",
                        },
                        ...
                    ]

        Returns:

            reachableVel_global_all_annotated:
                - A set of annotated velocities by 'in left' / 'in right' / 'in time horizon' / 'in collision cone'.
                - Example:
                    [
                        {
                            'vel': ndarray([0.82, 0.77]),
                            2001: 'inLeft',
                            2002: 'inTimeHorizon',
                            ...,
                        },
                        {
                            'vel': ndarray([0.56, 0.45]),
                            2001: 'inRight',
                            2002: 'inCollisionCone',
                            ...,
                        }
                    ]
        """
        # Make the `reachableVel_global_all` dictionary
        reachableVel_global_all_annotated = []

        for reachableVel_global in reachableVel_global_all:
            
            reachableVel_global_annotated = {'vel': reachableVel_global}

            for RVOdata in RVOdata_all:
                '''
                `RVOdata` structure:
                    RVOdata = {
                        "TS_ID",
                        "LOSdist", 
                        "mapped_radius", 
                        "vA",
                        "vB",
                        "boundLineAngle_left_rad_global", 
                        "boundLineAngle_right_rad_global", 
                        "collisionConeShifted_local",
                        "CRI"
                        }
                '''
                # NOTE: vA2B_RVO is the relative velocity from the agent A to B
                #       on the RVO configuration space, not the VO configuration space
                vA2B_RVO = reachableVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )
                
                if self.__is_in_left(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inLeft'
                
                elif self.__is_in_right(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'],
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    LOSangle_rad_global=0.5*(RVOdata['boundLineAngle_left_rad_global']+RVOdata['boundLineAngle_right_rad_global']),
                    ):

                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inRight'
                
                elif self.__is_within_time_horizon(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):
                    # print("is within timehorizon",RVOdata['CRI']*self.cri_param)
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inTimeHorizon'
                
                elif self.__is_in_collision_cone(
                    velVecAngle_rad_global=angle_vA2B_RVO_rad_global, 
                    boundLineAngle_left_rad_global=RVOdata['boundLineAngle_left_rad_global'], 
                    boundLineAngle_right_rad_global=RVOdata['boundLineAngle_right_rad_global'],
                    velVecNorm=np.linalg.norm(vA2B_RVO),
                    shortestRelativeDist=RVOdata['LOSdist']-RVOdata['mapped_radius'],
                    timeHorizon=RVOdata['CRI']*self.cri_param,
                    # timeHorizon=self.time_horizon
                    ):
                    # print('is in collision cone',RVOdata['CRI']*self.cri_param)
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

                else:
                    reachableVel_global_annotated[RVOdata['TS_ID']] = 'inCollisionCone'

            reachableVel_global_all_annotated.append(reachableVel_global_annotated)

        return reachableVel_global_all_annotated

    def __take_vels(self, vel_all_annotated, annotation, shipID_all):
        """
        It takes the velocities that contains specific annotation (in left / in right / in time horizon / in collision cone) given by the argument. 
        
        The categoriese that the argument `annotation` can take is:
            (1) 'inLeft'
            (2) 'inRight'
            (3) 'inTimeHorizon'
            (4) 'inCollisionCone'

        Example:
            Assuming the argument `annotation` is given as ['inLeft', 'inRight']. Then, it will convert the set of velocities:
                From (`vel_all_annotated`): 
                    [
                        {
                            'vel': ndarray([0.82, 0.77]),
                            2001: 'inLeft',
                            2002: 'inTimeHorizon',
                        },
                        {
                            'vel': ndarray([0.56, 0.45]),
                            2001: 'inRight',
                            2002: 'inCollisionCone',
                        },
                        {
                            'vel': ndarray([0.86, 0.35]),
                            2001: 'inTimeHorizon',
                            2002: 'inCollisionCone',
                        }
                    ]
                To (`vel_hasAnnotation_all_annotated`):
                    [
                        ndarray([0.82, 0.77]),
                        ndarray([0.56, 0.45]),
                        ...,
                    ]
                where 2001, 2002, ... are the ship IDs.

        Args:

            vel_all_annotated:
                - A set of velocities annotated with the velocity's category('inLeft'/'inRight'/'inTimeHorizon'/'inCollisionCone') for each target ship.
                - Example:
                    [
                        {
                            'vel': ndarray([xx, xx])
                            '2001': 'inLeft',
                            '2002': 'inTimeHorizon',
                            ...,
                        },
                        {
                            'vel': ndarray([xx, xx])
                            '2001': 'inRight',
                            '2002': 'inCollisionCone',
                            ...,
                        },
                        ...,
                    ]

            annotation:
                - A list of strings that includes the velocity categories to be taken.
                - The categoriese that the argument `annotation` can take is:
                    (1) 'inLeft'
                    (2) 'inRight'
                    (3) 'inTimeHorizon'
                    (4) 'inCollisionCone'
                - Example: ['inLeft', 'inTimeHorizon']

            shipID_all:
                - All of the IDs of the ships to be simulated.
                - Example: [OS, TS1, TS2, ...]
        """
        vel_hasAnnotation_all_annotated = []
        for vel_annotated in vel_all_annotated:
            hasAnnotation = True
            for shipID in shipID_all:
                if vel_annotated[shipID] not in annotation:
                    hasAnnotation = False
                    break
            if hasAnnotation:
                vel_hasAnnotation_all_annotated.append(vel_annotated)

        return vel_hasAnnotation_all_annotated

    def __is_in_between(self, theta_given, theta_left, theta_right):
        """
        It returns `True` if a given angle is between two given angles. Otherwise, it returns `False`.


        Args:
            
            theta_given:
                - Unit: rad.
                - The given global angle to be evaluated

            theta_left:
                - Unit: rad.
                - The global angle of the left boundary line of the collision cone.

            theta_right:
                - Unit: rad.
                - The global angle of the right boundary line of the collision cone.

        Returns:
            `True`: If the given angle is between the angles of the left and right boundary lines.

            `False`: If the given angle is NOT between the angles of the left and right boundary lines.
        """
        if abs(theta_right - theta_left) <= pi:
            if theta_left < 0 and theta_right < 0:
                if theta_given > 0:
                    theta_given -= 2*pi
            elif theta_left > 0 and theta_right > 0:
                if theta_given < 0:
                    theta_given += 2*pi
            if theta_right <= theta_given <= theta_left:    
                return True
            else :
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                ## 각도 보정 
                theta_left += 2*pi
                if theta_given <0:
                    theta_given += 2*pi

                if theta_right <= theta_given <= theta_left:
                    return True
                else :
                    return False
            
            if (theta_left > 0) and (theta_right <0):
                theta_right += 2*pi            
                if theta_given < 0:
                    theta_given += 2*pi
                    
                if theta_left <= theta_given <= theta_right:
                    return True
                else:
                    return False
        # print(theta_left, theta_right, theta_given)

    def __is_in_left(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        """
        It returns `True` if the velocity vector is pointing out the left side that will not cause any collision. Otherwise, it returns `False`.

        Args:

            velVecAngle_rad_global:
                - Unit: rad.
                - The global angle of the velocity vector.

            boundLIneAngle_left_rad_global:
                - Unit: rad.
                - The global angle of the left boundary line of the collision cone on x-y coordinate.

            boundLIneAngle_right_rad_global:
                - Unit: rad.
                - The global angle of the right boundary line of the collision cone on x-y coordinate.

            LOSangle_rad_global:
                - Unit: rad.
                - The global angle of the LOS (Line of sight) on x-y coordinate

        Returns:

            `True`: If the velocity vector is pointing out the left side that will not cause any collision.

            `False`: If the velocity vector is pointing out towards the obstacle that will cause a collsion.
        """
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return True
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return False

    def __is_in_right(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        LOSangle_rad_global,
        ):
        """
        It returns `True` if the velocity vector is pointing out the right side that will not cause any collision. Otherwise, it returns `False`.

        Args:

            velVecAngle_rad_global:
                - Unit: rad.
                - The global angle of the velocity vector.

            boundLIneAngle_left_rad_global:
                - Unit: rad.
                - The global angle of the left boundary line of the collision cone on x-y coordinate.

            boundLIneAngle_right_rad_global:
                - Unit: rad.
                - The global angle of the right boundary line of the collision cone on x-y coordinate.

            LOSangle_rad_global:
                - Unit: rad.
                - The global angle of the LOS (Line of sight)

        Returns:

            `True`: If the velocity vector is pointing out the right side that will not cause any collision.

            `False`: If the velocity vector is pointing out towards the obstacle that will cause a collsion.
        """
        velVecAngle_translated_rad_global = velVecAngle_rad_global - LOSangle_rad_global
        while velVecAngle_translated_rad_global < 0:
            velVecAngle_translated_rad_global += 2*pi
        velVecAngle_translated_rad_global %= (2*pi)

        if self.__is_in_between(
            velVecAngle_rad_global, 
            boundLineAngle_left_rad_global, 
            boundLineAngle_right_rad_global,
            ):
            return False
        elif (0 < velVecAngle_translated_rad_global <= pi):
            return False
        elif (pi < velVecAngle_translated_rad_global <= 2*pi):
            return True

    def __is_within_time_horizon(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        """
        It returns `True` if the velocity vector is within time horizon. Otherwise, it returns `False`.

        Args:

            velVecAngle_rad_global:
                - Unit: rad.
                - The global angle of the velocity vector.

            boundLIneAngle_left_rad_global:
                - Unit: rad.
                - The global angle of the left boundary line of the collision cone on x-y coordinate.

            boundLIneAngle_right_rad_global:
                - Unit: rad.
                - The global angle of the right boundary line of the collision cone on x-y coordinate.

            velVecNorm:
                - Unit: m/s
                - The magnitude of the velocity vector (a.k.a. speed)

            shortestRelatveDist:
                - Unit: m
                - The shortest distance to the obstacle.

            timeHorizon:
                - Unit: sec.
                - The time horizon which is the parameter that how much you will allow the ship to go inside the collision cone.
                - It will allow the velocities that are less than 'shortestRelativeDist/timeHorizon' even if the velocity vectors are in collision cone.
                - It implies that, "if the velocity does not cause a collision within the time horizon, this velocity is allowable."
                - For More details, see the Eq.(5) in the paper "Motion Planning in Dynamic Environments using Velocity Obstacles".

        Returns:

            `True`: If the velocity vector is within time horizon.

            `False`: If the velocity vector is out of the time horizon.
        """
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm <= shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __is_in_collision_cone(
        self, 
        velVecAngle_rad_global, 
        boundLineAngle_left_rad_global, 
        boundLineAngle_right_rad_global, 
        velVecNorm,
        shortestRelativeDist, 
        timeHorizon,
        ):
        """
        It returns `True` if the velocity vector is in collision cone. Otherwise, it returns `False`.

        Args:

            velVecAngle_rad_global:
                - Unit: rad.
                - The global angle of the velocity vector.

            boundLIneAngle_left_rad_global:
                - Unit: rad.
                - The global angle of the left boundary line of the collision cone on x-y coordinate.

            boundLIneAngle_right_rad_global:
                - Unit: rad.
                - The global angle of the right boundary line of the collision cone on x-y coordinate.

            velVecNorm:
                - Unit: m/s
                - The magnitude of the velocity vector (a.k.a. speed)

            shortestRelatveDist:
                - Unit: m
                - The shortest distance to the obstacle.

            timeHorizon:
                - Unit: sec.
                - The time horizon which is the parameter that how much you will allow the ship to go inside the collision cone.
                - It will allow the velocities that are less than 'shortestRelativeDist/timeHorizon' even if the velocity vectors are in collision cone.
                - It implies that, "if the velocity does not cause a collision within the time horizon, this velocity is allowable."

        Returns:

            `True`: If the velocity vector is in collision cone.

            `False`: If the velocity vector is out of the collision cone.
        """
        if self.__is_in_between(
            velVecAngle_rad_global,
            boundLineAngle_left_rad_global,
            boundLineAngle_right_rad_global,
            ) and (velVecNorm > shortestRelativeDist / timeHorizon):
            return True
        else:
            return False

    def __generate_vel_candidates(self, targetSpeed_all, targetHeading_rad_global_all, OS, static_obstacle_info, static_point_info):
        """
        It generates the velocity candidates based on given speeds and heading angles.

        Args:

            targetSpeed_all: A set of the speed that the ship can have

            targetHeading_rad_global_all: A set of the heading angles that the ship can have

        Returns:

            reachableVel_global_all: A set of velocities that the ship can have.

        """
        reachableVelX_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        reachableVelY_global_all = np.zeros(self.num_targetSpeedCandidates * self.num_targetHeadingCandidates)
        
        idx = int(0)
        for targetHeading_rad_global in targetHeading_rad_global_all:
            for targetSpeed in targetSpeed_all:

                reachableVelX_global_all[idx] = targetSpeed * cos(targetHeading_rad_global)
                reachableVelY_global_all[idx] = targetSpeed * sin(targetHeading_rad_global)

                idx += 1

        reachableVelX_global_all = np.reshape(reachableVelX_global_all, newshape=(-1, 1))
        reachableVelY_global_all = np.reshape(reachableVelY_global_all, newshape=(-1, 1))

        reachableVel_global_all = np.concatenate(
            (reachableVelX_global_all, reachableVelY_global_all),
             axis=-1,
             )
        
        """
        reachableVel_global_all_after_obstacle : The candidate of reachable vector that dosen't cross static obstacle.
        
        __delete_vector_inside_obstacle : The function that judge collision with static obstacle.

        """

        reachableVel_global_all_after_obstacle = self.__delete_vector_inside_obstacle(reachableVel_global_all, OS, static_obstacle_info,static_point_info)
        # print("number of vector:",len(reachableVel_global_all_after_obstacle))

        return reachableVel_global_all_after_obstacle


    # reachable velocity global all을 받아와서 detecting vector를 생성
    # detecting vector가 장애물과 겹친다면 후보에서 삭제

    def __delete_vector_inside_obstacle(self, reachableVel_global_all, OS, static_obstacle_info, static_point_info):

        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        reachableVel_global_all_copy = np.copy(reachableVel_global_all)
        static_OB_data = static_obstacle_info
        static_point_data = static_point_info
        
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])
        delta_t = rospy.get_param("delta_t") # constant
        detecting_radious = rospy.get_param("detecting_radious")

        #initial number for while
        obstacle_number = 0
        point_number = 0

        # obstacle_radious
        radious = 5

        while (obstacle_number) != len(static_OB_data):
            obstacle_point_x = [static_OB_data[obstacle_number],static_OB_data[obstacle_number+2]] 
            obstacle_point_y = [static_OB_data[obstacle_number+1],static_OB_data[obstacle_number+3]] 

            if obstacle_point_x[0] > obstacle_point_x[1]:
                obstacle_point_x.reverse()
                obstacle_point_y.reverse()

            if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                slope = 9999

            elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                slope =-9999

            else: 
                slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])
                
            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]
                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt(slope, 
                        vector_slope,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y):

                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)
                else:
                    pass

            reachableVel_global_all = np.array(result)

            obstacle_number = obstacle_number+4

        while point_number != len(static_point_data):
        
            point_x = static_point_data[point_number]
            point_y = static_point_data[point_number+1]

            reachableVel_global_all_after_delta_t = reachableVel_global_all * delta_t 
            result = []

            for i in range(len(reachableVel_global_all_after_delta_t)-1): 
                after_delta_t_x = reachableVel_global_all_after_delta_t[i][0]+pA[0]
                after_delta_t_y = reachableVel_global_all_after_delta_t[i][1]+pA[1]

                if (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) < 0:
                    vector_slope = 9999

                elif (pA[0]-after_delta_t_x) == 0 and (pA[1]-after_delta_t_y) > 0:
                    vector_slope = -9999
                else:
                    vector_slope = (pA[1]-after_delta_t_y)/(pA[0]-after_delta_t_x)

                if self.get_crosspt_circle(vector_slope,
                        point_x, 
                        point_y, 
                        radious, 
                        pA[0], 
                        pA[1], 
                        after_delta_t_x, 
                        after_delta_t_y ):
                    
                    component = reachableVel_global_all[i]
                    component_list = component.tolist()
                    result.append(component_list)

                else:
                    pass

            reachableVel_global_all = np.array(result)

            point_number = point_number+2

        if len(reachableVel_global_all) == 0:
            reachableVel_global_all = reachableVel_global_all = np.array([reachableVel_global_all_copy[-1,:]])
        else:
            pass

        return reachableVel_global_all
    
    def if_all_vector_collidable(self, OS, effective_static_OB, detecting_radious, reachableVel_global_all_copy):
        space_number = rospy.get_param("space_number")
        pA = [OS['Pos_X'], OS['Pos_Y']]
        vector_radian_plus = 0
        vector_radian_minus = 0
        vector_space_radian = pi/space_number
        detecting_vector_list = []
        detecting_vector_list_in_right = []
        detecting_vector_list_in_left = []
        for i in range(space_number):
            vector_radian_plus = vector_radian_plus + vector_space_radian * i
            vector_radian_minus = vector_radian_minus - vector_space_radian * i
            vector_point_plus = [(pA[0]+detecting_radious*cos(vector_radian_plus)),(pA[0]+detecting_radious*sin(vector_radian_plus))]
            vector_point_minus = [(pA[0]+detecting_radious*cos(vector_radian_minus)),(pA[0]+detecting_radious*sin(vector_radian_minus))]
            vector_slope_plus = tan(vector_radian_plus)
            vector_slope_minus = tan(vector_radian_minus)
            obstacle_number = 0

            while (obstacle_number) != len(effective_static_OB):
                obstacle_point_x = [effective_static_OB[obstacle_number],effective_static_OB[obstacle_number+2]] 
                obstacle_point_y = [effective_static_OB[obstacle_number+1],effective_static_OB[obstacle_number+3]] 

                if obstacle_point_x[0] > obstacle_point_x[1]:
                    obstacle_point_x.reverse()
                    obstacle_point_y.reverse()

                if (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) > 0:
                    slope = 9999

                elif (obstacle_point_x[0]-obstacle_point_x[1]) == 0 and (obstacle_point_y[0]-obstacle_point_y[1]) < 0:
                    slope =-9999

                else: 
                    slope = (obstacle_point_y[1]-obstacle_point_y[0])/(obstacle_point_x[1]-obstacle_point_x[0])

                if self.get_crosspt(slope, 
                        vector_slope_plus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_plus[0], 
                        vector_point_plus[1]):

                    detecting_vector_list_in_right.append(vector_radian_plus)

                elif self.get_crosspt(slope, 
                        vector_slope_minus,
                        obstacle_point_x[0], 
                        obstacle_point_y[0],
                        obstacle_point_x[1], 
                        obstacle_point_y[1],
                        pA[0], 
                        pA[1], 
                        vector_point_minus[0], 
                        vector_point_minus[1]):

                    detecting_vector_list_in_left.append(vector_radian_minus)
                else:
                    pass

            if len(detecting_vector_list) == 0:
                # print("all the vector is collidable")
                reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])

            else:
                if len(detecting_vector_list_in_right) >= len(detecting_vector_list_in_left):
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[182,:]])
                    # select vector that is in right
                else:
                    reachableVel_global_all = np.array([reachableVel_global_all_copy[123,:]])
                    # select vector that is in left

            return reachableVel_global_all

    def get_crosspt(self, slope, vector_slope, start_x, start_y,end_x, end_y, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y):

        x_point = [start_x, end_x]
        y_point = [start_y, end_y]

        if (slope) == (vector_slope): 
            return True

        else:
            cross_x = (start_x * slope - start_y - OS_pos_x * vector_slope + OS_pos_y) / (slope - vector_slope)
            cross_y = slope * (cross_x - start_x) + start_y

            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and OS_pos_y <= cross_y <= after_delta_t_y:

                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if OS_pos_x <= cross_x <= after_delta_t_x and  after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (min(x_point)-5) <= cross_x <= (max(x_point)+5) and (min(y_point)-5) <= cross_y <= (max(y_point)+5):
                    if after_delta_t_x <= cross_x <= OS_pos_x and after_delta_t_y <= cross_y <= OS_pos_y:

                        return False
                    else:
                        return True
                else:
                    return True
                
    def get_crosspt_circle(self, vector_slope, point_x, point_y, radious, OS_pos_x, OS_pos_y, after_delta_t_x, after_delta_t_y ):

        point = [point_x,point_y]
        r = radious

        vector_point_x = [OS_pos_x,after_delta_t_x]
        vector_point_y = [OS_pos_y,after_delta_t_y]

        slope = vector_slope

        a = slope
        b = -1
        c = -slope*vector_point_x[0]+vector_point_y[0]

        xp = point[0]
        yp = point[1]

        A = 1+(a**2/b**2)
        B = (-2*xp)+(2*a*c/b**2)+(2*a*yp/b)
        C = (xp**2)+(c**2/b**2)+(2*c*yp/b)+(yp**2)-(r**2)

        discriminant = (B**2)-(4*A*C)

        if discriminant >= 0:
            cross_x_1 = (-B+sqrt(discriminant))/(2*A)
            cross_y_1 = slope*(cross_x_1-vector_point_x[0])+vector_point_y[0]


            if OS_pos_x <= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x >= after_delta_t_x and OS_pos_y <= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and OS_pos_y <= cross_y_1 <= after_delta_t_y:
                        return False
                    else:
                        return True
                else:
                    return True

            elif OS_pos_x <= after_delta_t_x and OS_pos_y >= after_delta_t_y:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if OS_pos_x <= cross_x_1 <= after_delta_t_x and  after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

            else:
                if (xp-r) <= cross_x_1 <= (xp+r) and (yp-r) <= cross_y_1 <= (yp+r):
                    if after_delta_t_x <= cross_x_1 <= OS_pos_x and after_delta_t_y <= cross_y_1 <= OS_pos_y:
                        return False
                    else:
                        return True
                else:
                    return True

        else:
            return True


    def __select_vel_inside_RVOs(self, reachableCollisionVel_global_all, RVOdata_all, V_des):
        """
        It selects the best velocity based on the RVO formulation when all the velocity candidates are causing collisions 

        Args: 
            
            reachableCollisionVel_global_all:
                - A set of the velocities that will cause a collision
                - Example:
                    [
                        ndarray([0.82, 0.77]),
                        ndarray([0.56, 0.45]),
                        ...,
                    ]

            RVOdata_all:
                - A set of dictionaries that contain the information to calculate RVO
                - Example:
                    [
                        {
                            "TS_ID": 2001,
                            "LOSdist": 16.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.85, 1.15]),
                            "vB": ndarray([-0.12, 0.52]),
                            "boundLineAngle_left_rad_global: 0.24", 
                            "boundLineAngle_right_rad_global: 0.31", 
                            "collisionConeShifted_local: ndarray([0.12, 0.32])",
                        },
                        {
                            "TS_ID": 2002,
                            "LOSdist": 12.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.25, 1.25]),
                            "vB": ndarray([-0.22, 0.42]),
                            "boundLineAngle_left_rad_global: 0.21", 
                            "boundLineAngle_right_rad_global: 0.35", 
                            "collisionConeShifted_local: ndarray([0.32, 0.12])",
                        },
                        ...
                    ]

            V_des:
                - Unit: m/s
                - The desired velocity pointing out the next waypoint.RVOdata_all

        Returns:

            vA_post: 
                - Unit: m/s
                - Selected velocity by RVO formulation.
        """
        # Compute the minimum time to collision(tc) for every velocity candidates
        velCandidates_dict = dict()

        for reachableCollisionVel_global in reachableCollisionVel_global_all:
            
            velCandidates_dict[tuple(reachableCollisionVel_global)] = dict()
            tc_min = 99999.9 # initialize the time to collision to a large enough

            # Compute the minimum time to collision(tc) for a velocity candidate
            for RVOdata in RVOdata_all:
                '''
                Data structure of the `RVO_data`:
                    {
                        "TS_ID": 2001,
                        "LOSdist": 16.29, 
                        "mapped_radius": 16.0, 
                        "vA": ndarray([0.85, 1.15]),
                        "vB": ndarray([-0.12, 0.52]),
                        "boundLineAngle_left_rad_global: 0.24", 
                        "boundLineAngle_right_rad_global: 0.31", 
                        "collisionConeShifted_local: ndarray([0.12, 0.32])",
                    }
                '''

                vA2B_RVO = reachableCollisionVel_global - RVOdata['collisionConeTranslated']

                angle_vA2B_RVO_rad_global = atan2(
                    vA2B_RVO[1],
                    vA2B_RVO[0],
                    )

                # Consider only collidable agent.
                # NOTE: tc will maintain a large initial value if no collision 
                #       for the corresponding agent.
                if self.__is_in_between(
                RVOdata['boundLineAngle_right_rad_global'],
                angle_vA2B_RVO_rad_global, 
                RVOdata['boundLineAngle_left_rad_global'],
                ):

                    # NOTE: The angle between
                    #       (1) vA2B (on RVO config. space) 
                    #       (2) LOS line and
                    angle_at_pA = abs(angle_vA2B_RVO_rad_global - 0.5 * (RVOdata['boundLineAngle_right_rad_global'] + RVOdata['boundLineAngle_left_rad_global']))   
                    angle_at_pA %= (2*pi)
                    if angle_at_pA > pi:
                        angle_at_pA = abs(angle_at_pA - (2*pi))                
                    
                    # NOTE: The angle between 
                    #       (1) vA2B(on RVO config. space) and 
                    #       (2) the line from the collision point on the B_hat surface 
                    #       to the center of B_hat
                    # TODO: Velocity vector here must be directed to the B_hat, 
                    #       but some are to out of the B_hat. Review it. 
                    #       Now force the length `abs(RVOdata['LOSdist'] * sin(angle_at_pA))`
                    #       to be less than `RVOdata['mapped_radius']` temporarily.
                    if (abs(RVOdata['LOSdist'] * sin(angle_at_pA)) > RVOdata['mapped_radius']):
                        angle_at_collisionPoint = 0.0    
                    else:
                        angle_at_collisionPoint = asin(
                            abs(RVOdata['LOSdist'] * sin(angle_at_pA)) / RVOdata['mapped_radius']
                            )


                    # NOTE: The distance between pA and the surface of B_hat on the
                    #       RVO config. space
                    dist_collision = abs(RVOdata['LOSdist'] * cos(angle_at_pA)) - abs(RVOdata['mapped_radius'] * cos(angle_at_collisionPoint))

                    # NOTE: If collision already occured, 
                    #       tc(time to collision) will be zero.
                    if dist_collision < 0: dist_collision = 0
                    tc = dist_collision / np.linalg.norm([vA2B_RVO])
                    
                    # NOTE: Avoid zero division for the penalty calculation
                    if tc < 0.00001: tc = 0.00001

                    if tc < tc_min: tc_min = tc

            # Store the minimum time to collision for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['tc_min'] = tc_min

            # Comput and store the penalty for each velocity candidate
            velCandidates_dict[tuple(reachableCollisionVel_global)]['penalty'] = self.weight_aggresiveness / tc_min + np.linalg.norm([V_des - reachableCollisionVel_global])
           

        # Take the velocity that has the minimum penalty
        vA_post = min(velCandidates_dict, key=lambda k : velCandidates_dict[k]['penalty'])
        # print(reachableCollisionVel_global_all)
        vA_post = reachableCollisionVel_global_all[-1]
        return vA_post

    def __choose_velocity(self, V_des, RVOdata_all, OS, TS, static_obstacle_info, static_point_info): 
        """ 
        It chooses the best velocity for the collision avoidance task.

        Args:

            V_des:
                - Unit: m/s
                - The desired velocity pointing out the next waypoint.

            RVOdata_all:
                - A set of dictionaries that contain the information to calculate RVO
                - Example:
                    [
                        {
                            "TS_ID": 2001,
                            "LOSdist": 16.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.85, 1.15]),
                            "vB": ndarray([-0.12, 0.52]),
                            "boundLineAngle_left_rad_global: 0.24", 
                            "boundLineAngle_right_rad_global: 0.31", 
                            "collisionConeShifted_local: ndarray([0.12, 0.32])",
                        },
                        {
                            "TS_ID": 2002,
                            "LOSdist": 12.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.25, 1.25]),
                            "vB": ndarray([-0.22, 0.42]),
                            "boundLineAngle_left_rad_global: 0.21", 
                            "boundLineAngle_right_rad_global: 0.35", 
                            "collisionConeShifted_local: ndarray([0.32, 0.12])",
                        },
                        ...
                    ]

            OS: 
                - State of the OS
                - Example:
                    {
                        'Ship_ID': 2000,
                        'Pos_X': -49.99,
                        ...,
                        'radius': 8.0,
                    }

            TS: 
                - State of the TSs
                - Example:
                    {
                        2001:{
                            'Ship_ID': '2001',
                            ...,
                            'radius': 4.0,
                            },
                        2002:{
                            'Ship_ID': '2002',
                            ...,
                            'radius': 4.0,
                            },
                        ...,
                    }

        Returns:

            vA_post:
                - Unit: m/s
                - Selected velocity by RVO formulation.
        """
        # Generate target speed candidates
        # NOTE: We generate the target velocity candidates manually, not deriving from mmg and feasible acc.
        targetSpeed_all = np.linspace(
            start=self.min_targetSpeed, 
            stop=self.max_targetSpeed, 
            num=self.num_targetSpeedCandidates,
            )
        
        if TS == None:
            min_targetHeading_rad_local = np.deg2rad(self.min_targetHeading_deg_local)
            max_targetHeading_rad_local = np.deg2rad(self.max_targetHeading_deg_local)
            heading_rad = np.deg2rad(OS['Heading'])

            min_targetHeading_rad_global = heading_rad + min_targetHeading_rad_local
            max_targetHeading_rad_global = heading_rad + max_targetHeading_rad_local

            targetHeading_rad_global_all = np.linspace(
                start=min_targetHeading_rad_global, 
                stop=max_targetHeading_rad_global, 
                num=self.num_targetHeadingCandidates,
                )
            # Generate target velocity vector candidates
            reachableVel_global_all = self.__generate_vel_candidates(
                targetSpeed_all, 
                targetHeading_rad_global_all,
                OS,
                static_obstacle_info,
                static_point_info
                )
            
            vA_post = min(
                    reachableVel_global_all,
                    key= lambda v: np.linalg.norm(v - V_des),
                    )

        else:
            TS_ID = TS.keys()

            nearest_status = "starboard"
            nearest_DCPA = 100000

            for ts_ID in TS_ID:
                status = TS[ts_ID]['status']
                DCPA = TS[ts_ID]['DCPA']
                if DCPA <= nearest_DCPA:
                    nearest_DCPA = DCPA
                    nearest_ts_ID = ts_ID
                    nearest_status = status
                else:
                    pass

            

            # Generate target heading angle candidates
            min_targetHeading_rad_local = np.deg2rad(self.min_targetHeading_deg_local)
            max_targetHeading_rad_local = np.deg2rad(self.max_targetHeading_deg_local)
            heading_rad = np.deg2rad(OS['Heading'])

            min_targetHeading_rad_global = heading_rad + min_targetHeading_rad_local
            max_targetHeading_rad_global = heading_rad + max_targetHeading_rad_local

            targetHeading_rad_global_all = np.linspace(
                start=min_targetHeading_rad_global,
                stop=max_targetHeading_rad_global,
                num=self.num_targetHeadingCandidates,
                )

            # Generate target velocity vector candidates
            reachableVel_global_all = self.__generate_vel_candidates(
                targetSpeed_all,
                targetHeading_rad_global_all,
                OS,
                static_obstacle_info,
                static_point_info
                )

            # Annotate the velocities - 'in time horizon', 'in left', 'in right', 'in collision cone'
            reachableVel_all_annotated = self.__annotate_vels(
                reachableVel_global_all,
                RVOdata_all,
                TS,
                )

            '''
            Data structure of `vels_annotated`:
                [
                    {
                        'vel': array([xx, xx]), 
                        2001: 'inTimeHorizon', 
                        2002: 'inLeft', 
                        2003: 'inRight'
                        ...
                    }
                    {
                        'vel': array([xx, xx]), 
                        2001: 'inCollision', 
                        2002: 'inTimeHorizon', 
                        2003: 'inRight'
                        ...
                    }
                    ...
                ]
                where 2001, 2002, 2003, ... are the ship IDs
            '''

            # NOTE: Our annotation is based on VO config. space which is x-y coord.
            #       Thus, in y-x coord for the visualization,
            #       going left/right is opposite.
            #       That is, going left in VO config. space is going right in simulation space
            #       and vice versa.
            #       If you want to going right (a.k.a. complying with COLREGs),
            #       you have to design it to be going left in VO config. space

            isAllVelsCollidable = self.__is_all_vels_collidable(
                vel_all_annotated=reachableVel_all_annotated, 
                shipID_all=TS.keys(),
                )

            isAllVelsAvoidable = self.__is_all_vels_avoidable(
                vel_all_annotated=reachableVel_all_annotated, 
                shipID_all=TS.keys(),
                )

            # When no avoidance velocities
            if isAllVelsCollidable:
                velCandidates = self.__remove_annotation(reachableVel_all_annotated)
                # NOTE: `_select_vel_inside_RVOs()` returns a velocity where 
                #       all the reachable velocities are inside the collision cones
                vA_post = self.__select_vel_inside_RVOs(
                    velCandidates, 
                    RVOdata_all,
                    V_des,
                    )
                # print("All vectors can not avoid the collision\n Find vector in collision cone")

            # When no collision velocities
            elif isAllVelsAvoidable:
                velCandidates = self.__remove_annotation(reachableVel_all_annotated)
                vA_post = min(
                    velCandidates,
                    key= lambda v: np.linalg.norm(v - V_des),
                    )
                # print("All vectors can avoid the collision")
                
            else:
                # No strategy (only avoidance velocities)
                avoidanceVel_all_annotated = self.__take_vels(
                    vel_all_annotated=reachableVel_all_annotated,
                    annotation=['inLeft', 'inRight', 'inTimeHorizon'],
                    shipID_all=TS.keys(),
                    )
                
                selection_key = "inLeft"
                # if nearest_status == "Port crossing" and nearest_DCPA <= 60 :
                #     selection_key = "inRight"
                    # print("Neareast ship is Port crossing situation. Avoid to left side")
                

                #=========================================================+
                """ <<<<<<<< IMPORTANT! MUST READ IT CAREFULLY! >>>>>>>>>>|
                - Since the RVO in this code is implemented based on x-y coord., the annotations such as 'left' or 'right' relies on x-y coord. 
                - If you are simulating on y-x coord., it will be opposite from what you want. Thus, consider the 'left' and 'right' carefully. 
                - For example, if you want to take the 'left velocities' in y-x coord., it will be the 'right velocities in x-y coord, so you have to take 'inRight' annotations.                                    
                """                                                        # |:
                avoidanceAllRightVel_all_annotated = self.__take_vels(  # |   
                    vel_all_annotated=reachableVel_all_annotated,       # |
                    annotation=[selection_key],                              # |
                    shipID_all=TS.keys(),                               # |
                    )
                # avoidanceAllRightVel_all_annotated = self.__take_vels(  
                #     vel_all_annotated=reachableVel_all_annotated,       
                #     annotation=['inLeft', 'inRight', 'inTimeHorizon'],                              
                #     shipID_all=TS.keys(),
                #     )                                                   # |
                #=========================================================+

                if avoidanceAllRightVel_all_annotated:
                    velCandidates = self.__remove_annotation(avoidanceAllRightVel_all_annotated)
                else:
                    velCandidates = self.__remove_annotation(avoidanceVel_all_annotated)
                # Take the closest velocity to V_des among the chosen velocities
                vA_post = min(
                    velCandidates,
                    key= lambda v: np.linalg.norm(v - V_des),
                    )
                
                if np.linalg.norm(vA_post - V_des) < 0.1:
                    # print("No collision risk")
                    # print("Follow the vector to goal")
                    pass
                
        return vA_post 

    def __extract_RVO_data(self, OS, TS):
        # TODO: `static_OB` is used in the future?
        """ 
        It generates a container that contains the data for RVO formulation.

        Args:

            OS: 
                - State of the OS
                - Example:
                    {
                        'Ship_ID': 2000,
                        'Pos_X': -49.99,
                        ...,
                        'radius': 8.0,
                    }
            TS: 
                - State of the TSs
                - Example:
                    {
                        2001:{
                            'Ship_ID': '2001',
                            ...,
                            'radius': 4.0,
                            },
                        2002:{
                            'Ship_ID': '2002',
                            ...,
                            'radius': 4.0,
                            },
                        ...,
                    }

        Return:

            RVOdata_all: 
                - A set of dictionaries that contain the information to calculate RVO
                - Example:
                    [
                        {
                            "TS_ID": 2001,
                            "LOSdist": 16.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.85, 1.15]),
                            "vB": ndarray([-0.12, 0.52]),
                            "boundLineAngle_left_rad_global: 0.24", 
                            "boundLineAngle_right_rad_global: 0.31", 
                            "collisionConeShifted_local: ndarray([0.12, 0.32])",
                        },
                        {
                            "TS_ID": 2002,
                            "LOSdist": 12.29, 
                            "mapped_radius": 16.0, 
                            "vA": ndarray([0.25, 1.25]),
                            "vB": ndarray([-0.22, 0.42]),
                            "boundLineAngle_left_rad_global: 0.21", 
                            "boundLineAngle_right_rad_global: 0.35", 
                            "collisionConeShifted_local: ndarray([0.32, 0.12])",
                        },
                        ...
                    ]

            pub_collision_cone : 
                TODO: Add explanations

        """

        vA = np.array([OS['V_x'], OS['V_y']])
        pA = np.array([OS['Pos_X'], OS['Pos_Y']])

        RVOdata_all = []
        pub_collision_cone = []

        if TS == None:
            RVOdata_all = None
            pub_collision_cone = []

        else:
            TS_ID = TS.keys()

            for ts_ID in TS_ID:

                vB = np.array([TS[ts_ID]['V_x'], TS[ts_ID]['V_y']]) # velocity of the obstacle
                pB = np.array([TS[ts_ID]['Pos_X'], TS[ts_ID]['Pos_Y']]) # position of of the obstacle

                CRI = TS[ts_ID]['CRI']
                status = TS[ts_ID]['status']

                RVOapexPos_global = pA + (1 - self.weight_alpha) * vA + self.weight_alpha * vB

                # NOTE: LOS: line of sight. The line between pA and pB = relative distance
                LOSdist = np.linalg.norm([pA - pB]) 
                # NOTE: atan2(y, x) = arctangent of y/x 
                LOSangle_rad = atan2(pB[1] - pA[1], pB[0] - pA[0])
                
                # TODO: It represents the "collision" with the configured radius of objects. 
                #       Forcing to change of LOSdist would distort some calculation afterwards.
                #       Revisit and review it if there would be an calculation issue.
                #       I highly expect there must be.
                if TS[ts_ID]['mapped_radius'] > LOSdist:
                    LOSdist = TS[ts_ID]['mapped_radius']
                
                boundLineAngle_left_rad_global = LOSangle_rad + atan2(TS[ts_ID]['mapped_radius'],LOSdist) #TS[ts_ID]['mapped_radius']/
                # boundLineAngle_left_rad_global = TS[ts_ID]['left_boundary'] 
                boundLineAngle_right_rad_global = LOSangle_rad - atan2(TS[ts_ID]['mapped_radius'],LOSdist) #TS[ts_ID]['mapped_radius']/
                # boundLineAngle_right_rad_global = TS[ts_ID]['right_boundary']
                
                collisionConeTranslated = (1 - self.weight_alpha) * vA + self.weight_alpha * vB
                '''
                collisionConeTranslated: 
                    - It's a collision cone's translated position from where the cone's apex is at the center of the agent A. 
                    - This translation is from RVO formulation. For more details, see section:

                        - IV. RECIPROCAL VELOCITY OBSTACLES
                            |
                            + C.Generalized Reciprocal Velocity Obstacles"

                    in the paper "Reciprocal Velocity Obstacle for Real-Time Multi-Agent Navigation".
                '''

                # if status == 'Safe':
                #     boundLineAngle_left_rad_global = OS['Heading'] + pi
                #     boundLineAngle_right_rad_global = OS['Heading'] - pi
                #     RVOapexPos_global = pA


#################################### hyogeun annotation #####################################
                # if self.rule == True:
                #     if status == 'Safe' or status == 'Port crossing':
                #         boundLineAngle_left_rad_global = OS['Heading'] + pi
                #         boundLineAngle_right_rad_global = OS['Heading'] - pi
                #         RVOapexPos_global = pA
                #         LOSdist = 0

                #     else: 
                #         pass

##############################################################################################

# 룰이 추가되어 있으면, 포트 상태일 때 콘을 다르게 만드는 부분이 들어가서 문제가 발생. 이거는 어떻게 할지 무영이가 결정해야 할듯
# port일 때 콘이 위에것처럼 만들어지면 all avoidable이 되어서 가장 빠른 벡터를 선택


                RVOdata = {
                    "TS_ID": ts_ID,
                    "LOSdist": LOSdist,
                    "mapped_radius": TS[ts_ID]['mapped_radius'],
                    "vA": vA,
                    "vB": vB,
                    "boundLineAngle_left_rad_global" : boundLineAngle_left_rad_global,
                    "boundLineAngle_right_rad_global" : boundLineAngle_right_rad_global,
                    "collisionConeTranslated": collisionConeTranslated,
                    "CRI" : CRI,
                    "Status" : status,
                    }
                RVOdata_all.append(RVOdata)
                # To publish the collision cone data for visualization
                bound_left_view = [
                    cos(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                    sin(boundLineAngle_left_rad_global)* int(LOSdist)/2,
                    ]  # cone visualization /3 하면 장애물까지 거리의 1/3만 생김
                bound_right_view = [
                    cos(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                    sin(boundLineAngle_right_rad_global)* int(LOSdist)/2,
                    ]

                pub_collision_cone.append(RVOapexPos_global[0])
                pub_collision_cone.append(RVOapexPos_global[1])
                pub_collision_cone.append(bound_left_view[0] + RVOapexPos_global[0])
                pub_collision_cone.append(bound_left_view[1] + RVOapexPos_global[1])
                pub_collision_cone.append(bound_right_view[0] + RVOapexPos_global[0])
                pub_collision_cone.append(bound_right_view[1] + RVOapexPos_global[1])

        return RVOdata_all, pub_collision_cone

    def VO_update(self, OS_original, TS_original, V_des, static_obstacle_info, static_point_info):   
        """ 
        It computes the velocity selection with RVO formulation and returns selected velocity and collision cone information from the current state of the ships.

        Args: 

            OS_original: 
                - State of the OS
                - Example:
                    {
                        'Ship_ID': 2000,
                        'Pos_X': -49.99,
                        ...,
                        'radius': 8.0,
                    }

            TS_original: 
                - State of the TSs
                - Example:
                    {
                        2001:{
                            'Ship_ID': '2001',
                            ...,
                            'radius': 4.0,
                            },
                        2002:{
                            'Ship_ID': '2002',
                            ...,
                            'radius': 4.0,
                            },
                        ...,
                    }

            static_OB:
                TODO: Add explanations

            V_des: 
                - Unit: m/s
                - The desired velocity pointing out the next waypoint.

        Returns:

            V_opt:
                - Unit: m/s
                - Selected velocity from RVO formulation

            pub_collision_cone:
                TODO: Add explanations
        """
        # self.__include_mappedTSradius(OS_original, TS_original) 

        RVOdata_all, pub_collision_cone = self.__extract_RVO_data(
            OS_original,
            TS_original,
            )

        V_opt = self.__choose_velocity(V_des, RVOdata_all, OS_original, TS_original,static_obstacle_info, static_point_info)

        return V_opt, pub_collision_cone

    def vectorV_to_goal(self, OS, goal, V_max):
        """ 
        It returns the velocity towards the next waypoint regardless of the ship's current state. The velocity is limited by the given maximum speed (Target speed).

        Args: 

            OS:
                - State of the OS
                - Example:
                    {
                        'Ship_ID': 2000,
                        'Pos_X': -49.99,
                        ...,
                        'radius': 8.0,
                    }

            goal: Next waypoint

            V_max:
                - Unit: m/s
                - Designed maximum speed (Target speed)

        Returns:

            V_des:
                - Unit: m/s
                - The desired velocity pointing out the next waypoint.
        """

        Pos_x = OS['Pos_X']
        Pos_y = OS['Pos_Y']
        dif_x = [goal[0] - Pos_x, goal[1] - Pos_y]
        norm = np.linalg.norm(dif_x)
        V_des = [dif_x[k] * V_max / norm for k in range(2)]

        return V_des