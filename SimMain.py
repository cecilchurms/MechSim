import FreeCAD as CAD

import os
import numpy as np
from scipy.integrate import solve_ivp
import math

import SimTools as ST
import SimFunction

Debug = False
DebugArrays = False
# =============================================================================
# ==================================
# Matlab Code from Nikravesh: DAP_BC
# ==================================
# Body = struct ( ...
#    'm'     , 1     , ... % mass                                               NPMasskg
#    'J'     , 1     , ... % moment of inertia                                  NPmomentOfInertia
#    'r'	 , [0;0] , ... % x, y coordinates                                   NPworldCoG
#    'p'  	 , 0     , ... % angle phi                                          NPphi
#    'r_d'   , [0;0] , ... % time derivative of x and y                         NPworldCoGDot
#    'p_d'   , 0     , ... % time derivative of phi                             NPphiDot
#    'A'	 , eye(2), ... % rotational transformation matrix                   NPRotMatrixPhi
#    'r_dd'  , [0;0] , ... % x_double_dot,y_double_do                           NPworldCoGDotDot
#    'p_dd'  , 0     , ... % 2nd time derivative of phi                         NPphiDotDot
#    'irc'   , 0     , ... % index of the 1st element of r in u or u_dot
#    'irv'   , 0     , ... % index of the 1st element of r_dot in u or u_dot
#    'ira'   , 0     , ... % index of the 1st element of r_dot2 in v_dot
#    'm_inv' , 1     , ... % mass inverse                                       ---
#    'J_inv' , 1     , ... % inverse of moment of inertia                       ---
#    'wgt'   , [0;0] , ... % weight of body as a force vector                   NPWeight
#    'f'     , [0;0] , ... % sum of forces that act on the body                 NPsumForces
#    'n'     , 0     , ... % sum of moments that act on the body                NPsumMoments
#    'shape' , ' '   , ... % 'circle', 'rect', line
#    'R'     , 1     , ... % radius of the circle
#    'circ'  , []    , ... % points on circumference of the circle
#    'W'     , 0     , ... % width of the rectangle
#    'H'     , 0     , ... % hight of the rectangle
#    'color' , 'k'   , ... % default color for the body
#    'P4'    , []    , ... % 4 corners of the rectangle
#    'pts'   , []      ... % point indexes associated with this body
# );

# Point = struct ( ...
#    'Bindex' , 0     , ... % body index
#    'sPlocal', [0;0] , ... % body-fixed coordinates            NPpointXiEta
#    'sP'     , [0;0] , ... % x, y components of vector s       NPpointXYrelCoG
#    'sP_r'   , [0;0] , ... % vector s rotated                  NPpointXYrelCoGRot
#    'sP_d'   , [0;0] , ... % s_P_dot                           NPpointXYrelCoGDot
#    'rP'     , [0;0] , ... % x, y coordinates of the point     NPpointWorld
#    'rP_d'   , [0;0] , ... % r_P_dot                           NPpointWorldDot
#    'rP_dd'  , [0;0]   ... % r_P_dot2                          NPpointWorldDotDot
# );

# Force = struct ( ...
#    'type'   , 'ptp',  ... % element type: ptp, rot_sda, weight, fp, f, T
#    'iPindex', 0    ,  ... % index of the head (arrow) point
#    'jPindex', 0    ,  ... % index of the tail point
#    'iBindex', 0    ,  ... % index of the head (arrow) body
#    'jBindex', 0    ,  ... % index of the tail body
#    'k'      , 0    ,  ... % spring stiffness
#    'L0'     , 0    ,  ... % undeformed length
#    'theta0' , 0    ,  ... % undeformed angle
#    'dc'     , 0    ,  ... % damping coefficient
#    'f_a'    , 0    ,  ... % constant actuator force
#    'T_a'    , 0    ,  ... % constant actuator torque
#    'gravity', 9.81 ,  ... % gravitational constant
#    'wgt'    , [0;-1], ... % gravitational direction
#    'flocal' , [0;0],  ... % constant force in local frame
#    'f'      , [0;0],  ... % constant force in x-y frame
#    'T'      , 0    ,  ... % constant torque in x-y frame
#    'iFunct' , 0       ... % analytical function index
# );

# Joint = struct ( ...
#    'type'     , 'rev' , ... % joint type: rev, tran, rev-rev, rev-tran, Fixed, disc, rel-rot, rel-tran
#    'iBindex'  , 0     , ... % body index i
#    'jBindex'  , 0     , ... % body index j
#    'iPindex'  , 0     , ... % point Pi index
#    'jPindex'  , 0     , ... % point Pj index
#    'iUindex'  , 0     , ... % unit vector u_i index
#    'jUindex'  , 0     , ... % unit vector u_j index
#    'iFunct'   , 0     , ... % analytical function index
#    'L' 	   , 0     , ... % constant length
#    'R' 	   , 1     , ... % constant radius
#    'x0'       , 0     , ... % initial condition x for disc
#    'p0'       , 0     , ... % initial condition phi for a disc (or Fixed)
#    'd0'       , []    , ... % initial condition for d (Fixed)
#    'fix'      , 0     , ... % fix relative dof if = 1 (rev or tran)
#    'nbody'    , 2     , ... % number of moving bodies involved
#    'mrows'    , 2     , ... % number of rows (constraints)
#    'rows'     , 0     , ... % row index-start
#    'rowe'     , 0     , ... % row index-end
#    'colis'    , 0     , ... % column index for body i-start
#    'colie'    , 0     , ... % column index for body i-end
#    'coljs'    , 0     , ... % column index for body j-start
#    'colje'    , 0     , ... % column index for body j-end
#    'lagrange' , zeros(3,1) ... % Lagrange multipliers
# );

# Unit = struct ( ...
#    'Bindex', 0	,   ... % body index
#    'ulocal', [1;0],   ... % u_prime; xi and eta components
#    'u'     , [0;0],	... % x, y components
#    'u_r'   , [0;0],	... % vector u rotated
#    'u_d'   , [0;0]    ... % u_dot
# );

#  =========================================================================
#  -------------------------------------------------------------------------
class SimMainC:
    """Instantiated when the 'solve' button is clicked in the task panel"""
    #  -------------------------------------------------------------------------
    def __init__(self, simEnd, simDelta, Accuracy, correctInitial):
        if Debug: ST.Mess("SimMainClass-__init__")

        # Make the parameters passed via the __init__ function
        # global to the class
        self.simEnd = simEnd
        self.simDelta = simDelta
        self.correctInitial = correctInitial

        # Dictionary of the pointers for Dynamic calling of the Acceleration functions
        self.dictAccelerationFunctions = {
            0: self.Revolute_Acc,
            1: self.Fixed_Acc,
        }
        """
        2: self.Translational_Acc,
        3: self.Revolute_Revolute_Acc,
        4: self.Translational_Revolute_Acc,
        5: self.Disc_Acc,
        6: self.Driven_Revolute_Acc,
        7: self.Driven_Translational_Acc,
        """
        # Dictionary of the pointers for Dynamic calling of the constraint functions
        self.dictconstraintFunctions = {
            0: self.Revolute_constraint,
            1: self.Fixed_constraint,
        }
        """
        2: self.Translational_constraint,
        3: self.Revolute_Revolute_constraint,
        4: self.Translational_Revolute_constraint,
        5: self.Disc_constraint,
        6: self.Driven_Revolute_constraint,
        7: self.Driven_Translational_constraint,
        """
        # Dictionary of the pointers for Dynamic calling of the Jacobian functions
        self.dictJacobianFunctions = {
            0: self.Revolute_Jacobian,
            1: self.Fixed_Jacobian,
        }
        """
        2: self.Translational_Jacobian,
        3: self.Revolute_Revolute_Jacobian,
        4: self.Translational_Revolute_Jacobian,
        5: self.Disc_Jacobian,
        6: self.Driven_Revolute_Jacobian,
        7: self.Driven_Translational_Jacobian,
        """
        
        # Store the required accuracy figures
        self.relativeTolerance = 10 ** (-Accuracy - 2)
        self.absoluteTolerance = 10 ** (-Accuracy - 4)

        # Counter of function evaluations
        self.Counter = 0

        # We get the objects and set global constants
        self.simGlobalObj = CAD.ActiveDocument.findObjects(Name="^SimGlobal$")[0]

        self.solverObj = CAD.ActiveDocument.findObjects(Name="^SimSolver$")[0]

        self.bodyGroup = CAD.ActiveDocument.findObjects(Name="^LinkGroup")
        self.numBodies = len(self.bodyGroup)
        self.numMovBodiesx3 = (self.numBodies-1) * 3
        for body in self.bodyGroup:
            if Debug: ST.Mess(body.Name)

        self.jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0].Group
        self.numJoints = len(self.jointGroup)
        for joint in self.jointGroup:
            if Debug: ST.Mess(joint.Name)

        self.forceList = CAD.ActiveDocument.findObjects(Name="^SimForce")
        self.numForces = len(self.forceList)

        # Set a variable to flag whether we have reached the end error-free
        # It will be available to SimSolver as an instance variable
        self.initialised = False

        # Get the plane normal rotation matrix from SimGlobal
        # This will rotate all the coordinates in the model, to be in the X-Y plane
        self.xyzToXYRotation = CAD.Rotation(CAD.Vector(0.0, 0.0, 1.0), self.simGlobalObj.movementPlaneNormal)

        # Find the maximum number of points in any of the bodies
        # We will need this so we can initialise large enough NumPy arrays
        maxNumBodyJoints = 0
        for body in self.bodyGroup:
            if maxNumBodyJoints < len(body.JointIndexList):
                maxNumBodyJoints = len(body.JointIndexList)

        # Initialise the size of all the NumPy arrays and fill with zeros
        self.initNumPyArrays(maxNumBodyJoints)

        # Transfer all the 3D stuff into the NumPy arrays
        bodyIndex = -1
        for bodyObj in self.bodyGroup:
            bodyIndex += 1
            self.NPnumJointPointsInBody[bodyIndex] = len(bodyObj.JointIndexList)

            # Re-Calculate the body CoG and MoI
            ST.updateCoGMoI(bodyObj)

            # All Mass and moment of inertia stuff
            self.NPMasskg[bodyIndex] = bodyObj.masskg
            self.NPmomentOfInertia[bodyIndex] = bodyObj.momentOfInertia
            npVec = ST.CADVecToNumPy(self.simGlobalObj.gravityVector * bodyObj.masskg)
            self.NPWeight[bodyIndex] = npVec

            # The coordinates of the CoG in world coordinates
            # are the world coordinates of the body
            # All points in the body are relative to this point

            # World
            CoG = bodyObj.worldCoG
            npCoG = ST.CADVecToNumPy(CoG)
            self.NPworldCoG[bodyIndex, 0:2] = npCoG
            self.NPworldCoGRot[bodyIndex, 0:2] = ST.Rot90NumPy(npCoG)

            # WorldDot
            npvec = ST.CADVecToNumPy(bodyObj.worldDot)
            self.NPworldCoGDot[bodyIndex, 0:2] = npvec
            self.NPworldCoGDotRot[bodyIndex, 0:2] = ST.Rot90NumPy(npvec)

            # WorldDotDot
            self.NPworldCoGDotDot[bodyIndex, 0:2] = np.zeros((1, 2))

            # Phi defined by the longest vector from CoG to a Joint
            # The first body is ALWAYS the ground body
            # and hence cannot be rotated away from 0.0
            if bodyIndex == 0:
                self.NPphi[bodyIndex] = 0.0
            else:
                maxNorm = 0.0
                largest = 0
                relCoG = CAD.Vector(0.0, 0.0, 0.0)
                for jointIndex in range(len(bodyObj.JointIndexList)):
                    relCoG = bodyObj.PointRelWorldList[jointIndex] - CoG
                    if maxNorm < relCoG.Length:
                        maxNorm = relCoG.Length
                        largest = jointIndex
                # Handle the case where it is vertical
                if abs(relCoG[0]) < 1e-6:
                    if relCoG[1] > 0.0:
                        self.NPphi[bodyIndex] = np.pi/2.0
                    else:
                        self.NPphi[bodyIndex] = np.pi
                else:
                    relCoG = bodyObj.PointRelWorldList[largest] - CoG
                    self.NPphi[bodyIndex] = np.atan2(relCoG[1], relCoG[0])

            # The phiDot axis vector is by definition perpendicular to the movement plane,
            # so we don't have to do any rotating from the phiDot value set in bodyObj
            self.NPphiDot[bodyIndex] = bodyObj.phiDot

            # We will now calculate the rotation matrix 
            # and use it to find the local coordinates of the points
            self.NPRotMatrixPhi[bodyIndex] = ST.CalculateRotationMatrix(self.NPphi[bodyIndex])

            for pointIndex in range(len(bodyObj.JointIndexList)):
                # Point Local - vector from module body CoG to the point, in body LCS coordinates
                # [This is what we needed phi for, to set the orientation of the body LCS]

                npVec = ST.CADVecToNumPy(bodyObj.PointRelWorldList[pointIndex])
                self.NPpointWorld[bodyIndex, pointIndex] = npVec
                self.NPpointWorldRot[bodyIndex, pointIndex] = ST.Rot90NumPy(npVec)

                # Point Vector - vector from body CoG to the point in world coordinates
                npVec -= npCoG
                self.NPpointXYrelCoG[bodyIndex, pointIndex, 0:2] = npVec
                self.NPpointXYrelCoGRot[bodyIndex][pointIndex] = ST.Rot90NumPy(npVec)

                # Local coordinates of the point in Eta/Xi frame
                self.NPpointXiEta[bodyIndex, pointIndex, 0:2] = npVec @ self.NPRotMatrixPhi[bodyIndex]

                # Point Vector Dot
                self.NPpointXYrelCoGDot[bodyIndex][pointIndex] = np.zeros((1, 2))

                # Point World Dot
                self.NPpointWorldDot[bodyIndex][pointIndex] = np.zeros((1, 2))

            # Next pointIndex
        # Next bodyIndex

        # Print out what we have populated for debugging
        if DebugArrays == True: #Debug:
            ST.Mess("Body Labels -- Body Names")
            for bodyObj in self.bodyGroup:
                ST.Mess(bodyObj.Label+" - "+bodyObj.Name)
            ST.Mess("Mass: [g]")
            ST.PrintNp1D(True, self.NPMasskg * 1.0e3)
            ST.Mess("")
            ST.Mess("Mass: [kg]")
            ST.PrintNp1D(True, self.NPMasskg)
            ST.Mess("")
            ST.Mess("Moment of Inertia: [kg mm^2]")
            ST.PrintNp1D(True, self.NPmomentOfInertia)
            ST.Mess("")
            ST.Mess("Weight Vector: [kg mm /s^2 = mN]")
            ST.PrintNp2D(self.NPWeight)
            ST.Mess("")
            ST.Mess("Sum Forces: [kg.mm/s^2 = mN]")
            ST.PrintNp2D(self.NPsumForces)
            ST.Mess("")
            ST.Mess("Sum Moments: [N.mm]")
            ST.PrintNp1D(True, self.NPsumMoments)
            ST.Mess("")
            ST.Mess("World: [mm relative to CoG]")
            ST.PrintNp2D(self.NPworldCoG)
            ST.Mess("")
            ST.Mess("World Rot:")
            ST.PrintNp2D(self.NPworldCoGRot)
            ST.Mess("")
            ST.Mess("WorldDot: [mm/s]")
            ST.PrintNp2D(self.NPworldCoGDot)
            ST.Mess("")
            ST.Mess("WorldDotRot: [mm/s]")
            ST.PrintNp2D(self.NPworldCoGDotRot)
            ST.Mess("")
            ST.Mess("phi: [deg]")
            ST.PrintNp1Ddeg(True, self.NPphi)
            ST.Mess("")
            ST.Mess("phi: [rad]")
            ST.PrintNp1D(True, self.NPphi)
            ST.Mess("")
            ST.Mess("phiDot: [deg/sec]")
            ST.PrintNp1Ddeg(True, self.NPphiDot)
            ST.Mess("")
            ST.Mess("phiDot: [rad/sec]")
            ST.PrintNp1D(True, self.NPphiDot)
            ST.Mess("")
            ST.Mess("Rotation Matrix:")
            ST.PrintNp3D(self.NPRotMatrixPhi)
            ST.Mess("")
            ST.Mess("PointLocal: [mm]")
            ST.PrintNp3D(self.NPpointXiEta)
            ST.Mess("")
            ST.Mess("Number of Joint points in body:")
            ST.PrintNp1D(True, self.NPnumJointPointsInBody)
            ST.Mess("")
            ST.Mess("Vector from CoG to point in World coordinates: [mm]")
            ST.PrintNp3D(self.NPpointXYrelCoG)
            ST.Mess("")
            ST.Mess("Vector from Origin to point in World Coordinates: [mm]")
            ST.PrintNp3D(self.NPpointWorld)
            ST.Mess("")
            ST.Mess("Rotated Vector from Origin to point: [mm]")
            ST.PrintNp3D(self.NPpointWorldRot)
            ST.Mess("")
            ST.Mess("Velocity of Vector from Origin to point:")
            ST.PrintNp3D(self.NPpointWorldDot)
            ST.Mess("")

        # Make an array with the respective body Mass and moment of inertia
        # do not add the body=0 to the list because it is ground
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        # % Mass (inertia) matrix as an array
        #    M_array = zeros(nB3,1); M_inv_array = zeros(nB3,1);
        #    for Bi = 1:nB
        #        is = 3*(Bi - 1) + 1;
        #        ie = is + 2;
        #        M_array(is:ie,1) = [Bodies(Bi).m; Bodies(Bi).m; Bodies(Bi).J];
        #        M_inv_array(is:ie,1) = [Bodies(Bi).m_inv; Bodies(Bi).m_inv; Bodies(Bi).J_inv];
        #    end
        # ==================================
        self.NPMassArray = np.zeros(self.numMovBodiesx3)
        bodyIndex = -1
        for bodyObj in self.bodyGroup:
            bodyIndex += 1
            if bodyIndex != 0:
                self.NPMassArray[(bodyIndex-1)*3:bodyIndex*3] = (
                    self.NPMasskg[bodyIndex], 
                    self.NPMasskg[bodyIndex], 
                    self.NPmomentOfInertia[bodyIndex])

        # Transfer the joint unit vector coordinates to the NumPy arrays
        jointIndex = -1
        for jointObj in self.jointGroup:
            jointIndex += 1
            """
            # Unit vector on body I in body local coordinates
            self.NPjointUnit_I_XiEta[jointIndex] = ST.NormalizeVec(self.NPpointXiEta[jointObj.bodyHeadIndex, jointObj.point_I_j_Index] -
                                                                     self.NPpointXiEta[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])
            # Unit vector on body I in world coordinates
            self.NPjointUnit_I_World[jointIndex] = ST.NormalizeNpVec(self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.point_I_j_Index] -
                                                                     self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])
            self.NPjointUnit_I_WorldRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_I_World[jointIndex].copy())
            self.NPjointUnit_I_WorldDot[jointIndex] = ST.NormalizeNpVec(self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.point_I_j_Index] -
                                                                        self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])
            self.NPjointUnit_I_WorldDotRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_I_WorldDot[jointIndex].copy())
    
            # Unit vector on body J in body local coordinates
            self.NPjointUnit_J_XiEta[jointIndex] = ST.NormalizeVec(self.NPpointXiEta[jointObj.bodyTailIndex, jointObj.point_J_j_Index] -
                                                                     self.NPpointXiEta[jointObj.bodyTailIndex, jointObj.pointTailIndex])
            # Unit vector on body J in world coordinates
            self.NPjointUnit_J_World[jointIndex] = ST.NormalizeVec(self.NPpointWorld[jointObj.bodyTailIndex, jointObj.point_J_j_Index] -
                                                                     self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex])
            self.NPjointUnit_J_WorldRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_J_World[jointIndex].copy())
            self.NPjointUnit_J_WorldDot[jointIndex] = ST.NormalizeVec(self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.point_J_j_Index] -
                                                                        self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.pointTailIndex])
            self.NPjointUnit_J_WorldDotRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_J_WorldDot[jointIndex].copy())

            # Find the length of the link between the first point on each body - signed scalar
            unitPinInSlot = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                            self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
            length = np.sqrt(unitPinInSlot[0]**2 + unitPinInSlot[1]**2)
            dotProduct = self.NPjointUnit_I_World[jointIndex].dot(unitPinInSlot)
            if dotProduct < 0.0:
                jointObj.lengthLink = -length
            else:
                jointObj.lengthLink = length
            """
        """
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        # % Unit vectors
        #   nU = length(Uvectors);
        #    for Vi = 1:nU
        #        if Uvectors(Vi).Bindex == 0
        #            Uvectors(Vi).u = Uvectors(Vi).ulocal;
        #            Uvectors(Vi).u_r = s_rot(Uvectors(Vi).u);
        #        end
        #    end
        # ==================================
        jointIndex = -1
        for jointObj in self.jointGroup
            jointIndex += 1
            # If the body is attached to ground then its unit vector local coordinates are world coordinates
            if jointObj.bodyHeadIndex == 0:
                self.NPjointUnit_I_World[jointIndex] = self.NPjointUnit_I_XiEta[jointIndex]
                self.NPjointUnit_I_WorldRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_I_World[jointIndex].copy())
            if jointObj.bodyTailIndex == 0:
                self.NPjointUnit_J_World[jointIndex] = self.NPjointUnit_J_XiEta[jointIndex]
                self.NPjointUnit_J_WorldRot[jointIndex] = ST.Rot90NumPy(self.NPjointUnit_J_World[jointIndex].copy())
            """
        
        # Assign number of constraints and number of bodies
        # to each defined joint according to its type
        jointIndex = -1
        for jointObj in self.jointGroup:
            jointIndex += 1
            # Only allow joints currently included in the code
            if hasattr(jointObj, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                if jointObj.SimJoint == "Revolute":
                    # ==================================
                    # Matlab Code from Nikravesh: DAP_BC
                    # ==================================
                    #        case {'rev'};
                    #            Joints(Ji).mrows = 2;
                    #            Joints(Ji).nbody = 2;
                    #            Pi = Joints(Ji).iPindex;
                    #            Pj = Joints(Ji).jPindex;
                    #            Bi = Points(Pi).Bindex;
                    #            Joints(Ji).iBindex = Bi;
                    #            Bj = Points(Pj).Bindex;
                    #            Joints(Ji).jBindex = Bj;
                    #            if Joints(Ji).fix == 1
                    #                Joints(Ji).mrows = 3;
                    #                if Bi == 0
                    #                    Joints(Ji).p0 = -Bodies(Bj).p;
                    #                elseif Bj == 0
                    #                    Joints(Ji).p0 = Bodies(Bi).p;
                    #                else
                    #                    Joints(Ji).p0 = Bodies(Bi).p - Bodies(Bj).p;
                    #                end
                    #            end
                    # ==================================
                    if jointObj.FunctType == -1:
                        jointObj.mConstraints = 2
                        jointObj.nBodies = 2
                        if jointObj.fixDof is True:
                            jointObj.mConstraints = 3
                            # Set the initial angle phi0 
                            if jointObj.bodyHeadIndex == 0:
                                self.NPphi0[jointObj.jointIndex] = -self.NPphi[jointObj.bodyTailIndex]
                            elif jointObj.bodyTailIndex == 0:
                                self.NPphi0[jointObj.jointIndex] = self.NPphi[jointObj.bodyHeadIndex]
                            else:
                                self.NPphi0[jointObj.jointIndex] = (
                                        self.NPphi[jointObj.bodyHeadIndex] 
                                        - self.NPphi[jointObj.bodyTailIndex])
                    #else:
                        # ==================================
                        # Matlab Code from Nikravesh: DAP_BC
                        # ==================================
                        #        case {'rel-rot'}                                       % revised August 2022
                        #            Joints(Ji).mrows = 1; Joints(Ji).nbody = 1;        % revised August 2022
                        # ==================================
                    #    jointObj.mConstraints = 1
                    #    jointObj.nBodies = 1

                elif jointObj.SimJoint == "Fixed":
                    # ==================================
                    # Matlab Code from Nikravesh: DAP_BC
                    # ==================================
                    #        case {'Fixed'}
                    #            Joints(Ji).mrows = 3;
                    #            Joints(Ji).nbody = 2;
                    #            Bi = Joints(Ji).iBindex;
                    #            Bj = Joints(Ji).jBindex;
                    #            if Bi == 0
                    #                Joints(Ji).d0 = -Bodies(Bj).A'*Bodies(Bj).r;
                    #                Joints(Ji).p0 = -Bodies(Bj).p;
                    #            elseif Bj == 0
                    #                Joints(Ji).d0 = Bodies(Bi).r;
                    #                Joints(Ji).p0 = Bodies(Bi).p;
                    #            else
                    #                Joints(Ji).d0 = Bodies(Bj).A'*(Bodies(Bi).r - Bodies(Bj).r);
                    #                Joints(Ji).p0 = Bodies(Bi).p - Bodies(Bj).p;
                    #            end
                    # ==================================
                    jointObj.mConstraints = 3
                    jointObj.nBodies = 2
                    if jointObj.bodyHeadIndex == 0:
                        V = -self.NPRotMatrixPhi[jointObj.bodyTailIndex].T @ self.NPworldCoG[jointObj.bodyTailIndex]
                        self.NPphi0[jointObj.jointIndex] = -self.NPphi[jointObj.bodyTailIndex]
                    elif jointObj.bodyTailIndex == 0:
                        V = self.NPworldCoG[jointObj.bodyHeadIndex]
                        self.NPphi0[jointObj.jointIndex] = self.NPphi[jointObj.bodyHeadIndex]
                    else:
                        V = self.NPRotMatrixPhi[jointObj.bodyTailIndex].T @ (self.NPworldCoG[jointObj.bodyHeadIndex] -
                                                                          self.NPworldCoG[jointObj.bodyTailIndex])
                        self.NPphi0[jointObj.jointIndex] = self.NPphi[jointObj.bodyHeadIndex] - \
                                        self.NPphi[jointObj.bodyTailIndex]
                    self.NPd0[jointObj.jointIndex, 0] = V[0]
                    self.NPd0[jointObj.jointIndex, 1] = V[1]
                else:
                    CAD.Console.PrintError("Unknown Joint Type - this should never occur" + str(jointObj.SimJoint) + "\n")
                # end of is valid SimJoint
        # Next Joint Object
        """
        elif jointObj.SimJoint == ST.JOINT_TYPE_DICTIONARY["Translation"]:
            # ==================================
            # Matlab Code from Nikravesh: DAP_BC
            # ==================================
            #        case {'tran'}
            #            Joints(Ji).mrows = 2;
            #            Joints(Ji).nbody = 2;
            #            Pi = Joints(Ji).iPindex;
            #            Pj = Joints(Ji).jPindex;
            #            Bi = Points(Pi).Bindex;
            #            Joints(Ji).iBindex = Bi;
            #            Bj = Points(Pj).Bindex;
            #            Joints(Ji).jBindex = Bj;
            #            if Joints(Ji).fix == 1
            #                Joints(Ji).mrows = 3;
            #                if Bi == 0
            #                    Joints(Ji).p0 = norm(Points(Pi).rP - ...
            #                        Bodies(Bj).r - Bodies(Bj).A*Points(Pj).sPlocal);
            #                elseif Bj == 0
            #                    Joints(Ji).p0 = norm(Bodies(Bi).r + ...
            #                        Bodies(Bi).A*Points(Pi).sPlocal - Points(Pj).rP);
            #                else
            #                    Joints(Ji).p0 = norm(Bodies(Bi).r + ...
            #                        Bodies(Bi).A*Points(Pi).sPlocal - ...
            #                        Bodies(Bj).r - Bodies(Bj).A*Points(Pj).sPlocal);
            #                end
            #            end
            # ==================================
            jointObj.mConstraints = 2
            jointObj.nBodies = 2
            if jointObj.fixDof is True:
                jointObj.mConstraints = 3
                if jointObj.bodyHeadIndex == 0:
                    vec = (+ self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex]
                           - self.NPworldCoG[jointObj.bodyTailIndex]
                           - self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ self.NPpointXiEta[jointObj.bodyTailIndex, jointObj.pointTailIndex])
                elif jointObj.bodyTailIndex == 0:
                    vec = (- self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
                           + self.NPworldCoG[jointObj.bodyHeadIndex]
                           + self.NPRotMatrixPhi[jointObj.bodyHeadIndex] @ self.NPpointXiEta[jointObj.bodyHeadIndex, point_i_Index])
                else:
                    vec = (+ self.NPworldCoG[jointObj.bodyHeadIndex]
                           + self.NPRotMatrixPhi[jointObj.bodyHeadIndex] @ self.NPpointXiEta[jointObj.bodyHeadIndex, jointObj.pointHeadIndex]
                           - self.NPworldCoG[jointObj.bodyTailIndex]
                           - self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ self.NPpointXiEta[jointObj.bodyTailIndex, jointObj.pointTailIndex])
                jointObj.phi0 = np.sqrt(vec.dot(vec))
                """
        """
        elif jointObj.SimJoint == ST.JOINT_TYPE_DICTIONARY["Revolute-Revolute"]:
            # ==================================
            # Matlab Code from Nikravesh: DAP_BC
            # ==================================
            #        case {'rev-rev'}
            #            Joints(Ji).mrows = 1;
            #            Joints(Ji).nbody = 2;
            #            Pi = Joints(Ji).iPindex;
            #            Pj = Joints(Ji).jPindex;
            #            Joints(Ji).iBindex = Points(Pi).Bindex;
            #            Joints(Ji).jBindex = Points(Pj).Bindex;
            # ==================================
            jointObj.mConstraints = 1
            jointObj.nBodies = 2
            """
        """
        elif jointObj.SimJoint == ST.JOINT_TYPE_DICTIONARY["Translation-Revolute"]:
            # ==================================
            # Matlab Code from Nikravesh: DAP_BC
            # ==================================
            #        case {'rev-tran'}
            #            Joints(Ji).mrows = 1;
            #            Joints(Ji).nbody = 2;
            #            Pi = Joints(Ji).iPindex;
            #            Pj = Joints(Ji).jPindex;
            #            Joints(Ji).iBindex = Points(Pi).Bindex;
            #            Joints(Ji).jBindex = Points(Pj).Bindex;
            # ==================================
            jointObj.mConstraints = 1
            jointObj.nBodies = 2
            """
        """
        elif jointObj.SimJoint == ST.JOINT_TYPE_DICTIONARY["Driven-Translation"]:
            # ==================================
            # Matlab Code from Nikravesh: DAP_BC
            # ==================================
            #        case {'rel-tran'}                                      % revised August 2022
            #            Joints(Ji).mrows = 1; Joints(Ji).nbody = 1;        % revised August 2022
            #            Pi = Joints(Ji).iPindex; Pj = Joints(Ji).jPindex;  % revised August 2022
            #            Bi = Points(Pi).Bindex; Joints(Ji).iBindex = Bi;   % revised August 2022
            #            Bj = Points(Pj).Bindex; Joints(Ji).jBindex = Bj;   % revised August 2022
            # ==================================
            jointObj.mConstraints = 1
            jointObj.nBodies = 1
            """
        """
        elif jointObj.SimJoint == ST.JOINT_TYPE_DICTIONARY["Disc"]:
            # ==================================
            # Matlab Code from Nikravesh: DAP_BC
            # ==================================
            #        case {'disc'}
            #            Joints(Ji).mrows = 2;
            #            Joints(Ji).nbody = 1;
            # ==================================
            jointObj.mConstraints = 2
            jointObj.nBodies = 1
            radiusVector = self.NPpointXiEta[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                           self.NPpointXiEta[jointObj.bodyHeadIndex, jointObj.pointTailIndex]
            jointObj.Radius = np.sqrt(radiusVector.dot(radiusVector))
            jointObj.phi0 = np.arctan2(radiusVector[1], radiusVector[0])
            """

        """
        # Run through the joints and find if any of them use a driver function
        # if so, then initialize the parameters for the driver function routine
        self.driverObjDict = {}
        for jointObj in self.NPjointObjList:
            # If there is a driver function, then
            # store an instance of the class in driverObjDict and initialize its parameters
            if jointObj.FunctType != -1:
                self.driverObjDict[jointObj.Name] = SimFunction.FunctionC(
                    [jointObj.FunctType,
                     jointObj.startTimeDriveFunc, jointObj.endTimeDriveFunc,
                     jointObj.startValueDriveFunc, jointObj.endValueDriveFunc,
                     jointObj.endDerivativeDriveFunc,
                     jointObj.Coeff0, jointObj.Coeff1, jointObj.Coeff2, jointObj.Coeff3, jointObj.Coeff4, jointObj.Coeff5]
                )
                """
        # Add up all the numbers of constraints and allocate row start and end pointers
        self.numConstraints = 0
        for jointObj in self.jointGroup:
            if hasattr(jointObj, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                jointObj.rowStart = self.numConstraints
                jointObj.rowEnd = self.numConstraints + jointObj.mConstraints
                self.numConstraints = jointObj.rowEnd

        # Return with a flag to show we have reached the end of init error-free
        self.initialised = True
    #  -------------------------------------------------------------------------
    def MainSolve(self):
        ###########################################
        # CLEAN UP ERRONEOUS INPUTS BEFORE STARTING
        ###########################################
        
        # Correct for initial conditions consistency if requested
        if self.numConstraints != 0 and self.correctInitial:
            if self.correctInitialConditions() is False:
                CAD.Console.PrintError("Initial Conditions not successfully calculated")
                return

        # Determine any redundancy between constraints
        Jacobian = self.GetJacobian()
        if DebugArrays == True:
            ST.Mess("Jacobian calculated to determine rank of solution")
            ST.PrintNp2D(Jacobian)
        matrixRank = np.linalg.matrix_rank(Jacobian)
        if matrixRank < self.numConstraints:
            CAD.Console.PrintError('The constraints exhibit Redundancy\n')
            return

        # Velocity correction
        velArrayNp = np.zeros((self.numMovBodiesx3,), dtype=np.float64)
        # Move velocities to the corrections array
        for bodyIndex in range(1, self.numBodies):
            velArrayNp[(bodyIndex-1) * 3: bodyIndex*3] = \
                self.NPworldCoGDot[bodyIndex, 0], \
                self.NPworldCoGDot[bodyIndex, 1], \
                self.NPphiDot[bodyIndex]
        # Solve for velocity at time = 0
        # Unless the joint is Driven-Revolute or Driven-Translational
        # RHSVel = [0,0,...]   (i.e. a list of zeros)
        solution = np.linalg.solve(Jacobian @ Jacobian.T, (Jacobian @ velArrayNp)) # ToDo - self.RHSVel(0))
        deltaVel = -Jacobian.T @ solution
        if DebugArrays == True:
            ST.Mess("Velocity Array: ")
            ST.PrintNp1D(True, velArrayNp)
            ST.Mess("Velocity Correction Solution: ")
            ST.PrintNp1D(True, solution)
            ST.Mess("Delta velocity: ")
            ST.PrintNp1D(True, deltaVel)
            
        # Move corrected velocities back into the system
        for bodyIndex in range(1, self.numBodies):
            self.NPworldCoGDot[bodyIndex, 0] += deltaVel[(bodyIndex-1)*3]
            self.NPworldCoGDot[bodyIndex, 1] += deltaVel[(bodyIndex-1)*3 + 1]
            self.NPphiDot[bodyIndex] += deltaVel[(bodyIndex-1)*3 + 2]
        # Report corrected coordinates and velocities
        if DebugArrays == True:
            ST.Mess("Corrected Positions: [mm]")
            ST.PrintNp2D(self.NPworldCoG)
            ST.Mess("Corrected Phi:")
            ST.PrintNp1Ddeg(True, self.NPphi)
            ST.Mess("Corrected Velocities [mm/s]:")
            ST.PrintNp2D(self.NPworldCoGDot)
            ST.Mess("Corrected Angular Velocities:")
            ST.PrintNp1Ddeg(True, self.NPphiDot)

        ##############################
        # START OF THE SOLUTION PROPER
        ##############################
        # Pack coordinates and velocities into the NumPy NParray
        NParray = np.zeros((self.numMovBodiesx3 * 2,), dtype=np.float64)
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            NParray[index1:index1+2] = self.NPworldCoG[bodyIndex]
            NParray[index1+2] = self.NPphi[bodyIndex]
            NParray[index2:index2+2] = self.NPworldCoGDot[bodyIndex]
            NParray[index2+2] = self.NPphiDot[bodyIndex]
            index1 += 3
            index2 += 3

        # Set up the list of time intervals over which to integrate
        self.Tspan = np.arange(0.0, self.simEnd, self.simDelta)

        # ###################################################################################
        # Matrix Integration Function
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
        # ###################################################################################
        # scipy.integrate.solve_ivp
        # INPUTS:
        #       fun,                      Function name
        #       t_span,                   (startTime, endTime)
        #       y0,                       Initial values array [NParray]
        #       method='RK45',            RK45 | RK23 | DOP853 | Radau | BDF | LSODA
        #       t_eval=None,              times to evaluate at
        #       dense_output=False,       continuous solution or not
        #       events=None,              events to track
        #       vectorized=False,         whether fun is vectorized (i.e. parallelized)
        #       args=None,
        #       first_step=None,          none means algorithm chooses
        #       max_step=inf,             default is inf
        #       rtol=1e-3, atol=1e-6      relative and absolute tolerances
        #       jacobian,                 required for Radau, BDF and LSODA
        #       jac_sparsity=None,        to help algorithm when it is sparse
        #       lband=inf, uband=inf,     lower and upper bandwidth of Jacobian
        #       min_step=0                minimum step (required for LSODA)
        # RETURNS:
        #       t                         time array
        #       y                         values array
        #       sol                       instance of ODESolution (when dense_output=True)
        #       t_events                  array of event times
        #       y_events                  array of values at the event_times
        #       nfev                      number of times the rhs was evaluated
        #       njev                      number of times the Jacobian was evaluated
        #       nlu                       number of LU decompositions
        #       status                    -1 integration step failure | +1 termination event | 0 Successful
        #       message                   Human readable error message
        #       success                   True if 0 or +1 above
        # ###################################################################################

        # Integrate the equations: 
        #   <Dynamics function> 
        #   (<start time>, <end time>) 
        #   <position & velocity array> 
        #   <times at which to evaluate> 
        #   <relative Tolerance> 
        #   <absolute Tolerance>
        solution = solve_ivp(self.Dynamics,
                             (0.0, self.simEnd),
                             NParray,
                             t_eval=self.Tspan,
                             rtol=self.relativeTolerance,
                             atol=self.absoluteTolerance)

        # Output the positions/angles velocities results file
        self.PosFILE = open(os.path.join(self.solverObj.Directory, "SimAnimation.csv"), 'w')
        Sol = solution.y.T
        for tick in range(len(solution.t)):
            self.PosFILE.write(str(solution.t[tick])+" ")
            for body in range(1, self.numBodies):
                self.PosFILE.write(str(Sol[tick, body * 6 - 6]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 6 - 5]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 6 - 4]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 6 - 3]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 6 - 2]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 6 - 1]) + " ")
            self.PosFILE.write("\n")
        self.PosFILE.close()

        # Save the most important stuff into the solver object
        BodyNames = []
        for bodyObject in self.bodyGroup:
            BodyNames.append(bodyObject.Name)
        self.solverObj.BodyNames = BodyNames
        self.solverObj.DeltaTime = self.simDelta

        # Flag that the results are valid
        ST.getsimGlobalObject().SimResultsValid = True

        if self.solverObj.FileName != "-":
            self.outputResults(solution.t, solution.y.T)
            
    ##########################################
    #   This is the end of the actual solution
    #    The rest are all called subroutines
    ##########################################
    #  -------------------------------------------------------------------------
    def Dynamics(self, tick, NParray):
        """The Dynamics function which takes a 1D NParray 
        consisting of all the body world 3vectors concatenated to body velocity 3vectors
        and applies the physics of its movement to it,
        returning with a new NPDotArray
        """
        if DebugArrays == True:
            ST.Mess("Input to 'Dynamics'")
            ST.PrintNp1D(True, NParray)

        # Unpack NParray into world coordinate and world velocity sub-arrays
        # [X, Y, phi, ..... velX, velY, phidot .....]
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            self.NPworldCoG[bodyIndex, 0] = NParray[index1]
            self.NPworldCoG[bodyIndex, 1] = NParray[index1+1]
            self.NPphi[bodyIndex] = NParray[index1+2]
            self.NPworldCoGDot[bodyIndex, 0] = NParray[index2]
            self.NPworldCoGDot[bodyIndex, 1] = NParray[index2+1]
            self.NPphiDot[bodyIndex] = NParray[index2+2]
            index1 += 3
            index2 += 3

        if DebugArrays == True:
            ST.Mess("Dynamics - World CoG:")
            ST.PrintNp2D(self.NPworldCoG)
            ST.Mess("Dynamics - Phi:")
            ST.PrintNp1Ddeg(True, self.NPphi)
            ST.Mess("Dynamics - Velocity CoG:")
            ST.PrintNp2D(self.NPworldCoGDot)
            ST.Mess("Dynamics - Angular Velocity:")
            ST.PrintNp1Ddeg(True, self.NPphiDot)

        # Update the point stuff accordingly
        self.updatePointPositions()
        self.updatePointVelocities()

        # generate an array of currently applied forces
        self.makeForceArray()

        # find the accelerations ( a = F / m )
        accel = []
        # If we have no constraints, the bodies just move subject to the forces
        if self.numConstraints == 0:
            for index in range(self.numMovBodiesx3):
                accel.append = self.NPforceArray[index]/self.NPMasskgArray[index]
                
        # We go through this process if we have any constraints
        else:
            Jacobian = self.GetJacobian()
            if DebugArrays == True:
                ST.Mess("Dynamics - Jacobian")
                ST.PrintNp2D(Jacobian)

            # Create the Jacobian-Mass-Jacobian matrix
            # [ diagonal masses ---- negative Jacobian transpose ]
            # [    |                             |               ]
            # [  Jacobian      ------          Zeros             ]
            numBodPlusConstr = self.numMovBodiesx3 + self.numConstraints
            JacMasJac = np.zeros((numBodPlusConstr, numBodPlusConstr), dtype=np.float64)
            JacMasJac[0: self.numMovBodiesx3, 0: self.numMovBodiesx3] = np.diag(self.NPMassArray)
            JacMasJac[self.numMovBodiesx3:, 0: self.numMovBodiesx3] = Jacobian
            JacMasJac[0: self.numMovBodiesx3, self.numMovBodiesx3:] = -Jacobian.T
            if DebugArrays == True:
                ST.Mess("Dynamics - Jacobian-MassDiagonal-JacobianT Array")
                ST.PrintNp2D(JacMasJac)

            # get r-h-s of acceleration constraints at this time
            rhsAccel = self.RHSAcc(tick)
            if DebugArrays == True:
                ST.Mess("Dynamics - rhsAccel:")
                ST.PrintNp1D(True, rhsAccel)
                
            # Combine Force Array and rhs of Acceleration constraints into one 1D array
            rhs = np.zeros((numBodPlusConstr,), dtype=np.float64)
            rhs[0: self.numMovBodiesx3] = self.NPforceArray
            rhs[self.numMovBodiesx3:] = rhsAccel
            if DebugArrays == True:
                ST.Mess("Dynamics - rhs:")
                ST.PrintNp1D(True, rhs)
                
            # Solve the JacMasJac augmented with the rhs
            solvedVector = np.linalg.solve(JacMasJac, rhs)
            
            # First half of solution are the acceleration values
            accel = solvedVector[: self.numMovBodiesx3]
            
            # Second half is Lambda which is reported in the output results routine
            self.Lambda = solvedVector[self.numMovBodiesx3:]
            if DebugArrays == True:
                ST.Mess("Dynamics - Accelerations: ")
                ST.PrintNp1D(True, accel)
                ST.Mess("Dynamics - Lambda: ")
                ST.PrintNp1D(True, self.Lambda)
                ST.Mess("#"*80)

        # Transfer the velocity/accelerations into the
        # worldCoGDotDot/phiDotDot and DotArray/DotDotArray
        # [velX, velY, phidot, ..... accX, accY, phidotdot .....]
        NPDotArray = np.zeros((self.numMovBodiesx3 * 2,), dtype=np.float64)
        for bodyIndex in range(1, self.numBodies):
            accelIndex = (bodyIndex-1) * 3
            self.NPworldCoGDotDot[bodyIndex] = accel[accelIndex], accel[accelIndex + 1]
            self.NPphiDotDot[bodyIndex] = accel[accelIndex + 2]
            
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            NPDotArray[index1:index1+2] = self.NPworldCoGDot[bodyIndex]
            NPDotArray[index1+2] = self.NPphiDot[bodyIndex]
            NPDotArray[index2:index2+2] = self.NPworldCoGDotDot[bodyIndex]
            NPDotArray[index2+2] = self.NPphiDotDot[bodyIndex]
            index1 += 3
            index2 += 3

        # Increment number of function evaluations
        self.Counter += 1

        return NPDotArray
    #  -------------------------------------------------------------------------
    def correctInitialConditions(self):
        """This function corrects the supplied initial conditions by making
        the body coordinates and velocities consistent with the constraints"""

        """
        if Debug:
            ST.Mess("SimMain-correctInitialConditions")
        # Try Newton-Raphson iteration for n up to 20
        for n in range(20):
            # Update the points positions
            self.updatePointPositions()

            # Evaluate Deltaconstraint of the constraints at time=0
            Deltaconstraints = self.Getconstraints(0)
            if DebugArrays == True: #Debug:
                ST.Mess("Delta constraints Result:")
                ST.PrintNp1D(True, Deltaconstraints)
                
            # Evaluate Jacobian
            Jacobian = self.GetJacobian()
            if DebugArrays == True: #Debug:
                ST.Mess("Jacobian:")
                ST.PrintNp2D(Jacobian)

            # Determine any redundancy between constraints
            redundant = np.linalg.matrix_rank(Jacobian) 
            if redundant < self.numConstraints:
                CAD.Console.PrintError('The constraints exhibit Redundancy\n')
                return False

            # We have successfully converged if the ||Deltaconstraint|| is very small
            DeltaconstraintLengthSq = 0
            for index in range(self.numConstraints):
                DeltaconstraintLengthSq += Deltaconstraints[index] ** 2
            if Debug:
                ST.Mess("Total constraint Error: " + str(math.sqrt(DeltaconstraintLengthSq)))
            if DeltaconstraintLengthSq < 1.0e-16:
                return True

            # Solve for the new corrections
            solution = np.linalg.solve(Jacobian @ Jacobian.T, Deltaconstraints)
            delta = - Jacobian.T @ solution
            # Correct the estimates
            for bodyIndex in range(1, self.numBodies):
                self.NPworldCoG[bodyIndex, 0] += delta[(bodyIndex-1)*3]
                self.NPworldCoG[bodyIndex, 1] += delta[(bodyIndex-1)*3+1]
                self.NPphi[bodyIndex] += delta[(bodyIndex-1)*3+2]
                
        CAD.Console.PrintError("Newton-Raphson Correction failed to converge\n\n")
        """
        return True
    #  -------------------------------------------------------------------------
    def updatePointPositions(self):
        """Here we update the positions as the bodies move and rotate"""

        for bodyIndex in range(1, self.numBodies):

            # Compute the Rotation Matrix
            self.NPRotMatrixPhi[bodyIndex] = ST.CalculateRotationMatrix(self.NPphi[bodyIndex])

            if DebugArrays == True:
                ST.Mess("Update Point Positions:")
                ST.MessNoLF("In Xi-Eta Coordinates           ")
                ST.MessNoLF("Relative to CoG                 ")
                ST.MessNoLF("Relative to CoG Rotated 90      ")
                ST.Mess("World Coordinates               ")
            for pointIndex in range(self.NPnumJointPointsInBody[bodyIndex]):
                pointVector = self.NPRotMatrixPhi[bodyIndex] @ self.NPpointXiEta[bodyIndex, pointIndex]
                self.NPpointXYrelCoG[bodyIndex, pointIndex] = pointVector
                self.NPpointXYrelCoGRot[bodyIndex][pointIndex] = ST.Rot90NumPy(pointVector)
                self.NPpointWorld[bodyIndex, pointIndex] = self.NPworldCoG[bodyIndex] + pointVector
                if DebugArrays == True:
                    ST.PrintNp1D(False, self.NPpointXiEta[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.NPpointXYrelCoG[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.NPpointXYrelCoGRot[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(True, self.NPpointWorld[bodyIndex][pointIndex])
        # Next bodyIndex
    #  -------------------------------------------------------------------------
    def updatePointVelocities(self):
        if Debug:
            ST.Mess("SimMain-updatePointVelocities")
        for bodyIndex in range(1, self.numBodies):
            for pointIndex in range(self.NPnumJointPointsInBody[bodyIndex]):
                deltaVelVector = self.NPpointXYrelCoGRot[bodyIndex, pointIndex] * self.NPphiDot[bodyIndex]
                self.NPpointXYrelCoGDot[bodyIndex, pointIndex] = deltaVelVector
                self.NPpointWorldDot[bodyIndex, pointIndex] = self.NPworldCoGDot[bodyIndex] + deltaVelVector
        # for forceObj in self.NPforceObjList:
        #   if forceObj.forceType != 0:
        #        if forceObj.bodyHeadIndex != 0:
        #            forceObj.FUnit_I_WorldDot = ST.Rot90NumPy(forceObj.FUnit_I_World) * self.NPphiDot[forceObj.bodyHeadIndex]
    #  -------------------------------------------------------------------------
    def Getconstraints(self, tick):
        """Returns a numConstraints-long vector which contains the current deviation
        from the defined constraints"""
        if Debug:
            ST.Mess("SimMain-constraints")

        deltaConstraintNp = np.zeros((self.numConstraints,), dtype=np.float64)
        
        # Call the applicable function which is pointed to by the constraint function dictionary
        for jointObj in self.jointGroup:
            #if jointObj.SimJoint == "Revolute" and jointObj.FunctType != -1:
            #    setattr(jointObj, "SimJoint", "Driven-Revolute")
            constraint = self.dictconstraintFunctions[jointObj.SimJoint](jointObj, tick)
            deltaConstraintNp[jointObj.rowStart: jointObj.rowEnd] = constraint

        return deltaConstraintNp
    #  =========================================================================
    def GetJacobian(self):
        """Returns the Jacobian matrix numConstraints X (3 x numMovBodies)"""
        if Debug:
            ST.Mess("SimMain-Jacobian")
        Jacobian = np.zeros((self.numConstraints, self.numMovBodiesx3,))
        for jointObj in self.jointGroup:
            if ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                # Call the applicable function which is pointed to by the Jacobian dictionary
                #if jointObj.SimJoint == "Revolute" and jointObj.FunctType != -1:
                #    setattr(jointObj, "SimJoint", "Driven-Revolute")
                JacobianHead, JacobianTail = self.dictJacobianFunctions[ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint]](jointObj)
                # Fill in the values in the Jacobian
                if jointObj.bodyHeadIndex != 0:
                    columnHeadStart = (jointObj.bodyHeadIndex-1) * 3
                    columnHeadEnd = jointObj.bodyHeadIndex * 3
                    Jacobian[jointObj.rowStart: jointObj.rowEnd, columnHeadStart: columnHeadEnd] = JacobianHead
                if jointObj.bodyTailIndex != 0:
                    columnTailStart = (jointObj.bodyTailIndex-1) * 3
                    columnTailEnd = jointObj.bodyTailIndex * 3
                    Jacobian[jointObj.rowStart: jointObj.rowEnd, columnTailStart: columnTailEnd] = JacobianTail
        return Jacobian
    #  =========================================================================
    def RHSAcc(self, tick):
        """Returns a numConstraints-long vector containing gamma"""
        if Debug: ST.Mess("SimMain-RHSAcc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    rhs = zeros(nConst,1);
        # for Ji = 1:nJ
        #    switch (Joints(Ji).type);
        #        case {'rev'}
        #            A_rev
        #        case {'tran'}
        #            A_tran
        #        case {'rev-rev'}
        #            A_rev_rev
        #        case {'rev-tran'}
        #            A_rev_tran
        #        case {'Fixed'}
        #            A_Fixed
        #        case {'disc'}
        #            A_disc
        #        case {'rel-rot'}
        #            A_rel_rot
        #        case {'rel-tran'}
        #            A_rel_tran
        #    end
        #        rs = Joints(Ji).rows;
        #        re = Joints(Ji).rowe;
        #        rhs(rs:re) = f;
        # end
        # ==================================
        # Determine the Right-Hand-Side of the acceleration equation (gamma)
        rhsAcc = np.zeros((self.numConstraints,), dtype=np.float64)

        # Call the applicable function which is pointed to by the Acceleration function dictionary
        for jointObj in self.jointGroup:
            if ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                #if jointObj.SimJoint == "Revolute" and jointObj.FunctType != -1:
                #    setattr(jointObj, "SimJoint", "Driven-Revolute")
                gamma = self.dictAccelerationFunctions[ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint]](jointObj, tick)
                rhsAcc[jointObj.rowStart: jointObj.rowEnd] = gamma
        return rhsAcc
    #  -------------------------------------------------------------------------
    def RHSVel(self, tick):
        if Debug: ST.Mess("SimMain-RHSVel")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #     rhs = zeros(nConst,1);
        # for Ji = 1:nJ
        #    switch (Joints(Ji).type);
        #        case {'rel-rot'}
        #            [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #            f = fun_d;
        #            rhs(Joints(Ji).rows:Joints(Ji).rowe) = f;
        #        case {'rel-tran'}
        #            [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #            f = fun*fun_d;
        #            rhs(Joints(Ji).rows:Joints(Ji).rowe) = f;
        #    end
        # end
        # ==================================
        # Call the applicable Driven-Revolute or Driven-Translation function where applicable
        rhsVelNp = np.zeros((self.numConstraints,), dtype=np.float64)
        """
        for jointObj in self.jointGroup:
            if jointObj.SimJoint == "Revolute" and jointObj.FunctType != -1:
                setattr(jointObj, "SimJoint", "Driven-Rotation")
                [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
                rhsVelNp[jointObj.rowStart: jointObj.rowEnd] = func * funcDot
            elif jointObj.SimJoint == "Driven-Translation":
                [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
                rhsVelNp[jointObj.rowStart: jointObj.rowEnd] = funcDot
                """
        return rhsVelNp
    #  =========================================================================
    def Revolute_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Revolute joint"""
        if Debug: ST.Mess("SimMain-Revolute_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #
        #    f = Points(Pi).rP - Points(Pj).rP;
        #    if Joints(Ji).fix == 1
        #        Bi = Joints(Ji).iBindex;
        #        Bj = Joints(Ji).jBindex;
        #        if Bi == 0
        #        f = [f
        #            (- Bodies(Bj).p - Joints(Ji).p0)];
        #        elseif Bj == 0
        #            f = [f
        #            (Bodies(Bi).p - Joints(Ji).p0)];
        #        else
        #            f = [f
        #            (Bodies(Bi).p - Bodies(Bj).p - Joints(Ji).p0)];
        #        end
        #    end
        # ==================================
        constraintNp = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                       self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        if DebugArrays == True:
            ST.Mess('Revolute Constraint:')
            ST.MessNoLF('    Point I: ')
            ST.PrintNp1D(True, self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])
            ST.MessNoLF('    Point J: ')
            ST.PrintNp1D(True, self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex])
            ST.MessNoLF('    Difference Vector: ')
            ST.PrintNp1D(True, constraintNp)
        if jointObj.fixDof:
            if jointObj.bodyHeadIndex == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (-self.NPphi[jointObj.bodyTailIndex] - self.NPphi0[jointObj.jointIndex])])
            elif jointObj.bodyTailIndex == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.NPphi[jointObj.bodyHeadIndex] - self.NPphi0[jointObj.jointIndex])])
            else:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.NPphi[jointObj.bodyHeadIndex]
                                          - self.NPphi[jointObj.bodyTailIndex]
                                          - self.NPphi0[jointObj.jointIndex])])
        return constraintNp
    #  -------------------------------------------------------------------------
    def Revolute_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Revolute joint
        if Debug:
            ST.Mess("SimMain-Revolute_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        # Pi = Joints(Ji).iPindex;
        # Pj = Joints(Ji).jPindex;
        #
        #    Di = [ eye(2)  Points(Pi).sP_r];
        #    Dj = [-eye(2) -Points(Pj).sP_r];
        #
        #    if Joints(Ji).fix == 1
        #        Di = [Di
        #              0  0  1];
        #        Dj = [Dj
        #              0  0 -1];
        #    end
        # ==================================
        if jointObj.fixDof is False:
            if DebugArrays == True:
                ST.Mess("NPpointXYrelCoGRot")
                ST.PrintNp3D(self.NPpointXYrelCoGRot)
            JacobianHead = np.array([
                [1.0, 0.0, self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex, 0]],
                [0.0, 1.0, self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex, 1]]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex, 0]],
                [0.0, -1.0, -self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex, 1]]])
        else:
            JacobianHead = np.array([
                [1.0, 0.0, self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex, 0]],
                [0.0, 1.0, self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex, 1]],
                [0.0, 0.0, 1.0]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex, 0]],
                [0.0, -1.0, -self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex, 1]],
                [0.0, 0.0, -1.0]])
        if DebugArrays == True:
            ST.Mess("Jacobians")
            ST.PrintNp2D(JacobianHead)
            ST.PrintNp2D(JacobianTail)

        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Revolute_Acc(self, jointObj, tick):
        # Evaluate gamma for a Revolute joint
        if Debug: ST.Mess("SimMain-Revolute_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        # Pi = Joints(Ji).iPindex;
        # Pj = Joints(Ji).jPindex;
        # Bi = Points(Pi).Bindex;
        # Bj = Points(Pj).Bindex;
        #
        # if Bi == 0
        #    f = s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d;
        # elseif Bj == 0
        #    f = -s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d;
        # else
        #    f = -s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d + ...
        #         s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d;
        # end
        #
        #    if Joints(Ji).fix == 1
        #       f = [f
        #            0];
        #    end
        # ==================================
        if jointObj.bodyHeadIndex == 0:
            gammaNp = (ST.Rot90NumPy(
                self.NPpointXYrelCoGDot[jointObj.bodyTailIndex,
                        jointObj.pointTailIndex]) *
                        self.NPphiDot[jointObj.bodyTailIndex])
        elif jointObj.bodyTailIndex == 0:
            gammaNp = (-ST.Rot90NumPy(
                self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex,
                        jointObj.pointHeadIndex]) *
                        self.NPphiDot[jointObj.bodyHeadIndex])
        else:
            gammaNp = (-ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex,
                        jointObj.pointHeadIndex]) *
                        self.NPphiDot[jointObj.bodyHeadIndex] +\
                       ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex,
                        jointObj.pointTailIndex]) *
                        self.NPphiDot[jointObj.bodyTailIndex])
        if jointObj.fixDof:
            gammaNp = np.array([gammaNp[0], gammaNp[1], 0.0])

        return gammaNp
    #  =========================================================================
    """
    def Revolute_Revolute_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Revolute-Revolute joint
        if Debug:
            ST.Mess("SimMain-Revolute_Revolute_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    L = Joints(Ji).L;
        #    u = d/L;
        #       f = (u'*d - L)/2;
        # ==================================
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        Length = jointObj.lengthLink
        jointUnitVec = diff / Length
        return np.array([(jointUnitVec.dot(diff) - Length) / 2.0])
        """
    #  -------------------------------------------------------------------------
    """
    def Revolute_Revolute_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Revolute-Revolute joint
        if Debug:
            ST.Mess("SimMain-Revolute_Revolute_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    L = Joints(Ji).L;
        #    u = d/L;
        #        Di = [ u'  u'*Points(Pi).sP_r];
        #        Dj = [-u' -u'*Points(Pj).sP_r];
        # ==================================
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        jointUnitVec = diff / jointObj.lengthLink

        JacobianHead = np.array([jointUnitVec[0], jointUnitVec[1],
                                 jointUnitVec.dot(self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])])
        JacobianTail = np.array([-jointUnitVec[0], -jointUnitVec[1],
                                 -jointUnitVec.dot(self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex])])
        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Revolute_Revolute_Acc(self, jointObj, tick):
        # Evaluate gamma for a Revolute-Revolute joint
        if Debug:
            ST.Mess("SimMain-Revolute_Revolute_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    d_d  = Points(Pi).rP_d - Points(Pj).rP_d;
        #
        #    L = Joints(Ji).L;
        #    u = d/L;
        #    u_d = d_d/L;
        #
        #        f = - u_d'*d_d;
        #    if Bi == 0
        #        f = f + u'*s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d;
        #    elseif Bj == 0
        #        f = f - u'*s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d;
        #    else
        #        f = f - u'*(s_rot(Points(Pi).sP_d*Bodies(Bi).p_d - ...
        #                          Points(Pj).sP_d*Bodies(Bj).p_d));
        #    end
        # ==================================
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        diffDot = self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                  self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        jointUnitVec = diff/jointObj.lengthLink
        jointUnitVecDot = diffDot/jointObj.lengthLink
        f = -jointUnitVecDot.dot(diffDot)
        if jointObj.bodyHeadIndex == 0:
            f += jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]) *
                               self.NPphiDot[jointObj.bodyTailIndex])
        elif jointObj.bodyTailIndex == 0:
            f -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex]) *
                               self.NPphiDot[jointObj.bodyHeadIndex])
        else:
            f -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] *
                                             self.NPphiDot[jointObj.bodyHeadIndex] +
                                             self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]) *
                                             self.NPphiDot[jointObj.bodyTailIndex])
        return f
        """
    #  =========================================================================
    def Fixed_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Fixed joint
        if Debug:
            ST.Mess("SimMain-Fixed_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #
        #    if Bi == 0
        #        f = [ -(Bodies(Bj).r + Bodies(Bj).A*Joints(Ji).d0)
        #              -Bodies(Bj).p - Joints(Ji).p0];
        #    elseif Bj == 0
        #        f = [Bodies(Bi).r - Joints(Ji).d0
        #             Bodies(Bi).p - Joints(Ji).p0];
        #    else
        #        f = [Bodies(Bi).r - (Bodies(Bj).r + Bodies(Bj).A*Joints(Ji).d0)
        #             Bodies(Bi).p - Bodies(Bj).p - Joints(Ji).p0];
        #    end
        # ==================================
        if jointObj.bodyHeadIndex == 0:
            return np.array([-(self.NPworldCoG[jointObj.bodyTailIndex] +
                               self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ ST.CADVecToNumPy(self.NPd0[jointObj.jointIndex])),
                             -self.NPphi[jointObj.bodyTailIndex] - self.NPphi0])
        elif jointObj.bodyTailIndex == 0:
            return np.array([self.NPworldCoG[jointObj.bodyHeadIndex] - ST.CADVecToNumPy(self.NPd0[jointObj.jointIndex]),
                             self.NPphi[jointObj.bodyHeadIndex] - self.NPphi0])
        else:
            return np.array([self.NPworldCoG[jointObj.bodyHeadIndex] -
                             (self.NPworldCoG[jointObj.bodyTailIndex] +
                              self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ ST.CADVecToNumPy(self.NPd0[jointObj.jointIndex])),
                             self.NPphi[jointObj.bodyHeadIndex] -
                             self.NPphi[jointObj.bodyTailIndex] -
                             self.NPphi0])
    #  -------------------------------------------------------------------------
    def Fixed_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Fixed joint
        if Debug:
            ST.Mess("SimMain-Fixed_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bj = Joints(Ji).jBindex;
        #
        #    Di = eye(3);
        #    if Bj ~= 0
        #        Dj = [-eye(2) -s_rot(Bodies(Bj).A*Joints(Ji).d0)
        #               0  0   -1];
        #    end
        # ==================================
        JacobianHead = np.array([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        JacobianTail = np.array([[-1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0],
                                 [0.0, 0.0, -1.0]])
        if jointObj.bodyTailIndex != 0:
            tailVector = ST.Rot90NumPy(self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ self.NPd0[jointObj.jointIndex])
            JacobianTail = np.array([[-1.0, 0.0, -tailVector[0]],
                                     [0.0, -1.0, -tailVector[1]],
                                     [0.0, 0.0, -1.0]])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Fixed_Acc(self, jointObj, tick):
        # Evaluate gamma for a Fixed joint
        if Debug:
            ST.Mess("SimMain-Fixed_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bj = Joints(Ji).jBindex;
        #
        #    f = [0; 0; 0];
        #    if Bj ~= 0
        #        f = [-Bodies(Bj).A*Joints(Ji).d0*Bodies(Bj).p_d^2; 0];
        #
        #    end
        # ==================================
        if jointObj.bodyTailIndex != 0:
            tailVector = -self.NPRotMatrixPhi[jointObj.bodyTailIndex] @ (self.NPd0[jointObj.jointIndex] *
                                                                     (self.NPphiDot[jointObj.bodyTailIndex]**2))
            return np.array([tailVector[0], tailVector[1], 0.0])
        else:
            return np.array([0.0, 0.0, 0.0])
    #  =========================================================================
    """
    def Translational_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Translational joint
        if Debug:
            ST.Mess("SimMain-Translational_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #
        #    uj_r = Uvectors(Joints(Ji).jUindex).u_r;
        #    ui = Uvectors(Joints(Ji).iUindex).u;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #
        #    f  = [uj_r'*d; uj_r'*ui];
        #
        #    if Joints(Ji).fix == 1
        #        f = [f
        #            (ui'*d - Joints(Ji).p0)/2];
        #    end
        # ==================================
        jointUnitJRot = self.NPjointUnit_J_WorldRot[jointObj.JointNumber]
        jointUnitIVec = self.NPjointUnit_I_World[jointObj.JointNumber]
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        if Debug:
            ST.Mess('Translational Constraint:')
            ST.MessNoLF('    Unit I Vector: ')
            ST.PrintNp1D(True, jointUnitIVec)
            ST.MessNoLF('    Unit J Vector Rotated: ')
            ST.PrintNp1D(True, jointUnitJRot)
            ST.MessNoLF('    World I: ')
            ST.PrintNp1D(True, self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])
            ST.MessNoLF('    World J: ')
            ST.PrintNp1D(True, self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex])
            ST.MessNoLF('    Difference vector: ')
            ST.PrintNp1D(True, diff)

        if jointObj.fixDof is False:
            if Debug:
                ST.MessNoLF('    Unit J vector Rotated . diff: ')
                ST.Mess(jointUnitJRot.dot(diff))
                ST.MessNoLF('    Unit J vector Rotated . Unit I Vector: ')
                ST.Mess(jointUnitJRot.dot(jointUnitIVec))
            return np.array([jointUnitJRot.dot(diff),
                             jointUnitJRot.dot(jointUnitIVec)])
        else:
            return np.array(
                [jointUnitJRot.dot(diff),
                 jointUnitJRot.dot(jointUnitIVec),
                 (jointUnitIVec.dot(diff) - jointObj.phi0) / 2])
        """
    #  -------------------------------------------------------------------------
    """
    def Translational_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Translational joint
        if Debug:
            ST.Mess("SimMain-Translational_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    uj = Uvectors(Joints(Ji).jUindex).u;
        #    uj_r = Uvectors(Joints(Ji).jUindex).u_r;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #
        #        Di = [ uj_r'  uj'*Points(Pi).sP
        #               0 0       1];
        #        Dj = [-uj_r' -uj'*(Points(Pj).sP + d)
        #               0 0      -1];
        #
        #    if Joints(Ji).fix == 1
        #        Di = [Di
        #              uj'  uj'*Points(Pi).sP_r];
        #        Dj = [Dj
        #              -uj' -uj'*Points(Pj).sP_r];
        #    end
        # ==================================
        jointUnitJVec = self.NPjointUnit_J_World[jointObj.JointNumber]
        jointUnitJRot = self.NPjointUnit_J_WorldRot[jointObj.JointNumber]
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]

        if jointObj.fixDof is False:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.NPpointXYrelCoG[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])],
                                     [0.0, 0.0, 1.0]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.NPpointXYrelCoG[jointObj.bodyTailIndex, jointObj.pointTailIndex] + diff)],
                                     [0.0, 0.0, -1.0]])
        else:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.NPpointXYrelCoG[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])],
                                     [0.0, 0.0, 1.0],
                                     [jointUnitJVec[0], jointUnitJVec[1],
                                      jointUnitJVec.dot(self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.NPpointXYrelCoG[jointObj.bodyTailIndex, jointObj.pointTailIndex] + diff)],
                                     [0.0, 0.0, -1.0],
                                     [-jointUnitJVec[0], -jointUnitJVec[1],
                                      -jointUnitJVec.dot(self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex])]])
        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Translational_Acc(self, jointObj, tick):
        # Evaluate gamma for a Translational joint
        if Debug:
            ST.Mess("SimMain-Translational_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #
        #    ujd = Uvectors(Joints(Ji).jUindex).u_d;
        #    ujd_r = s_rot(ujd);
        #
        #    if Bi == 0
        #        f2 = 0;
        #    elseif Bj == 0
        #        f2 = 0;
        #    else
        #        f2 = ujd'*(Bodies(Bi).r - Bodies(Bj).r)*Bodies(Bi).p_d - ...
        #             2*ujd_r'*(Bodies(Bi).r_d - Bodies(Bj).r_d);
        #    end
        #        f  = [f2; 0];
        #
        #    if Joints(Ji).fix == 1
        #        d    = Points(Pi).rP - Points(Pj).rP;
        #        d_d  = Points(Pi).rP_d - Points(Pj).rP_d;
        #        L = Joints(Ji).p0;
        #        u = d/L;
        #        u_d = d_d/L;
        #        f3 = - u_d'*d_d;
        #        if Bi == 0
        #            f3 = f3 + u'*s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d;
        #        elseif Bj == 0
        #            f3 = f3 - u'*s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d;
        #        else
        #            f3 = f3 - u'*(s_rot(Points(Pi).sP_d*Bodies(Bi).p_d - ...
        #                                Points(Pj).sP_d*Bodies(Bj).p_d));
        #        end
        #        f = [f; f3];
        #    end
        # ==================================
        jointUnitJDotVec = self.NPjointUnit_J_WorldDot[jointObj.JointNumber]
        jointUnitJDotRot = ST.Rot90NumPy(jointUnitJDotVec.copy())
        if jointObj.bodyHeadIndex == 0:
            f2 = 0
        elif jointObj.bodyTailIndex == 0:
            f2 = 0
        else:
            f2 = jointUnitJDotVec.dot(self.NPworldCoG[jointObj.bodyHeadIndex] -
                                      self.NPworldCoG[jointObj.bodyTailIndex]) * \
                 self.NPphiDot[jointObj.bodyHeadIndex] - \
                 2 * jointUnitJDotRot.dot(self.NPworldCoGDot[jointObj.bodyHeadIndex] -
                                          self.NPworldCoGDot[jointObj.bodyTailIndex])

        if jointObj.fixDof is False:
            return np.array([f2, 0.0])
        else:
            diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                   self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
            diffDot = self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                      self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]
            jointUnitVec = diff/jointObj.phi0
            jointUnitVecDot = diffDot/jointObj.phi0
            f3 = -jointUnitVecDot.dot(diffDot)
            if jointObj.bodyHeadIndex == 0:
                f3 += jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]) *
                                       self.NPphiDot[jointObj.bodyTailIndex])
            elif jointObj.bodyTailIndex == 0:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex]) *
                                       self.NPphiDot[jointObj.bodyHeadIndex])
            else:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] *
                                                     self.NPphiDot[jointObj.bodyHeadIndex] -
                                                     self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex] *
                                                     self.NPphiDot[jointObj.bodyTailIndex]))
            return np.array([f2, 0.0, f3])
            """
    #  =========================================================================
    """
    def Translational_Revolute_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Translational-Revolute joint
        if Debug:
            ST.Mess("SimMain-Translational_Revolute_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    ui_r = Uvectors(Joints(Ji).iUindex).u_r;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #        f = ui_r'*d - Joints(Ji).L;
        # ==================================
        jointUnitVecRot = self.NPjointUnitVecRot[jointObj.bodyHeadIndex]
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        return np.array([jointUnitVecRot.dot(diff) - jointObj.lengthLink])
        """
    #  -------------------------------------------------------------------------
    """
    def Translational_Revolute_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Translational-Revolute joint
        if Debug:
            ST.Mess("SimMain-Translational_Revolute_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    ui = Uvectors(Joints(Ji).iUindex).u;
        #    ui_r = Uvectors(Joints(Ji).iUindex).u_r;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #        Di = [ ui_r'  ui'*(Points(Pi).sP - d)];
        #        Dj = [-ui_r' -ui'*Points(Pj).sP];
        # ==================================
        jointUnitVec = self.NPjointUnit_I_World[jointObj.JointNumber]
        jointUnitVecRot = self.NPjointUnit_I_WorldRot[jointObj.JointNumber]
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]

        JacobianHead = np.array([jointUnitVecRot[0], jointUnitVecRot[1],
                                 jointUnitVec.dot(self.NPpointXYrelCoG[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - diff)])
        JacobianTail = np.array([-jointUnitVecRot[0], -jointUnitVecRot[1],
                                 -jointUnitVec.dot(self.NPpointXYrelCoG[jointObj.bodyTailIndex, jointObj.pointTailIndex])])

        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Translational_Revolute_Acc(self, jointObj, tick):
        # Evaluate gamma for a Translational-Revolute joint
        if Debug:
            ST.Mess("SimMain-Translational_Revolute_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #    ui  = Uvectors(Joints(Ji).iUindex).u;
        #    ui_d = Uvectors(Joints(Ji).iUindex).u_d;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    d_d  = Points(Pi).rP_d - Points(Pj).rP_d;
        #
        #    if Bi == 0
        #        f = ui'*Points(Pj).sP_d*Bodies(Bj).p_d;
        #    elseif Bj == 0
        #        f = ui_d'*(d*Bodies(Bi).p_d + 2*s_rot(d_d)) - ...
        #            ui'*Points(Pi).sP_d*Bodies(Bi).p_d;
        #    else
        #        f = ui_d'*(d*Bodies(Bi).p_d + 2*s_rot(d_d)) - ...
        #            ui'*(Points(Pi).sP_d*Bodies(Bi).p_d - ...
        #                  Points(Pj).sP_d*Bodies(Bj).p_d);
        #    end
        # ==================================
        jointUnitVec = self.NPjointUnit_I_World[jointObj.JointNumber]
        jointUnitVecDot = self.NPjointUnit_I_WorldDot[jointObj.JointNumber]
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        diffDot = self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                  self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        if jointObj.bodyHeadIndex == 0:
            f = jointUnitVec.dot(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex] *
                                 self.NPphiDot[jointObj.bodyTailIndex])
        elif jointObj.bodyTailIndex == 0:
            f = jointUnitVecDot.dot(diff * self.NPphiDot[jointObj.bodyHeadIndex] + 2 * ST.Rot90NumPy(diffDot)) - \
                jointUnitVec.dot(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] *
                                 self.NPphiDot[jointObj.bodyHeadIndex])
        else:
            f = jointUnitVecDot.dot(diff * self.NPphiDot[jointObj.bodyHeadIndex] + 2 * ST.Rot90NumPy(diffDot)) - \
                jointUnitVec.dot(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] *
                                 self.NPphiDot[jointObj.bodyHeadIndex] - \
                self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex] *
                                 self.NPphiDot[jointObj.bodyTailIndex])
        return f
        """
    #  =========================================================================
    """
    def Driven_Revolute_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Driven Revolute joint
        if Debug:
            ST.Mess("SimMain-Driven_Revolute_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #
        #    if Bi == 0
        #        f = -Bodies(Bj).p - fun;
        #    elseif Bj == 0
        #        f =  Bodies(Bi).p - fun;
        #    else
        #        f =  Bodies(Bi).p - Bodies(Bj).p - fun;
        #    end
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        if jointObj.bodyHeadIndex == 0:
            f = -self.NPphi[jointObj.bodyTailIndex] - func
        elif jointObj.bodyTailIndex == 0:
            f = self.NPphi[jointObj.bodyHeadIndex] - func
        else:
            f = self.NPphi[jointObj.bodyHeadIndex] - self.NPphi[jointObj.bodyTailIndex] - func
        return np.array([f])
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Revolute_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Driven Revolute joint
        if Debug:
            ST.Mess("SimMain-Driven_Revolute_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Di = [0 0  1];
        #    Dj = [0 0 -1];
        # ==================================
        JacobianHead = np.array([0.0, 0.0, 1.0])
        JacobianTail = np.array([0.0, 0.0, -1.0])
        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Revolute_Acc(self, jointObj, tick):
        # Evaluate gamma for a Driven Revolute joint
        if Debug:
            ST.Mess("SimMain-Driven_Revolute_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #    f = fun_dd;
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        return funcDotDot
        """
    #  =========================================================================
    """
    def Driven_Translational_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Driven Translational joint
        if Debug:
            ST.Mess("SimMain-Driven_Translational_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #        f = (d'*d - fun^2)/2;
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        return np.array([(diff.dot(diff) - func ** 2) / 2])
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Translational_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Driven Translational joint
        if Debug:
            ST.Mess("SimMain-Driven_Translational_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #        Di = [ d'  d'*Points(Pi).sP_r];
        #        Dj = [-d' -d'*Points(Pj).sP_r];
        # ==================================
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]

        JacobianHead = np.array([diff[0], diff[1],
                                 diff.dot(self.NPpointXYrelCoGRot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])])
        JacobianTail = np.array([-diff[0], -diff[1],
                                 -diff.dot(self.NPpointXYrelCoGRot[jointObj.bodyTailIndex, jointObj.pointTailIndex])])

        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Translational_Acc(self, jointObj, tick):
        # Evaluate gamma for a Driven Translational joint
        if Debug:
            ST.Mess("SimMain-Drven_Translational_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #    d  = Points(Pi).rP - Points(Pj).rP;
        #    d_d  = Points(Pi).rP_d - Points(Pj).rP_d;
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #
        #    f = fun*fun_dd + fun_d^2;
        #    if Bi == 0
        #        f = f + d'*s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d;
        #    elseif Bj == 0
        #        f = f - d'*s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d - d_d'*d_d;
        #    else
        #        f = f + d'*s_rot(Points(Pj).sP_d)*Bodies(Bj).p_d ...
        #              - d'*s_rot(Points(Pi).sP_d)*Bodies(Bi).p_d - d_d'*d_d;
        #    end
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        diff = self.NPpointWorld[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
               self.NPpointWorld[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        diffDot = self.NPpointWorldDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex] - \
                  self.NPpointWorldDot[jointObj.bodyTailIndex, jointObj.pointTailIndex]
        f = func * funcDotDot + funcDot**2
        if jointObj.bodyHeadIndex == 0:
            f += diff.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex])) * \
                 self.NPphiDot[jointObj.bodyTailIndex]
        elif jointObj.bodyTailIndex == 0:
            f -= diff.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])) * \
                 self.NPphiDot[jointObj.bodyHeadIndex] + \
                 diffDot.dot(diffDot)
        else:
            f += diff.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyTailIndex, jointObj.pointTailIndex])) * \
                 self.NPphiDot[jointObj.bodyTailIndex] - \
                 diff.dot(ST.Rot90NumPy(self.NPpointXYrelCoGDot[jointObj.bodyHeadIndex, jointObj.pointHeadIndex])) * \
                 self.NPphiDot[jointObj.bodyHeadIndex] - \
                 diffDot.dot(diffDot)
        return f
        """
    #  =========================================================================
    """
    def Disc_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Disc joint
        if Debug:
            ST.Mess("SimMain-Disc_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bi = Joints(Ji).iBindex;
        #    f = [(Bodies(Bi).r(2) - Joints(Ji).R)
        #         ((Bodies(Bi).r(1) - Joints(Ji).x0) + ...
        #           Joints(Ji).R*(Bodies(Bi).p - Joints(Ji).p0))];
        # ==================================
        return np.array([(self.NPworldCoG[jointObj.bodyHeadIndex, 1] - jointObj.Radius),
                         ((self.NPworldCoG[jointObj.bodyHeadIndex, 0] - jointObj.x0) +
                          jointObj.Radius * (self.NPphi[jointObj.bodyHeadIndex] - jointObj.phi0))])
        """
    #  -------------------------------------------------------------------------
    """
    def Disc_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Disc joint
        if Debug:
            ST.Mess("SimMain-Disc_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Di = [ 0  1  0
        #           1  0  Joints(Ji).R];
        JacobianHead = np.array([[0.0, 1.0, 0.0],
                                 [1.0, 0.0, jointObj.Radius]])
        return JacobianHead, JacobianHead
        """
    #  -------------------------------------------------------------------------
    """
    def Disc_Acc(self, jointObj, tick):
        #Evaluate gamma for a Disc joint
        if Debug:
            ST.Mess("SimMain-Disc_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    f = [0; 0];
        # ==================================
        return np.array([0.0, 0.0])
        """
    #  =========================================================================
    def makeForceArray(self):
        if Debug:
            ST.Mess("SimMainC - makeForceArray")

        # Reset all forces and moments to zero
        for bodyIndex in range(1, self.numBodies):
            self.NPsumForces[bodyIndex] = np.zeros((2,), dtype=np.float64)
            self.NPsumMoments[bodyIndex] = np.zeros((1,), dtype=np.float64)

        # Add up all the body force vectors for all the bodies
        forceIndex = -1
        for forceObj in self.forceList:
            forceIndex += 1
            if forceObj.forceType == "Gravity":
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'weight'}
                #            for Bi=1:nB
                #                Bodies(Bi).f = Bodies(Bi).f + Bodies(Bi).wgt;
                #            end
                for bodyIndex in range(1, self.numBodies):
                    self.NPsumForces[bodyIndex] += self.NPWeight[bodyIndex]
            else:
                CAD.Console.PrintError("Unknown Force type - this should never occur\n")
        # Next forceIndex

            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Spring"] or \
                    forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Linear Spring Damper"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'ptp'}
                # % Point-to-point spring-damper-actuator
                #  Pi = Forces(Fi).iPindex;
                #  Pj = Forces(Fi).jPindex;
                #  Bi = Forces(Fi).iBindex;
                #  Bj = Forces(Fi).jBindex;
                #  d  = Points(Pi).rP - Points(Pj).rP;
                #  d_dot = Points(Pi).rP_d - Points(Pj).rP_d;
                #  L  = sqrt(d'*d);
                #  L_dot = d'*d_dot/L;
                #  del = L - Forces(Fi).L0;
                #  u = d/L;
                #
                #  f = Forces(Fi).k*del + Forces(Fi).dc*L_dot + Forces(Fi).f_a;
                #  fi = f*u;
                #  if Bi ~= 0
                #    Bodies(Bi).f = Bodies(Bi).f - fi;
                #    Bodies(Bi).n = Bodies(Bi).n - Points(Pi).sP_r'*fi;
                #  end
                #  if Bj ~= 0
                #    Bodies(Bj).f = Bodies(Bj).f + fi;
                #    Bodies(Bj).n = Bodies(Bj).n + Points(Pj).sP_r'*fi;
                #  end

                diffNp = self.NPpointWorld[forceObj.bodyHeadIndex, forceObj.point_i_Index] - \
                       self.NPpointWorld[forceObj.bodyTailIndex, forceObj.point_j_Index]
                diffDotNp = self.NPpointWorldDot[forceObj.bodyHeadIndex, forceObj.point_i_Index] - \
                       self.NPpointWorldDot[forceObj.bodyTailIndex, forceObj.point_j_Index]
                length = np.sqrt(diffNp.dot(diffNp))
                lengthDot = (diffNp.dot(diffDotNp))/length
                delta = length - forceObj.LengthAngle0
                unitVecNp = diffNp/length
                # Find the component of the force in the direction of
                # the vector between the head and the tail of the force
                force = forceObj.Stiffness * delta + forceObj.DampingCoeff * lengthDot + forceObj.ForceMagnitude
                forceUnitNp = unitVecNp * force
                if forceObj.bodyHeadIndex != 0:
                    self.NPsumForces[forceObj.bodyHeadIndex] -= forceUnitNp
                    self.NPsumMoments[forceObj.bodyHeadIndex] -= (self.NPpointXYrelCoGRot[forceObj.bodyHeadIndex, forceObj.point_i_Index]).dot(forceUnitNp)
                if forceObj.bodyTailIndex != 0:
                    self.NPsumForces[forceObj.bodyTailIndex] += forceUnitNp
                    self.NPsumMoments[forceObj.bodyTailIndex] += self.NPpointXYrelCoGRot[forceObj.bodyTailIndex, forceObj.point_j_Index].dot(forceUnitNp)
                    """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring"] or \
                    forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring Damper"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'rot-sda'}
                # % Rotational spring-damper-actuator
                #
                #  Bi = Forces(Fi).iBindex;
                #  Bj = Forces(Fi).jBindex;
                #
                #    if Bi == 0
                #        theta   = -Bodies(Bj).p;
                #        theta_d = -Bodies(Bj).p_d;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bj).n = Bodies(Bj).n + T;
                #    elseif Bj == 0
                #        theta   = Bodies(Bi).p;
                #        theta_d = Bodies(Bi).p_d;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bi).n = Bodies(Bi).n - T;
                #    else
                #        theta   = Bodies(Bi).p - Bodies(Bj).p;
                #        theta_d = Bodies(Bi).p_d - Bodies(Bj).p_d;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bi).n = Bodies(Bi).n - T;
                #        Bodies(Bj).n = Bodies(Bj).n + T;
                #    end
                if forceObj.bodyHeadIndex == 0:
                    theta = -self.NPphi[forceObj.bodyTailIndex]
                    thetaDot = -self.NPphiDot[forceObj.bodyTailIndex]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.bodyTailIndex] += Torque
                elif forceObj.bodyTailIndex == 0:
                    theta = self.NPphi[forceObj.bodyHeadIndex]
                    thetaDot = self.NPphiDot[forceObj.bodyHeadIndex]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.bodyHeadIndex] -= Torque
                else:
                    theta = self.NPphi[forceObj.bodyHeadIndex] - self.NPphi[forceObj.bodyTailIndex]
                    thetaDot = self.NPphiDot[forceObj.bodyHeadIndex] - self.NPphiDot[forceObj.bodyTailIndex]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.bodyHeadIndex] -= Torque
                    self.NPsumMoments[forceObj.bodyTailIndex] += Torque
                    """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Unilateral Spring Damper"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                CAD.Console.PrintError("Still in development\n")
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Force Local to Body"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'flocal'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).f = Bodies(Bi).f + Bodies(Bi).A*Forces(Fi).flocal;
                self.NPsumForces[forceObj.bodyHeadIndex] += self.NPRotMatrixPhi[forceObj.bodyHeadIndex] @ forceObj.constLocalForce
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Global Force"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'f'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).f = Bodies(Bi).f + Forces(Fi).f;
                self.NPsumForces[forceObj.bodyHeadIndex, 0] += forceObj.constWorldForce[0]
                self.NPsumForces[forceObj.bodyHeadIndex, 1] += forceObj.constWorldForce[1]
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Torque about a Point"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'T'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).n = Bodies(Bi).n + Forces(Fi).T;
                self.NPsumMoments[forceObj.bodyHeadIndex] += forceObj.constTorque
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Contact Friction"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                CAD.Console.PrintError("Still in development\n")
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Motor"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                CAD.Console.PrintError("Still in development\n")
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Motor with Air Friction"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                CAD.Console.PrintError("Still in development\n")
                """

        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        # g = zeros(nB3,1);
        # for Bi = 1:nB
        #    ks = Bodies(Bi).irc; ke = ks + 2;
        #    g(ks:ke) = [Bodies(Bi).f; Bodies(Bi).n];
        # end
        # ==================================
        # The force array has three values for every body
        # x and y are the sum of forces and z is the sum of moments
        for bodyIndex in range(1, self.numBodies):
            self.NPforceArray[(bodyIndex - 1) * 3: bodyIndex * 3 - 1] = self.NPsumForces[bodyIndex]
            self.NPforceArray[bodyIndex * 3 - 1] = self.NPsumMoments[bodyIndex]

        if DebugArrays == True:
            ST.Mess("Force Array:  ")
            ST.PrintNp1D(True, self.NPforceArray)
    #  =========================================================================
    def initNumPyArrays(self, maxNumPoints):
        # Initialize all the NumPy arrays with zeros

        # Parameters for each body
        self.NPMasskg = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPmomentOfInertia = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPWeight = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPsumForces = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPsumMoments = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPworldCoG = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPworldCoGRot = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPworldCoGDot = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPworldCoGDotRot = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPworldCoGDotDot = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPphi = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPphiDot = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPphiDotDot = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPRotMatrixPhi = np.zeros((self.numBodies, 2, 2,), dtype=np.float64)
        self.NPnumJointPointsInBody = np.zeros((self.numBodies,), dtype=np.integer)

        self.NPphi0 = np.zeros((self.numJoints), dtype=np.float64)
        self.NPd0 = np.zeros((self.numJoints, 2), dtype=np.float64)

        self.NPpotEnergyZeroPoint = np.zeros((self.numBodies,), dtype=np.float64)

        # Parameters for each point within a body, for each body
        # Vector from CoG to the point in body local coordinates
        self.NPpointXiEta = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from CoG to the point in world coordinates
        self.NPpointXYrelCoG = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpointXYrelCoGRot = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpointXYrelCoGDot = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from the origin to the point in world coordinates
        self.NPpointWorld = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpointWorldRot = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpointWorldDot = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)

        # Unit vector (if applicable) of the first body of the joint in body local coordinates
        self.NPjointUnit_I_XiEta = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # Unit vector (if applicable) of the first body of the joint in world coordinates
        self.NPjointUnit_I_World = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_I_WorldRot = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_I_WorldDot = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_I_WorldDotRot = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # Second unit vector (if applicable) of the second body of the joint in body local coordinates
        self.NPjointUnit_J_XiEta = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # second unit vector (if applicable) of the second body of the joint in world coordinates
        self.NPjointUnit_J_World = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_J_WorldRot = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_J_WorldDot = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPjointUnit_J_WorldDotRot = np.zeros((self.numJoints, 2,), dtype=np.float64)

        self.NPforceArray = np.zeros((self.numMovBodiesx3,), dtype=np.float64)
    #  -------------------------------------------------------------------------
    def outputResults(self, timeValues, uResults):
        if Debug:
            ST.Mess("SimMain-outputResults")

        # Compute body accelerations, Lagrange multipliers, coordinates and
        #    velocity of all points, kinetic and potential energies,
        #             at every reporting time interval
        self.solverObj = CAD.ActiveDocument.findObjects(Name="^SimSolver$")[0]
        fileName = self.solverObj.Directory+"/"+self.solverObj.FileName+".csv"
        SimResultsFILE = open(fileName, 'w')
        numTicks = len(timeValues)

        ##################################################################3
        # THE HEADINGS
        # Create the vertical headings list
        # To write each body name into the top row of the spreadsheet,
        # would make some columns very big by default
        # So body names and point names are also written vertically in
        # The column before the body/point data is written
        VerticalHeaders = []
        # Write the column headers horizontally
        for threeLines in ["first", "second", "third"]:
            ColumnCounter = 0
            SimResultsFILE.write("Time: ")
            # Bodies Headings
            bodyIndex = -1
            for bodyObj in self.bodyGroup:
                bodyIndex += 1
                if bodyIndex != 0:
                    if threeLines == "first":
                        VerticalHeaders.append(bodyObj.Label)
                        SimResultsFILE.write("B" + str(bodyIndex))
                        SimResultsFILE.write(" x y phi phi dx dy dphi dphi d2x d2y d2phi d2phi ")
                    elif threeLines == "second":
                        SimResultsFILE.write(" - - - (rad) (deg) dt dt dt(r) dt(d) dt2 dt2 dt2(r) dt2(d) ")
                    else:
                        SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " -"*12 + " ")
                    ColumnCounter += 1
                    # Points Headings
                    for jointIndex in range(self.NPnumJointPointsInBody[bodyIndex]):
                        if threeLines == "first":
                            VerticalHeaders.append(bodyObj.JointNameList[jointIndex])
                            SimResultsFILE.write("J" + str(jointIndex+1) + " x y dx dy ")
                        elif threeLines == "second":
                            SimResultsFILE.write("- - - dt dt ")
                        else:
                            SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " -"*4 + " ")
                        ColumnCounter += 1
            # Next bodyObj

            # Lambda Headings
            if self.numConstraints > 0:
                bodyIndex = -1
                for bodyObj in self.bodyGroup:
                    bodyIndex += 1
                    if bodyIndex !=0:
                        if threeLines == "first":
                            VerticalHeaders.append(bodyObj.Label)
                            SimResultsFILE.write("Lam" + str(bodyIndex) + " x y ")
                        elif threeLines == "second":
                            SimResultsFILE.write("- - - ")
                        else:
                            SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - - ")
                        ColumnCounter += 1

            # Kinetic Energy Headings
            bodyIndex = -1
            for bodyObj in self.bodyGroup:
                bodyIndex += 1
                if bodyIndex !=0:
                    if threeLines == "first":
                        VerticalHeaders.append(bodyObj.Label)
                        SimResultsFILE.write("Kin" + str(bodyIndex) + " - ")
                    elif threeLines == "second":
                        SimResultsFILE.write("- - ")
                    else:
                        SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - ")
                    ColumnCounter += 1

            # Potential Energy Headings
            forceIndex = -1
            for forceObj in self.forceList:
                forceIndex += 1
                if forceObj.forceType == "Gravity":
                    bodyIndex = -1
                    for bodyObj in self.bodyGroup:
                        bodyIndex += 1
                        if bodyIndex != 0:
                            if threeLines == "first":
                                VerticalHeaders.append(bodyObj.Label)
                                SimResultsFILE.write("Pot" + str(bodyIndex) + " - ")
                            elif threeLines == "second":
                                SimResultsFILE.write("- - ")
                            else:
                                SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - ")
                            ColumnCounter += 1

            # Energy Totals Headings
            if threeLines == "first":
                SimResultsFILE.write("Total Total Total\n")
            elif threeLines == "second":
                SimResultsFILE.write("Kinet Poten Energy\n")
            else:
                SimResultsFILE.write(" - - -\n")
        # Next threeLines

        ##################################################################3
        # THE DATA
        # Do the calculations for each point in time
        # Plus an extra one at time=0 (with no printing)
        VerticalCounter = 0
        TickRange = [0]
        TickRange += range(numTicks)
        for timeIndex in TickRange:
            tick = timeValues[timeIndex]
            ColumnCounter = 0
            potEnergy = 0

            # Do the Dynamics on the stored uResults
            self.Dynamics(tick, uResults[timeIndex])

            # Write Time
            if timeIndex != 0:
                SimResultsFILE.write(str(tick) + " ")

            # Write All the Bodies position, positionDot, positionDotDot
            for bodyIndex in range(1, self.numBodies):
                if timeIndex != 0:
                    # Write Body Name vertically
                    if VerticalCounter < len(VerticalHeaders[ColumnCounter]):
                        character = VerticalHeaders[ColumnCounter][VerticalCounter]
                        if character in "0123456789":
                            SimResultsFILE.write("'" + character + "' ")
                        else:
                            SimResultsFILE.write(character + " ")
                    else:
                        SimResultsFILE.write("- ")

                    ColumnCounter += 1
                    # X Y
                    SimResultsFILE.write(str(self.NPworldCoG[bodyIndex])[1:-1:] + " ")
                    # Phi (rad)
                    SimResultsFILE.write(str(self.NPphi[bodyIndex])[1:-1:] + " ")
                    # Phi (deg)
                    SimResultsFILE.write(str(self.NPphi[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")
                    # Xdot Ydot
                    SimResultsFILE.write(str(self.NPworldCoGDot[bodyIndex])[1:-1:] + " ")
                    # PhiDot (rad)
                    SimResultsFILE.write(str(self.NPphiDot[bodyIndex])[1:-1:] + " ")
                    # PhiDot (deg)
                    SimResultsFILE.write(str(self.NPphiDot[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")
                    # Xdotdot Ydotdot
                    SimResultsFILE.write(str(self.NPworldCoGDotDot[bodyIndex])[1:-1:] + " ")
                    # PhiDotDot (rad)
                    SimResultsFILE.write(str(self.NPphiDotDot[bodyIndex])[1:-1:] + " ")
                    # PhiDotDot (deg)
                    SimResultsFILE.write(str(self.NPphiDotDot[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")

                # Write all the points position and positionDot in the body
                for index in range(self.NPnumJointPointsInBody[bodyIndex]):
                    if timeIndex != 0:
                        # Write Point Name vertically
                        if VerticalCounter < len(VerticalHeaders[ColumnCounter]):
                            character = VerticalHeaders[ColumnCounter][VerticalCounter]
                            if character in "0123456789":
                                SimResultsFILE.write("'" + character + "' ")
                            else:
                                SimResultsFILE.write(character + " ")
                        else:
                            SimResultsFILE.write("- ")

                        ColumnCounter += 1
                        # Point X Y
                        SimResultsFILE.write(str(self.NPpointWorld[bodyIndex, index])[1:-1:] + " ")
                        # Point Xdot Ydot
                        SimResultsFILE.write(str(self.NPpointWorldDot[bodyIndex, index])[1:-1:] + " ")
            # Next bodyIndex

            # Write the Lambdas
            if self.numConstraints > 0:
                if timeIndex != 0:
                    # Lambda
                    for bodyIndex in range(self.numBodies-1):
                        # Write the Body Name vertically
                        if VerticalCounter < len(VerticalHeaders[ColumnCounter]):
                            character = VerticalHeaders[ColumnCounter][VerticalCounter]
                            if character in "0123456789":
                                SimResultsFILE.write("'" + character + "' ")
                            else:
                                SimResultsFILE.write(character + " ")
                        else:
                            SimResultsFILE.write("- ")

                        ColumnCounter += 1
                        SimResultsFILE.write(str(self.Lambda[bodyIndex*2])[1:-1:] + " " + str(self.Lambda[bodyIndex*2 + 1])[1:-1:] + " ")

            # Compute kinetic and potential energies in Joules
            totKinEnergy = 0
            for bodyIndex in range(1, self.numBodies):
                kinEnergy = 0.5e-6 * (
                        (self.NPMassArray[(bodyIndex - 1) * 3] *
                         (self.NPworldCoGDot[bodyIndex, 0] ** 2 + self.NPworldCoGDot[bodyIndex, 1] ** 2)) +
                        (self.NPMassArray[(bodyIndex - 1) * 3 + 2] * (self.NPphiDot[bodyIndex] ** 2)))

                # Kinetic Energy (m^2 = mm^2 * 1e-6)
                if timeIndex != 0:
                    # Body Name vertically
                    if VerticalCounter < len(VerticalHeaders[ColumnCounter]):
                        character = VerticalHeaders[ColumnCounter][VerticalCounter]
                        if character in "0123456789":
                            SimResultsFILE.write("'" + character + "' ")
                        else:
                            SimResultsFILE.write(character + " ")
                    else:
                        SimResultsFILE.write("- ")
                    ColumnCounter += 1
                    SimResultsFILE.write(str(kinEnergy)[1:-1:] + " ")
                totKinEnergy += kinEnergy

            # Currently, calculate only gravitational potential energy
            totPotEnergy = 0
            forceIndex = -1
            for forceObj in self.forceList:
                forceIndex += 1
                # Potential Energy
                potEnergy = 0
                if forceObj.forceType == "Gravity":
                    for bodyIndex in range(1, self.numBodies):
                        potEnergy = -self.NPWeight[bodyIndex].dot(self.NPworldCoG[bodyIndex]) * 1e-3 - self.NPpotEnergyZeroPoint[bodyIndex]
                        totPotEnergy += potEnergy
                        if timeIndex == 0:
                            self.NPpotEnergyZeroPoint[bodyIndex] = potEnergy
                        else:
                            # Body Name vertically
                            if VerticalCounter < len(VerticalHeaders[ColumnCounter]):
                                character = VerticalHeaders[ColumnCounter][VerticalCounter]
                                if character in "0123456789":
                                    SimResultsFILE.write("'" + character + "' ")
                                else:
                                    SimResultsFILE.write(character + " ")
                            else:
                                SimResultsFILE.write("- ")
                            ColumnCounter += 1
                            SimResultsFILE.write(str(potEnergy) + " ")
                    if timeIndex == 0:
                        VerticalCounter = 0
                    else:
                        SimResultsFILE.write(str(totKinEnergy) + " ")
                        SimResultsFILE.write(str(totPotEnergy) + " ")
                        SimResultsFILE.write(str(totKinEnergy + totPotEnergy) + " ")
                        SimResultsFILE.write("\n")
                        VerticalCounter += 1
        # Next timeIndex

                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Spring"] or \
                    forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Linear Spring Damper"]:
                    # potEnergy += 0.5 * forceObj.k * delta**2
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring"] or \
                        forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring Damper"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Force Local to Body"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Global Force"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Torque about a Point"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Contact Friction"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Unilateral Spring Damper"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Motor"]:
                    pass
                    """
                """
                elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Motor with Air Friction"]:
                    pass
                    """

        SimResultsFILE.close()
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug: ST.Mess("SimMainC-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug: ST.Mess("SimMainC-__setstate__")
    #  =========================================================================
    """
    def cleanUpIndices(self, bodyName, bodyIndex):
        # Clean up Joint Indices in case the body order has been altered
        for jointNum in range(self.numJoints):
            if self.NPjointObjList[jointNum].body_I_Name == bodyName:
                self.NPjointObjList[jointNum].bodyHeadIndex = bodyIndex
            if self.NPjointObjList[jointNum].body_J_Name == bodyName:
                self.NPjointObjList[jointNum].bodyTailIndex = bodyIndex
        # Clean up force Indices in case the body order has been altered
        for forceNum in range(self.numForces):
            if self.NPforceObjList[forceNum].body_I_Name == bodyName:
                self.NPforceObjList[forceNum].bodyForceHeadIndex = bodyIndex
            if self.NPforceObjList[forceNum].body_J_Name == bodyName:
                self.NPforceObjList[forceNum].bodyForceTailIndex = bodyIndex
                """
    #  -------------------------------------------------------------------------
    """
    def clearZombieBodies(self, bodyObjDict):
        # Clean up any zombie body names
        for jointNum in range(self.numJoints):
            if self.NPjointObjList[jointNum].body_I_Name not in bodyObjDict:
                self.NPjointObjList[jointNum].body_I_Name = ""
                self.NPjointObjList[jointNum].body_I_Label = ""
                self.NPjointObjList[jointNum].bodyHeadIndex = 0
            if self.NPjointObjList[jointNum].body_J_Name not in bodyObjDict:
                self.NPjointObjList[jointNum].body_J_Name = ""
                self.NPjointObjList[jointNum].body_J_Label = ""
                self.NPjointObjList[jointNum].bodyTailIndex = 0
        for forceNum in range(self.numForces):
            if self.NPforceObjList[forceNum].body_I_Name not in bodyObjDict:
                self.NPforceObjList[forceNum].body_I_Name = ""
                self.NPforceObjList[forceNum].body_I_Label = ""
                self.NPforceObjList[forceNum].bodyForceHeadIndex = 0
            if self.NPforceObjList[forceNum].body_J_Name not in bodyObjDict:
                self.NPforceObjList[forceNum].body_J_Name = ""
                self.NPforceObjList[forceNum].body_J_Label = ""
                self.NPforceObjList[forceNum].bodyForceTailIndex = 0
                """
    #  =========================================================================
