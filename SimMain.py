import FreeCAD as CAD
import SimTools as ST
import SimFunction

import numpy as np
from scipy.integrate import solve_ivp
import math

Debug = True
# =============================================================================
# ==================================
# Matlab Code from Nikravesh: DAP_BC
# ==================================
# Body = struct ( ...
#    'm'     , 1     , ... % mass
#    'J'     , 1     , ... % moment of inertia
#    'r'	 , [0;0] , ... % x, y coordinates
#    'p'  	 , 0     , ... % angle phi
#    'r_d'   , [0;0] , ... % time derivative of x and y
#    'p_d'   , 0     , ... % time derivative of phi
#    'A'	 , eye(2), ... % rotational transformation matrix
#    'r_dd'  , [0;0] , ... % x_double_dot,y_double_do
#    'p_dd'  , 0     , ... % 2nd time derivative of phi
#    'irc'   , 0     , ... % index of the 1st element of r in u or u_dot
#    'irv'   , 0     , ... % index of the 1st element of r_dot in u or u_dot
#    'ira'   , 0     , ... % index of the 1st element of r_dot2 in v_dot
#    'm_inv' , 1     , ... % mass inverse
#    'J_inv' , 1     , ... % inverse of moment of inertia
#    'wgt'   , [0;0] , ... % weight of body as a force vector
#    'f'     , [0;0] , ... % sum of forces that act on the body
#    'n'     , 0     , ... % sum of moments that act on the body
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
#    'sPlocal', [0;0] , ... % body-fixed coordinates
#    'sP'     , [0;0] , ... % x, y components of vector s
#    'sP_r'   , [0;0] , ... % vector s rotated
#    'rP'     , [0;0] , ... % x, y coordinates of the point
#    'sP_d'   , [0;0] , ... % s_P_dot
#    'rP_d'   , [0;0] , ... % r_P_dot
#    'rP_dd'  , [0;0]   ... % r_P_dot2
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
#    'type'     , 'rev' , ... % joint type: rev, tran, rev-rev, rev-tran, rigid, disc, rel-rot, rel-tran
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
#    'p0'       , 0     , ... % initial condition phi for a disc (or rigid)
#    'd0'       , []    , ... % initial condition for d (rigid)
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

        # Dictionary of the pointers for Dynamic calling of the Acceleration functions
        self.dictAccelerationFunctions = {
            0: self.Revolute_Acc,
            1: self.Translational_Acc,
            2: self.Revolute_Revolute_Acc,
            3: self.Translational_Revolute_Acc,
            4: self.Rigid_Acc,
            5: self.Disc_Acc,
            6: self.Driven_Revolute_Acc,
            7: self.Driven_Translational_Acc,
        }
        # Dictionary of the pointers for Dynamic calling of the constraint functions
        self.dictconstraintFunctions = {
            0: self.Revolute_constraint,
            1: self.Translational_constraint,
            2: self.Revolute_Revolute_constraint,
            3: self.Translational_Revolute_constraint,
            4: self.Rigid_constraint,
            5: self.Disc_constraint,
            6: self.Driven_Revolute_constraint,
            7: self.Driven_Translational_constraint,
        }
        # Dictionary of the pointers for Dynamic calling of the Jacobian functions
        self.dictJacobianFunctions = {
            0: self.Revolute_Jacobian,
            1: self.Translational_Jacobian,
            2: self.Revolute_Revolute_Jacobian,
            3: self.Translational_Revolute_Jacobian,
            4: self.Rigid_Jacobian,
            5: self.Disc_Jacobian,
            6: self.Driven_Revolute_Jacobian,
            7: self.Driven_Translational_Jacobian,
        }

        # Save the parameters passed via the __init__ function
        self.simEnd = simEnd
        self.simDelta = simDelta
        self.correctInitial = correctInitial

        # Store the required accuracy figures
        self.relativeTolerance = 10**(-Accuracy-2)
        self.absoluteTolerance = 10**(-Accuracy-4)
        
        # Counter of function evaluations
        self.Counter = 0

        # We get the objects and set global constants
        self.solverObj = CAD.ActiveDocument.findObjects(Name="^SimSolver$")[0]

        self.bodyGroup = CAD.ActiveDocument.findObjects(Name="^LinkGroup")
        for body in self.bodyGroup:
            if Debug: ST.Mess(body.Name)
        self.numBodies = len(self.bodyGroup)
        self.numMovBodiesx3 = (self.numBodies-1) * 3

        self.jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0].Group
        self.numJoints = len(self.jointGroup)
        for joint in self.jointGroup:
            if Debug: ST.Mess(joint.Name)

        # Set a variable to flag whether we have reached the end error-free
        # It will be available to SimSolverMod as an instance variable
        self.initialised = False

        # Get the plane normal rotation quaternion from the main Sim container
        # This will rotate all the coordinates in the model, to be in the X-Y plane
        # ToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDoToDo
        # xyzToXYRotation = CAD.Rotation(CAD.Vector(0.0, 0.0, 1.0), ST.getContainerObject().movementPlaneNormal)

        # Find the maximum number of points in any of the bodies
        # We will need this so we can initialise large enough NumPy arrays
        maxNumberPoints = 0
        for body in self.bodyGroup:
            if maxNumberPoints < body.NumberPoints:
                maxNumberPoints = body.NumberPoints

        # Initialise the size of all the NumPy arrays and fill with zeros
        self.initNumPyArrays(maxNumberPoints)

        # Transfer all the 3D stuff into the NumPy arrays while doing the projection onto the X-Y plane
        bodyIndex = 0
        for bodyObj in self.bodyGroup:

            # Calculate the body CoG and MoI
            ST.updateCoG(bodyObj)

            # All Mass and moment of inertia stuff
            self.MassNp[bodyIndex] = bodyObj.masskg
            self.momentInertiaNp[bodyIndex] = bodyObj.momentOfInertia
            npVec = ST.CADVecToNumPy(ST.getContainerObject().gravityVector * bodyObj.masskg)
            self.WeightNp[bodyIndex] = npVec

            # Change the local vectors to be relative to the CoG, rather than the body origin
            # The CoG in world coordinates are the world coordinates of the body
            # All points in the body are relative to this point

            # World
            CoG = bodyObj.worldCoG
            npCoG = ST.CADVecToNumPy(CoG)
            self.worldNp[bodyIndex, 0:2] = npCoG
            self.worldRotNp[bodyIndex, 0:2] = ST.Rot90NumPy(npCoG.copy())

            # WorldDot
            npWorldDot = ST.CADVecToNumPy(bodyObj.worldDot)
            self.worldDotNp[bodyIndex, 0:2] = npWorldDot
            self.worldDotRotNp[bodyIndex, 0:2] = ST.Rot90NumPy(npWorldDot.copy())

            # WorldDotDot
            self.worldDotDotNp[bodyIndex, 0:2] = np.zeros((1, 2))

            # Transform the joint points from world Placement to World X-Y plane relative to the CoG
            for index in range(bodyObj.NumberPoints):
                bodyObj.PointLocals[index] = bodyObj.PointWorlds[index] - CoG

            # Phi defined by the longest vector from CoG to Joint
            # The first body is ALWAYS the ground body
            if bodyIndex == 0:
                self.phiNp[bodyIndex] = 0.0
            else:
                maxNorm = 0.0
                largest = 0
                for jointIndex in range(bodyObj.NumberPoints):
                    if maxNorm < bodyObj.PointLocals[jointIndex].Length:
                        maxNorm = bodyObj.PointLocals[jointIndex].Length
                        largest = jointIndex
                self.phiNp[bodyIndex] = CAD.Vector(1.0,0.0,0.0).getAngle(bodyObj.PointWorlds[largest]-CoG)

            # The phiDot axis vector is by definition perpendicular to the movement plane,
            # so we don't have to do any rotating from the phiDot value set in bodyObj
            self.phiDotNp[bodyIndex] = bodyObj.phiDot

            # We will now calculate the rotation matrix and use it to find the coordinates of the points
            self.RotMatPhiNp[bodyIndex] = ST.RotationMatrixNp(self.phiNp[bodyIndex])

            for pointIndex in range(bodyObj.NumberPoints):
                # Point Local - vector from module body CoG to the point, in body LCS coordinates
                # [This is what we needed phi for, to fix the orientation of the body]
                npVec = ST.CADVecToNumPy(bodyObj.PointWorlds[pointIndex])
                self.pointXiEtaNp[bodyIndex, pointIndex, 0:2] = npVec @ self.RotMatPhiNp[bodyIndex]
                # Point Vector - vector from body CoG to the point in world coordinates
                self.pointXYrelCoGNp[bodyIndex, pointIndex, 0:2] = npVec
                self.pointXYrelCoGrotNp[bodyIndex][pointIndex] = ST.Rot90NumPy(npVec.copy())
                # Point Vector Dot
                self.pointXYrelCoGdotNp[bodyIndex][pointIndex] = np.zeros((1, 2))
                # Point World - coordinates of the point relative to the system origin - in world coordinates
                self.pointXYWorldNp[bodyIndex, pointIndex] = npVec
                self.pointWorldRotNp[bodyIndex, pointIndex] = ST.Rot90NumPy(npVec.copy())
                # Point World Dot
                self.pointWorldDotNp[bodyIndex][pointIndex] = np.zeros((1, 2))
            # Next pointIndex
            bodyIndex += 1
        # Next bodyObject

        if 1==1: #Debug:
            for body in self.bodyGroup:
                ST.Mess(body.Label+" : "+body.Name)
            ST.Mess("Mass: [g]")
            ST.PrintNp1D(True, self.MassNp * 1.0e3)
            ST.Mess("")
            ST.Mess("Mass: [kg]")
            ST.PrintNp1D(True, self.MassNp)
            ST.Mess("")
            ST.Mess("Weight Vector: [kg mm /s^2 = mN]")
            ST.PrintNp2D(self.WeightNp)
            ST.Mess("")
            ST.Mess("MomentInertia: [kg.mm^2]")
            ST.Mess(self.momentInertiaNp)
            ST.Mess("")
            ST.Mess("World [CoG]: [mm]")
            ST.PrintNp2D(self.worldNp)
            ST.Mess("")
            ST.Mess("WorldDot: [mm/s]")
            ST.PrintNp2D(self.worldDotNp)
            ST.Mess("")
            ST.MessNoLF("phi: [deg]")
            ST.PrintNp1Ddeg(True, self.phiNp)
            ST.Mess("")
            ST.MessNoLF("phi: [rad]")
            ST.PrintNp1D(True, self.phiNp)
            ST.Mess("")
            ST.MessNoLF("phiDot: [deg/sec]")
            ST.PrintNp1Ddeg(True, self.phiDotNp)
            ST.Mess("")
            ST.MessNoLF("phiDot: [rad/sec]")
            ST.PrintNp1D(True, self.phiDotNp)
            ST.Mess("")
            ST.MessNoLF("Number of Points: ")
            ST.Mess("PointLocal: [mm]")
            ST.PrintNp3D(self.pointXiEtaNp)
            ST.Mess("")
            ST.Mess("PointVector: [mm]")
            ST.PrintNp3D(self.pointXYrelCoGNp)
            ST.Mess("")
            ST.Mess("PointWorld: [mm]")
            ST.PrintNp3D(self.pointXYWorldNp)
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
        self.massArrayNp = np.zeros(self.numMovBodiesx3)
        for index in range(1, self.numBodies):
            bodyObj = self.bodyObjList[index]
            self.massArrayNp[(index-1)*3:index*3] = bodyObj.masskg, bodyObj.masskg, bodyObj.momentOfInertia

        # Transfer the joint unit vector coordinates to the NumPy arrays
        for jointIndex in range(self.numJoints):
            jointObj = self.jointObjList[jointIndex]
            # Unit vector on body I in body local coordinates
            self.jointUnit_I_XiEtaNp[jointIndex] = ST.NormalizeNpVec(self.pointXiEtaNp[jointObj.body_I_Index, jointObj.point_I_j_Index] -
                                                                     self.pointXiEtaNp[jointObj.body_I_Index, jointObj.point_I_i_Index])
            # Unit vector on body I in world coordinates
            self.jointUnit_I_WorldNp[jointIndex] = ST.NormalizeNpVec(self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_j_Index] -
                                                                     self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index])
            self.jointUnit_I_WorldRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_I_WorldNp[jointIndex].copy())
            self.jointUnit_I_WorldDotNp[jointIndex] = ST.NormalizeNpVec(self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_j_Index] -
                                                                        self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])
            self.jointUnit_I_WorldDotRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_I_WorldDotNp[jointIndex].copy())

            # Unit vector on body J in body local coordinates
            self.jointUnit_J_XiEtaNp[jointIndex] = ST.NormalizeNpVec(self.pointXiEtaNp[jointObj.body_J_Index, jointObj.point_J_j_Index] -
                                                                     self.pointXiEtaNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
            # Unit vector on body J in world coordinates
            self.jointUnit_J_WorldNp[jointIndex] = ST.NormalizeNpVec(self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_j_Index] -
                                                                     self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
            self.jointUnit_J_WorldRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_J_WorldNp[jointIndex].copy())
            self.jointUnit_J_WorldDotNp[jointIndex] = ST.NormalizeNpVec(self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_j_Index] -
                                                                        self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
            self.jointUnit_J_WorldDotRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_J_WorldDotNp[jointIndex].copy())

            # Find the length of the link between the first point on each body - signed scalar
            unitPinInSlot = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                            self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
            length = np.sqrt(unitPinInSlot[0]**2 + unitPinInSlot[1]**2)
            dotProduct = self.jointUnit_I_WorldNp[jointIndex].dot(unitPinInSlot)
            if dotProduct < 0.0:
                jointObj.lengthLink = -length
            else:
                jointObj.lengthLink = length

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
        for jointIndex in range(self.numJoints):
            jointObj = self.jointObjList[jointIndex]
            # If the body is attached to ground then its unit vector local coordinates are world coordinates
            if jointObj.body_I_Index == 0:
                self.jointUnit_I_WorldNp[jointIndex] = self.jointUnit_I_XiEtaNp[jointIndex]
                self.jointUnit_I_WorldRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_I_WorldNp[jointIndex].copy())
            if jointObj.body_J_Index == 0:
                self.jointUnit_J_WorldNp[jointIndex] = self.jointUnit_J_XiEtaNp[jointIndex]
                self.jointUnit_J_WorldRotNp[jointIndex] = ST.Rot90NumPy(self.jointUnit_J_WorldNp[jointIndex].copy())

        # Assign number of constraint and number of bodies to each defined joint according to its type
        for jointObj in self.jointObjList:
            if jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Revolute"]:
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
                    jointObj.nMovBodies = 2
                    if jointObj.fixDof is True:
                        jointObj.mConstraints = 3
                        # Set the initial angle phi0 and handle the case where one is attached to ground
                        if jointObj.body_I_Index == 0:
                            jointObj.phi0 = -self.phiNp[jointObj.body_J_Index]
                        elif jointObj.body_J_Index == 0:
                            jointObj.phi0 = self.phiNp[jointObj.body_I_Index]
                        else:
                            jointObj.phi0 = self.phiNp[jointObj.body_I_Index] - self.phiNp[jointObj.body_J_Index]
                else:
                    # ==================================
                    # Matlab Code from Nikravesh: DAP_BC
                    # ==================================
                    #        case {'rel-rot'}                                       % revised August 2022
                    #            Joints(Ji).mrows = 1; Joints(Ji).nbody = 1;        % revised August 2022
                    # ==================================
                    jointObj.mConstraints = 1
                    jointObj.nMovBodies = 1

            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Translation"]:
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
                jointObj.nMovBodies = 2
                if jointObj.fixDof is True:
                    jointObj.mConstraints = 3
                    if jointObj.body_I_Index == 0:
                        vec = (+ self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index]
                               - self.worldNp[jointObj.body_J_Index]
                               - self.RotMatPhiNp[jointObj.body_J_Index] @ self.pointXiEtaNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
                    elif jointObj.body_J_Index == 0:
                        vec = (- self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
                               + self.worldNp[jointObj.body_I_Index]
                               + self.RotMatPhiNp[jointObj.body_I_Index] @ self.pointXiEtaNp[jointObj.body_I_Index, point_i_Index])
                    else:
                        vec = (+ self.worldNp[jointObj.body_I_Index]
                               + self.RotMatPhiNp[jointObj.body_I_Index] @ self.pointXiEtaNp[jointObj.body_I_Index, jointObj.point_I_i_Index]
                               - self.worldNp[jointObj.body_J_Index]
                               - self.RotMatPhiNp[jointObj.body_J_Index] @ self.pointXiEtaNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
                    jointObj.phi0 = np.sqrt(vec.dot(vec))
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Revolute-Revolute"]:
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
                jointObj.nMovBodies = 2
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Translation-Revolute"]:
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
                jointObj.nMovBodies = 2
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Driven-Translation"]:
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
                jointObj.nMovBodies = 1
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Rigid"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'rigid'}
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
                jointObj.nMovBodies = 2
                if jointObj.body_I_Index == 0:
                    A = -self.RotMatPhiNp[jointObj.body_J_Index].T @ self.worldNp[jointObj.body_J_Index]
                    jointObj.phi0 = -self.phiNp[jointObj.body_J_Index]
                elif jointObj.body_J_Index == 0:
                    A = self.worldNp[jointObj.body_I_Index]
                    jointObj.phi0 = self.phiNp[jointObj.body_I_Index]
                else:
                    A = self.RotMatPhiNp[jointObj.body_J_Index].T @ (self.worldNp[jointObj.body_I_Index] -
                                                                     self.worldNp[jointObj.body_J_Index])
                    jointObj.phi0 = self.phiNp[jointObj.body_I_Index] - \
                                    self.phiNp[jointObj.body_J_Index]
                jointObj.d0.x = A[0]
                jointObj.d0.y = A[1]
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY["Disc"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'disc'}
                #            Joints(Ji).mrows = 2;
                #            Joints(Ji).nbody = 1;
                # ==================================
                jointObj.mConstraints = 2
                jointObj.nMovBodies = 1
                radiusVector = self.pointXiEtaNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                               self.pointXiEtaNp[jointObj.body_I_Index, jointObj.point_J_i_Index]
                jointObj.Radius = np.sqrt(radiusVector.dot(radiusVector))
                jointObj.phi0 = np.arctan2(radiusVector[1], radiusVector[0])
            else:
                CAD.Console.PrintError("Unknown Joint Type - this should never occur"+str(jointObj.JointType)+"\n")
        # Next Joint Object

        # Run through the joints and find if any of them use a driver function
        # if so, then initialize the parameters for the driver function routine
        self.driverObjDict = {}
        for jointObj in self.jointObjList:
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

        # Add up all the numbers of constraints and allocate row start and end pointers
        self.numConstraints = 0
        for jointObj in self.jointObjList:
            jointObj.rowStart = self.numConstraints
            jointObj.rowEnd = self.numConstraints + jointObj.mConstraints
            self.numConstraints = jointObj.rowEnd

        # Return with a flag to show we have reached the end of init error-free
        self.initialised = True
    #  -------------------------------------------------------------------------
    def MainSolve(self):
        if self.numConstraints != 0 and self.correctInitial:
            # Correct for initial conditions consistency
            if self.correctInitialConditions() is False:
                CAD.Console.PrintError("Initial Conditions not successfully calculated")
                return

        # Determine any redundancy between constraints
        Jacobian = self.getJacobian()
        if Debug:
            ST.Mess("Jacobian calculated to determine rank of solution")
            ST.PrintNp2D(Jacobian)
        redundant = np.linalg.matrix_rank(Jacobian)
        if redundant < self.numConstraints:
            CAD.Console.PrintError('The constraints exhibit Redundancy\n')
            return

        # Velocity correction
        velCorrArrayNp = np.zeros((self.numMovBodiesx3,), dtype=np.float64)
        # Move velocities to the corrections array
        for bodyIndex in range(1, self.numBodies):
            velCorrArrayNp[(bodyIndex-1) * 3: bodyIndex*3] = \
                self.worldDotNp[bodyIndex, 0], \
                self.worldDotNp[bodyIndex, 1], \
                self.phiDotNp[bodyIndex]
        # Solve for velocity at time = 0
        # Unless the joint is Driven-Revolute or Driven-Translational
        # RHSVel = [0,0,...]   (i.e. a list of zeros)
        solution = np.linalg.solve(Jacobian @ Jacobian.T, (Jacobian @ velCorrArrayNp) - self.RHSVel(0))
        deltaVel = -Jacobian.T @ solution
        if Debug:
            ST.MessNoLF("Velocity Correction Array: ")
            ST.PrintNp1D(True, velCorrArrayNp)
            ST.MessNoLF("Velocity Correction Solution: ")
            ST.PrintNp1D(True, solution)
            ST.MessNoLF("Delta velocity: ")
            ST.PrintNp1D(True, deltaVel)
        # Move corrected velocities back into the system
        for bodyIndex in range(1, self.numBodies):
            self.worldDotNp[bodyIndex, 0] += deltaVel[(bodyIndex-1)*3]
            self.worldDotNp[bodyIndex, 1] += deltaVel[(bodyIndex-1)*3+1]
            self.phiDotNp[bodyIndex] += deltaVel[(bodyIndex-1)*3+2]
        # Report corrected coordinates and velocities
        if Debug:
            ST.Mess("Corrected Positions: [mm]")
            ST.PrintNp2D(self.worldNp)
            ST.PrintNp1Ddeg(True, self.phiNp)
            ST.Mess("Corrected Velocities [mm/s]")
            ST.PrintNp2D(self.worldDotNp)
            ST.PrintNp1Ddeg(True, self.phiDotNp)

        ##############################
        # START OF THE SOLUTION PROPER
        ##############################
        # Pack coordinates and velocities into the NumPy uArray
        uArray = np.zeros((self.numMovBodiesx3 * 2,), dtype=np.float64)
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            uArray[index1:index1+2] = self.worldNp[bodyIndex]
            uArray[index1+2] = self.phiNp[bodyIndex]
            uArray[index2:index2+2] = self.worldDotNp[bodyIndex]
            uArray[index2+2] = self.phiDotNp[bodyIndex]
            index1 += 3
            index2 += 3
        if Debug:
            ST.Mess("uArray:")
            ST.PrintNp1D(True, uArray)
        # Set up the list of time intervals over which to integrate
        self.Tspan = np.arange(0.0, self.simEnd, self.simDelta)

        # ###################################################################################
        # Matrix Integration Function
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_iSV.html
        # ###################################################################################
        # scipy.integrate.solve_ivp
        # INPUTS:
        #       fun,                      Function name
        #       t_span,                   (startTime, endTime)
        #       y0,                       Initial values array [uArray]
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

        # Solve the equations: <analysis function> (<start time>, <end time>) <pos & vel array> <times at which to evaluate>
        solution = solve_ivp(self.Analysis,
                             (0.0, self.simEnd),
                             uArray,
                             t_eval=self.Tspan,
                             rtol=self.relativeTolerance,
                             atol=self.absoluteTolerance)

        # Output the positions/angles results file
        self.PosFILE = open(os.path.join(self.solverObj.Directory, "SimAnimation.csv"), 'w')
        Sol = solution.y.T
        for tick in range(len(solution.t)):
            self.PosFILE.write(str(solution.t[tick])+" ")
            for body in range(self.numBodies-1):
                self.PosFILE.write(str(Sol[tick, body * 3]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 3 + 1]) + " ")
                self.PosFILE.write(str(Sol[tick, body * 3 + 2]) + " ")
            self.PosFILE.write("\n")
        self.PosFILE.close()

        # Save the most important stuff into the solver object
        BodyNames = []
        BodyCoG = []
        for bodyIndex in range(1, len(self.bodyObjList)):
            BodyNames.append(self.bodyObjList[bodyIndex].Name)
            BodyCoG.append(self.bodyObjList[bodyIndex].centreOfGravity)
        self.solverObj.BodyNames = BodyNames
        self.solverObj.BodyCoG = BodyCoG
        self.solverObj.DeltaTime = self.simDelta
        # Flag that the results are valid
        self.solverObj.SimResultsValid = True

        if self.solverObj.FileName != "-":
            self.outputResults(solution.t, solution.y.T)
    ##########################################
    #   This is the end of the actual solution
    #    The rest are all called subroutines
    ##########################################
    #  -------------------------------------------------------------------------
    def Analysis(self, tick, uArray):
        """The Analysis function which takes a
            uArray consisting of a world 3vector and a velocity 3vector"""
        if Debug:
            ST.Mess("Input to 'Analysis'")
            ST.PrintNp1D(True, uArray)

        # Unpack uArray into world coordinate and world velocity sub-arrays
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            self.worldNp[bodyIndex, 0] = uArray[index1]
            self.worldNp[bodyIndex, 1] = uArray[index1+1]
            self.phiNp[bodyIndex] = uArray[index1+2]
            self.worldDotNp[bodyIndex, 0] = uArray[index2]
            self.worldDotNp[bodyIndex, 1] = uArray[index2+1]
            self.phiDotNp[bodyIndex] = uArray[index2+2]
            index1 += 3
            index2 += 3
        if Debug:
            ST.PrintNp2D(self.worldNp)
            ST.PrintNp1Ddeg(True, self.phiNp)
            ST.PrintNp2D(self.worldDotNp)
            ST.PrintNp1Ddeg(True, self.phiDotNp)

        # Update the point stuff accordingly
        self.updatePointPositions()
        self.updatePointVelocities()

        # array of applied forces
        self.makeForceArray()
        # find the accelerations ( a = F / m )
        accel = []
        if self.numConstraints == 0:
            for index in range(self.numMovBodiesx3):
                accel.append = self.massInvArray[index] * self.forceArrayNp[index]
        # We go through this if we have any constraints
        else:
            Jacobian = self.getJacobian()
            if Debug:
                ST.Mess("Jacobian")
                ST.PrintNp2D(Jacobian)

            # Create the Jacobian-Mass-Jacobian matrix
            # [ diagonal masses ---- Jacobian transpose ]
            # [    |                        |           ]
            # [  Jacobian      ------     Zeros         ]
            numBodPlusConstr = self.numMovBodiesx3 + self.numConstraints
            JacMasJac = np.zeros((numBodPlusConstr, numBodPlusConstr), dtype=np.float64)
            JacMasJac[0: self.numMovBodiesx3, 0: self.numMovBodiesx3] = np.diag(self.massArrayNp)
            JacMasJac[self.numMovBodiesx3:, 0: self.numMovBodiesx3] = Jacobian
            JacMasJac[0: self.numMovBodiesx3, self.numMovBodiesx3:] = -Jacobian.T
            if Debug:
                ST.Mess("Jacobian-MassDiagonal-JacobianT Array")
                ST.PrintNp2D(JacMasJac)

            # get r-h-s of acceleration constraints at this time
            rhsAccel = self.RHSAcc(tick)
            if Debug:
                ST.Mess("rhsAccel")
                ST.PrintNp1D(True, rhsAccel)
            # Combine Force Array and rhs of Acceleration constraints into one array
            rhs = np.zeros((numBodPlusConstr,), dtype=np.float64)
            rhs[0: self.numMovBodiesx3] = self.forceArrayNp
            rhs[self.numMovBodiesx3:] = rhsAccel
            if Debug:
                ST.Mess("rhs")
                ST.PrintNp1D(True, rhs)
            # Solve the JacMasJac augmented with the rhs
            solvedVector = np.linalg.solve(JacMasJac, rhs)
            # First half of solution are the acceleration values
            accel = solvedVector[: self.numMovBodiesx3]
            # Second half is Lambda which is reported in the output results routine
            self.Lambda = solvedVector[self.numMovBodiesx3:]
            if Debug:
                ST.MessNoLF("Accelerations: ")
                ST.PrintNp1D(True, accel)
                ST.MessNoLF("Lambda: ")
                ST.PrintNp1D(True, self.Lambda)

        # Transfer the accelerations back into the worldDotDot/phiDotDot and uDot/uDotDot Arrays
        for bodyIndex in range(1, self.numBodies):
            accelIndex = (bodyIndex-1)*3
            self.worldDotDotNp[bodyIndex] = accel[accelIndex], accel[accelIndex+1]
            self.phiDotDotNp[bodyIndex] = accel[accelIndex+2]
        uDotArray = np.zeros((self.numMovBodiesx3 * 2), dtype=np.float64)
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            uDotArray[index1:index1+2] = self.worldDotNp[bodyIndex]
            uDotArray[index1+2] = self.phiDotNp[bodyIndex]
            uDotArray[index2:index2+2] = self.worldDotDotNp[bodyIndex]
            uDotArray[index2+2] = self.phiDotDotNp[bodyIndex]
            index1 += 3
            index2 += 3

        # Increment number of function evaluations
        self.Counter += 1

        return uDotArray
        #  -------------------------------------------------------------------------
    def correctInitialConditions(self):
        """This function corrects the supplied initial conditions by making
        the body coordinates and velocities consistent with the constraints"""
        if Debug:
            ST.Mess("SimMain-correctInitialConditions")
        # Try Newton-Raphson iteration for n up to 20
        for n in range(20):
            # Update the points positions
            self.updatePointPositions()

            # Evaluate Deltaconstraint of the constraints at time=0
            Deltaconstraints = self.GetconstraintsF(0)
            if Debug:
                ST.Mess("Delta constraints Result:")
                ST.PrintNp1D(True, Deltaconstraints)

            # Evaluate Jacobian
            Jacobian = self.getJacobian()
            if Debug:
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
                self.worldNp[bodyIndex, 0] += delta[(bodyIndex-1)*3]
                self.worldNp[bodyIndex, 1] += delta[(bodyIndex-1)*3+1]
                self.phiNp[bodyIndex] += delta[(bodyIndex-1)*3+2]

        CAD.Console.PrintError("Newton-Raphson Correction failed to converge\n\n")
        return False
    #  -------------------------------------------------------------------------
    def updatePointPositions(self):
        for bodyIndex in range(1, self.numBodies):
            # Compute the Rotation Matrix
            self.RotMatPhiNp[bodyIndex] = ST.RotationMatrixNp(self.phiNp[bodyIndex])

            if Debug:
                ST.MessNoLF("In Xi-Eta Coordinates           ")
                ST.MessNoLF("Relative to CoG                 ")
                ST.MessNoLF("Relative to CoG Rotated 90      ")
                ST.Mess("World Coordinates               ")
            for pointIndex in range(len(self.pointDictList[bodyIndex])):
                pointVector = self.RotMatPhiNp[bodyIndex] @ self.pointXiEtaNp[bodyIndex, pointIndex]
                self.pointXYrelCoGNp[bodyIndex, pointIndex] = pointVector
                self.pointXYWorldNp[bodyIndex, pointIndex] = self.worldNp[bodyIndex] + pointVector
                self.pointXYrelCoGrotNp[bodyIndex][pointIndex] = ST.Rot90NumPy(pointVector)
                if Debug:
                    ST.PrintNp1D(False, self.pointXiEtaNp[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.pointXYrelCoGNp[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.pointXYrelCoGrotNp[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(True, self.pointXYWorldNp[bodyIndex][pointIndex])
    #  -------------------------------------------------------------------------
    def updatePointVelocities(self):
        if Debug:
            ST.Mess("SimMain-updatePointVelocities")
        for bodyIndex in range(1, self.numBodies):
            for pointIndex in range(len(self.pointDictList[bodyIndex])):
                velVector = self.pointXYrelCoGrotNp[bodyIndex, pointIndex] * self.phiDotNp[bodyIndex]
                self.pointXYrelCoGdotNp[bodyIndex, pointIndex] = velVector
                self.pointWorldDotNp[bodyIndex, pointIndex] = self.worldDotNp[bodyIndex] + velVector
        # for forceObj in self.forceObjList:
        #   if forceObj.actuatorType != 0:
        #        if forceObj.body_I_Index != 0:
        #            forceObj.FUnit_I_WorldDot = ST.Rot90NumPy(forceObj.FUnit_I_World) * self.phiDotNp[forceObj.body_I_Index]
    #  -------------------------------------------------------------------------
    def cleanUpIndices(self, bodyName, bodyIndex):
        # Clean up Joint Indices in case the body order has been altered
        for jointNum in range(self.numJoints):
            if self.jointObjList[jointNum].body_I_Name == bodyName:
                self.jointObjList[jointNum].body_I_Index = bodyIndex
            if self.jointObjList[jointNum].body_J_Name == bodyName:
                self.jointObjList[jointNum].body_J_Index = bodyIndex
        # Clean up force Indices in case the body order has been altered
        for forceNum in range(self.numForces):
            if self.forceObjList[forceNum].body_I_Name == bodyName:
                self.forceObjList[forceNum].body_I_Index = bodyIndex
            if self.forceObjList[forceNum].body_J_Name == bodyName:
                self.forceObjList[forceNum].body_J_Index = bodyIndex
    #  -------------------------------------------------------------------------
#    def clearZombieBodies(self, bodyObjDict):
#        # Clean up any zombie body names
#        for jointNum in range(self.numJoints):
#            if self.jointObjList[jointNum].body_I_Name not in bodyObjDict:
#                self.jointObjList[jointNum].body_I_Name = ""
#                self.jointObjList[jointNum].body_I_Label = ""
#                self.jointObjList[jointNum].body_I_Index = 0
#            if self.jointObjList[jointNum].body_J_Name not in bodyObjDict:
#                self.jointObjList[jointNum].body_J_Name = ""
#                self.jointObjList[jointNum].body_J_Label = ""
#                self.jointObjList[jointNum].body_J_Index = 0
#        for forceNum in range(self.numForces):
#            if self.forceObjList[forceNum].body_I_Name not in bodyObjDict:
#                self.forceObjList[forceNum].body_I_Name = ""
#                self.forceObjList[forceNum].body_I_Label = ""
#                self.forceObjList[forceNum].body_I_Index = 0
#            if self.forceObjList[forceNum].body_J_Name not in bodyObjDict:
#                self.forceObjList[forceNum].body_J_Name = ""
#                self.forceObjList[forceNum].body_J_Label = ""
#                self.forceObjList[forceNum].body_J_Index = 0
    #  =========================================================================
    def GetconstraintsF(self, tick):
        """Returns a numConstraints-long vector which contains the current deviation
        from the defined constraints"""
        if Debug:
            ST.Mess("SimMain-constraints")

        DeltaconstraintNp = np.zeros((self.numConstraints,), dtype=np.float64)

        # Call the applicable function which is pointed to by the constraint function dictionary
        for jointObj in self.jointObjList:
            if jointObj.JointType == ST.JOINT_TYPE_DICTIONARY['Revolute'] and jointObj.FunctType != -1:
                jointObj.JointType = ST.JOINT_TYPE_DICTIONARY['Driven-Revolute']
            constraintNp = self.dictconstraintFunctions[jointObj.JointType](jointObj, tick)
            DeltaconstraintNp[jointObj.rowStart: jointObj.rowEnd] = constraintNp


        return DeltaconstraintNp
    #  =========================================================================
    def getJacobian(self):
        """Returns the Jacobian matrix numConstraints X (3 x numMovBodies)"""
        if Debug:
            ST.Mess("SimMain-Jacobian")
        Jacobian = np.zeros((self.numConstraints, self.numMovBodiesx3,))
        for jointObj in self.jointObjList:
            # Call the applicable function which is pointed to by the Jacobian dictionary
            if jointObj.JointType == ST.JOINT_TYPE_DICTIONARY['Revolute'] and jointObj.FunctType != -1:
                jointObj.JointType = ST.JOINT_TYPE_DICTIONARY['Driven-Revolute']
            JacobianHead, JacobianTail = self.dictJacobianFunctions[jointObj.JointType](jointObj)
            # Fill in the values in the Jacobian
            if jointObj.body_I_Index != 0:
                columnHeadStart = (jointObj.body_I_Index-1) * 3
                columnHeadEnd = jointObj.body_I_Index * 3
                Jacobian[jointObj.rowStart: jointObj.rowEnd, columnHeadStart: columnHeadEnd] = JacobianHead
            if jointObj.body_J_Index != 0:
                columnTailStart = (jointObj.body_J_Index-1) * 3
                columnTailEnd = jointObj.body_J_Index * 3
                Jacobian[jointObj.rowStart: jointObj.rowEnd, columnTailStart: columnTailEnd] = JacobianTail
        return Jacobian
    #  =========================================================================
    def RHSAcc(self, tick):
        """Returns a numConstraints-long vector containing gamma"""
        if Debug:
            ST.Mess("SimMain-RHSAcc")
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
        #        case {'rigid'}
        #            A_rigid
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
        for jointObj in self.jointObjList:
            if jointObj.JointType == ST.JOINT_TYPE_DICTIONARY['Revolute'] and jointObj.FunctType != -1:
                jointObj.JointType = ST.JOINT_TYPE_DICTIONARY['Driven-Revolute']
            gamma = self.dictAccelerationFunctions[jointObj.JointType](jointObj, tick)
            rhsAcc[jointObj.rowStart: jointObj.rowEnd] = gamma
        return rhsAcc
    #  -------------------------------------------------------------------------
    def RHSVel(self, tick):
        if Debug:
            ST.Mess("SimMain-RHSVel")
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
        for jointObj in self.jointObjList:
            if jointObj.JointType == ST.JOINT_TYPE_DICTIONARY['Revolute'] and jointObj.FunctType != -1:
                jointObj.JointType = ST.JOINT_TYPE_DICTIONARY['Driven-Rotation']
                [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
                rhsVelNp[jointObj.rowStart: jointObj.rowEnd] = func * funcDot
            elif jointObj.JointType == ST.JOINT_TYPE_DICTIONARY['Driven-Translation']:
                [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
                rhsVelNp[jointObj.rowStart: jointObj.rowEnd] = funcDot
        return rhsVelNp
    #  =========================================================================
    def Revolute_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Revolute joint"""
        if Debug:
            ST.Mess("SimMain-Revolute_constraint")
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
        constraintNp = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                       self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        if Debug:
            ST.Mess('Revolute Constraint:')
            ST.MessNoLF('    Point I: ')
            ST.PrintNp1D(True, self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index])
            ST.MessNoLF('    Point J: ')
            ST.PrintNp1D(True, self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
            ST.MessNoLF('    Difference Vector: ')
            ST.PrintNp1D(True, constraintNp)
        if jointObj.fixDof:
            if jointObj.body_I_Index == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (-self.phiNp[jointObj.body_J_Index] - jointObj.phi0)])
            elif jointObj.body_J_Index == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.phiNp[jointObj.body_I_Index] - jointObj.phi0)])
            else:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.phiNp[jointObj.body_I_Index]
                                          - self.phiNp[jointObj.body_J_Index]
                                          - jointObj.phi0)])
        return constraintNp
    #  -------------------------------------------------------------------------
    def Revolute_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Revolute joint"""
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
            JacobianHead = np.array([
                [1.0, 0.0, self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index, 0]],
                [0.0, 1.0, self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index, 1]]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index, 0]],
                [0.0, -1.0, -self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index, 1]]])
        else:
            JacobianHead = np.array([
                [1.0, 0.0, self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index, 0]],
                [0.0, 1.0, self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index, 1]],
                [0.0, 0.0, 1.0]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index, 0]],
                [0.0, -1.0, -self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index, 1]],
                [0.0, 0.0, -1.0]])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Revolute_Acc(self, jointObj, tick):
        """Evaluate gamma for a Revolute joint"""
        if Debug:
            ST.Mess("SimMain-Revolute_Acc")
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
        if jointObj.body_I_Index == 0:
            gammaNp = ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]) * self.phiDotNp[jointObj.body_J_Index]
        elif jointObj.body_J_Index == 0:
            gammaNp = -ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index]) * self.phiDotNp[jointObj.body_I_Index]
        else:
            gammaNp = -ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index]) * self.phiDotNp[jointObj.body_I_Index] +\
                       ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]) * self.phiDotNp[jointObj.body_J_Index]
        if jointObj.fixDof:
            gammaNp = np.array([gammaNp[0], gammaNp[1], 0.0])

        return gammaNp
    #  =========================================================================
    def Revolute_Revolute_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Revolute-Revolute joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        Length = jointObj.lengthLink
        jointUnitVec = diff / Length
        return np.array([(jointUnitVec.dot(diff) - Length) / 2.0])
    #  -------------------------------------------------------------------------
    def Revolute_Revolute_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Revolute-Revolute joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        jointUnitVec = diff / jointObj.lengthLink

        JacobianHead = np.array([jointUnitVec[0], jointUnitVec[1],
                                 jointUnitVec.dot(self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])])
        JacobianTail = np.array([-jointUnitVec[0], -jointUnitVec[1],
                                 -jointUnitVec.dot(self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Revolute_Revolute_Acc(self, jointObj, tick):
        """Evaluate gamma for a Revolute-Revolute joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        diffDot = self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                  self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        jointUnitVec = diff/jointObj.lengthLink
        jointUnitVecDot = diffDot/jointObj.lengthLink
        f = -jointUnitVecDot.dot(diffDot)
        if jointObj.body_I_Index == 0:
            f += jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]) *
                               self.phiDotNp[jointObj.body_J_Index])
        elif jointObj.body_J_Index == 0:
            f -= jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index]) *
                               self.phiDotNp[jointObj.body_I_Index])
        else:
            f -= jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] *
                                             self.phiDotNp[jointObj.body_I_Index] +
                                             self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]) *
                                             self.phiDotNp[jointObj.body_J_Index])
        return f
    #  =========================================================================
    def Rigid_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Rigid joint"""
        if Debug:
            ST.Mess("SimMain-Rigid_constraint")
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
        if jointObj.body_I_Index == 0:
            return np.array([-(self.worldNp[jointObj.body_J_Index] +
                               self.RotMatPhiNp[jointObj.body_J_Index] @ ST.CADVecToNumPy(jointObj.d0)),
                             -self.phiNp[jointObj.body_J_Index] - jointObj.phi0])
        elif jointObj.body_J_Index == 0:
            return np.array([self.worldNp[jointObj.body_I_Index] - ST.CADVecToNumPy(jointObj.d0),
                             self.phiNp[jointObj.body_I_Index] - jointObj.phi0])
        else:
            return np.array([self.worldNp[jointObj.body_I_Index] -
                             (self.worldNp[jointObj.body_J_Index] +
                              self.RotMatPhiNp[body_J_Index] @ ST.CADVecToNumPy(jointObj.d0)),
                             self.phiNp[jointObj.body_I_Index] -
                             self.phiNp[jointObj.body_J_Index] -
                             jointObj.phi0])
    #  -------------------------------------------------------------------------
    def Rigid_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Rigid joint"""
        if Debug:
            ST.Mess("SimMain-Rigid_Jacobian")
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
        if jointObj.body_J_Index != 0:
            tailVector = ST.Rot90NumPy(self.RotMatPhiNp[jointObj.body_J_Index] @ ST.CADVecToNumPy(jointObj.d0))
            JacobianTail = np.array([[-1.0, 0.0, -tailVector[0]],
                                     [0.0, -1.0, -tailVector[1]],
                                     [0.0, 0.0, -1.0]])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Rigid_Acc(self, jointObj, tick):
        """Evaluate gamma for a Rigid joint"""
        if Debug:
            ST.Mess("SimMain-Rigid_Acc")
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
        if jointObj.body_J_Index != 0:
            tailVector = -self.RotMatPhiNp[jointObj.body_J_Index] @ (ST.CADVecToNumPy(jointObj.d0) *
                                                                     (self.phiDotNp[jointObj.body_J_Index]**2))
            return np.array([tailVector[0], tailVector[1], 0.0])
        else:
            return np.array([0.0, 0.0, 0.0])
    #  =========================================================================
    def Translational_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Translational joint"""
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
        jointUnitJRot = self.jointUnit_J_WorldRotNp[jointObj.JointNumber]
        jointUnitIVec = self.jointUnit_I_WorldNp[jointObj.JointNumber]
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        if Debug:
            ST.Mess('Translational Constraint:')
            ST.MessNoLF('    Unit I Vector: ')
            ST.PrintNp1D(True, jointUnitIVec)
            ST.MessNoLF('    Unit J Vector Rotated: ')
            ST.PrintNp1D(True, jointUnitJRot)
            ST.MessNoLF('    World I: ')
            ST.PrintNp1D(True, self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index])
            ST.MessNoLF('    World J: ')
            ST.PrintNp1D(True, self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index])
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
    #  -------------------------------------------------------------------------
    def Translational_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Translational joint"""
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
        jointUnitJVec = self.jointUnit_J_WorldNp[jointObj.JointNumber]
        jointUnitJRot = self.jointUnit_J_WorldRotNp[jointObj.JointNumber]
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]

        if jointObj.fixDof is False:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.pointXYrelCoGNp[jointObj.body_I_Index, jointObj.point_I_i_Index])],
                                     [0.0, 0.0, 1.0]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.pointXYrelCoGNp[jointObj.body_J_Index, jointObj.point_J_i_Index] + diff)],
                                     [0.0, 0.0, -1.0]])
        else:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.pointXYrelCoGNp[jointObj.body_I_Index, jointObj.point_I_i_Index])],
                                     [0.0, 0.0, 1.0],
                                     [jointUnitJVec[0], jointUnitJVec[1],
                                      jointUnitJVec.dot(self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.pointXYrelCoGNp[jointObj.body_J_Index, jointObj.point_J_i_Index] + diff)],
                                     [0.0, 0.0, -1.0],
                                     [-jointUnitJVec[0], -jointUnitJVec[1],
                                      -jointUnitJVec.dot(self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])]])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Translational_Acc(self, jointObj, tick):
        """Evaluate gamma for a Translational joint"""
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
        jointUnitJDotVec = self.jointUnit_J_WorldDotNp[jointObj.JointNumber]
        jointUnitJDotRot = ST.Rot90NumPy(jointUnitJDotVec.copy())
        if jointObj.body_I_Index == 0:
            f2 = 0
        elif jointObj.body_J_Index == 0:
            f2 = 0
        else:
            f2 = jointUnitJDotVec.dot(self.worldNp[jointObj.body_I_Index] -
                                      self.worldNp[jointObj.body_J_Index]) * \
                 self.phiDotNp[jointObj.body_I_Index] - \
                 2 * jointUnitJDotRot.dot(self.worldDotNp[jointObj.body_I_Index] -
                                          self.worldDotNp[jointObj.body_J_Index])

        if jointObj.fixDof is False:
            return np.array([f2, 0.0])
        else:
            diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                   self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
            diffDot = self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                      self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
            jointUnitVec = diff/jointObj.phi0
            jointUnitVecDot = diffDot/jointObj.phi0
            f3 = -jointUnitVecDot.dot(diffDot)
            if jointObj.body_I_Index == 0:
                f3 += jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]) *
                                       self.phiDotNp[jointObj.body_J_Index])
            elif jointObj.body_J_Index == 0:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index]) *
                                       self.phiDotNp[jointObj.body_I_Index])
            else:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] *
                                                     self.phiDotNp[jointObj.body_I_Index] -
                                                     self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index] *
                                                     self.phiDotNp[jointObj.body_J_Index]))
            return np.array([f2, 0.0, f3])
    #  =========================================================================
    def Translational_Revolute_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Translational-Revolute joint"""
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
        jointUnitVecRot = self.jointUnitVecRotNp[jointObj.body_I_Index]
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        return np.array([jointUnitVecRot.dot(diff) - jointObj.lengthLink])
    #  -------------------------------------------------------------------------
    def Translational_Revolute_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Translational-Revolute joint"""
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
        jointUnitVec = self.jointUnit_I_WorldNp[jointObj.JointNumber]
        jointUnitVecRot = self.jointUnit_I_WorldRotNp[jointObj.JointNumber]
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]

        JacobianHead = np.array([jointUnitVecRot[0], jointUnitVecRot[1],
                                 jointUnitVec.dot(self.pointXYrelCoGNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - diff)])
        JacobianTail = np.array([-jointUnitVecRot[0], -jointUnitVecRot[1],
                                 -jointUnitVec.dot(self.pointXYrelCoGNp[jointObj.body_J_Index, jointObj.point_J_i_Index])])

        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Translational_Revolute_Acc(self, jointObj, tick):
        """Evaluate gamma for a Translational-Revolute joint"""
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
        jointUnitVec = self.jointUnit_I_WorldNp[jointObj.JointNumber]
        jointUnitVecDot = self.jointUnit_I_WorldDotNp[jointObj.JointNumber]
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        diffDot = self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                  self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        if jointObj.body_I_Index == 0:
            f = jointUnitVec.dot(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index] *
                                 self.phiDotNp[jointObj.body_J_Index])
        elif jointObj.body_J_Index == 0:
            f = jointUnitVecDot.dot(diff * self.phiDotNp[jointObj.body_I_Index] + 2 * ST.Rot90NumPy(diffDot)) - \
                jointUnitVec.dot(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] *
                                 self.phiDotNp[jointObj.body_I_Index])
        else:
            f = jointUnitVecDot.dot(diff * self.phiDotNp[jointObj.body_I_Index] + 2 * ST.Rot90NumPy(diffDot)) - \
                jointUnitVec.dot(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] *
                                 self.phiDotNp[jointObj.body_I_Index] - \
                self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index] *
                                 self.phiDotNp[jointObj.body_J_Index])
        return f
    #  =========================================================================
    def Driven_Revolute_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Driven Revolute joint"""
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
        if jointObj.body_I_Index == 0:
            f = -self.phiNp[jointObj.body_J_Index] - func
        elif jointObj.body_J_Index == 0:
            f = self.phiNp[jointObj.body_I_Index] - func
        else:
            f = self.phiNp[jointObj.body_I_Index] - self.phiNp[jointObj.body_J_Index] - func
        return np.array([f])
    #  -------------------------------------------------------------------------
    def Driven_Revolute_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Driven Revolute joint"""
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
    #  -------------------------------------------------------------------------
    def Driven_Revolute_Acc(self, jointObj, tick):
        """Evaluate gamma for a Driven Revolute joint"""
        if Debug:
            ST.Mess("SimMain-Driven_Revolute_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #    f = fun_dd;
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        return funcDotDot
    #  =========================================================================
    def Driven_Translational_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Driven Translational joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        return np.array([(diff.dot(diff) - func ** 2) / 2])
    #  -------------------------------------------------------------------------
    def Driven_Translational_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Driven Translational joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]

        JacobianHead = np.array([diff[0], diff[1],
                                 diff.dot(self.pointXYrelCoGrotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])])
        JacobianTail = np.array([-diff[0], -diff[1],
                                 -diff.dot(self.pointXYrelCoGrotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])])

        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Driven_Translational_Acc(self, jointObj, tick):
        """Evaluate gamma for a Driven Translational joint"""
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
        diff = self.pointXYWorldNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
               self.pointXYWorldNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        diffDot = self.pointWorldDotNp[jointObj.body_I_Index, jointObj.point_I_i_Index] - \
                  self.pointWorldDotNp[jointObj.body_J_Index, jointObj.point_J_i_Index]
        f = func * funcDotDot + funcDot**2
        if jointObj.body_I_Index == 0:
            f += diff.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])) * \
                 self.phiDotNp[jointObj.body_J_Index]
        elif jointObj.body_J_Index == 0:
            f -= diff.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])) * \
                 self.phiDotNp[jointObj.body_I_Index] + \
                 diffDot.dot(diffDot)
        else:
            f += diff.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_J_Index, jointObj.point_J_i_Index])) * \
                 self.phiDotNp[jointObj.body_J_Index] - \
                 diff.dot(ST.Rot90NumPy(self.pointXYrelCoGdotNp[jointObj.body_I_Index, jointObj.point_I_i_Index])) * \
                 self.phiDotNp[jointObj.body_I_Index] - \
                 diffDot.dot(diffDot)
        return f
    #  =========================================================================
    def Disc_constraint(self, jointObj, tick):
        """Evaluate the constraints for a Disc joint"""
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
        return np.array([(self.worldNp[jointObj.body_I_Index, 1] - jointObj.Radius),
                         ((self.worldNp[jointObj.body_I_Index, 0] - jointObj.x0) +
                          jointObj.Radius * (self.phiNp[jointObj.body_I_Index] - jointObj.phi0))])
    #  -------------------------------------------------------------------------
    def Disc_Jacobian(self, jointObj):
        """Evaluate the Jacobian for a Disc joint"""
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
    #  -------------------------------------------------------------------------
    def Disc_Acc(self, jointObj, tick):
        """Evaluate gamma for a Disc joint"""
        if Debug:
            ST.Mess("SimMain-Disc_Acc")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    f = [0; 0];
        # ==================================
        return np.array([0.0, 0.0])
    #  =========================================================================
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

        # Create the vertical headings list
        # To write each body name into the top row of the spreadsheet,
        # would make some columns very big by default
        # So body names and point names are also written vertically in
        # The column before the body/point data is written
        VerticalHeaders = []
        # Write the column headers horizontally
        for twice in range(2):
            ColumnCounter = 0
            SimResultsFILE.write("Time: ")
            # Bodies Headings
            for bodyIndex in range(1, self.numBodies):
                if twice == 0:
                    VerticalHeaders.append(self.bodyObjList[bodyIndex].Label)
                    SimResultsFILE.write("Body" + str(bodyIndex))
                    SimResultsFILE.write(" x y phi(r) phi(d) dx/dt dy/dt dphi/dt(r) dphi/dt(d) d2x/dt2 d2y/dt2 d2phi/dt2(r) d2phi/dt2(d) ")
                else:
                    SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " -"*12 + " ")
                ColumnCounter += 1
                # Points Headings
                for index in range(len(self.pointDictList[bodyIndex])):
                    if twice == 0:
                        VerticalHeaders.append(self.bodyObjList[bodyIndex].pointLabels[index])
                        SimResultsFILE.write("Point" + str(index+1) + " x y dx/dt dy/dt ")
                    else:
                        SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " -"*4 + " ")
                    ColumnCounter += 1
            # Lambda Headings
            if self.numConstraints > 0:
                for bodyIndex in range(1, self.numBodies):
                    if twice == 0:
                        VerticalHeaders.append(self.bodyObjList[bodyIndex].Label)
                        SimResultsFILE.write("Lam" + str(bodyIndex) + " x y ")
                    else:
                        SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - - ")
                    ColumnCounter += 1
            # Kinetic Energy Headings
            for bodyIndex in range(1, self.numBodies):
                if twice == 0:
                    VerticalHeaders.append(self.bodyObjList[bodyIndex].Label)
                    SimResultsFILE.write("Kin" + str(bodyIndex) + " - ")
                else:
                    SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - ")
                ColumnCounter += 1

            # Potential Energy Headings
            for forceIndex in range(self.numForces):
                forceObj = self.forceObjList[forceIndex]
                if forceObj.actuatorType == 0:
                    for bodyIndex in range(1, self.numBodies):
                        if twice == 0:
                            VerticalHeaders.append(self.bodyObjList[bodyIndex].Label)
                            SimResultsFILE.write("Pot" + str(bodyIndex) + " - ")
                        else:
                            SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - ")
                        ColumnCounter += 1

            # Energy Totals Headings
            if twice == 0:
                SimResultsFILE.write("TotKin TotPot Total\n")
            else:
                SimResultsFILE.write("\n")

        # Do the calculations for each point in time
        # Plus an extra one at time=0 (with no printing)
        VerticalCounter = 0
        TickRange = [0]
        TickRange += range(numTicks)
        for timeIndex in TickRange:
            tick = timeValues[timeIndex]
            ColumnCounter = 0
            potEnergy = 0

            # Do the analysis on the stored uResults
            self.Analysis(tick, uResults[timeIndex])

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
                    SimResultsFILE.write(str(self.worldNp[bodyIndex]*1e-3)[1:-1:] + " ")
                    # Phi (rad)
                    SimResultsFILE.write(str(self.phiNp[bodyIndex])[1:-1:] + " ")
                    # Phi (deg)
                    SimResultsFILE.write(str(self.phiNp[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")
                    # Xdot Ydot
                    SimResultsFILE.write(str(self.worldDotNp[bodyIndex]*1e-3)[1:-1:] + " ")
                    # PhiDot (rad)
                    SimResultsFILE.write(str(self.phiDotNp[bodyIndex])[1:-1:] + " ")
                    # PhiDot (deg)
                    SimResultsFILE.write(str(self.phiDotNp[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")
                    # Xdotdot Ydotdot
                    SimResultsFILE.write(str(self.worldDotDotNp[bodyIndex]*1e-3)[1:-1:] + " ")
                    # PhiDotDot (rad)
                    SimResultsFILE.write(str(self.phiDotDotNp[bodyIndex])[1:-1:] + " ")
                    # PhiDotDot (deg)
                    SimResultsFILE.write(str(self.phiDotDotNp[bodyIndex] * 180.0 / math.pi)[1:-1:] + " ")

                # Write all the points position and positionDot in the body
                for index in range(len(self.pointDictList[bodyIndex])):
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
                        SimResultsFILE.write(str(self.pointXYWorldNp[bodyIndex, index]*1e-3)[1:-1:] + " ")
                        # Point Xdot Ydot
                        SimResultsFILE.write(str(self.pointWorldDotNp[bodyIndex, index]*1e-3)[1:-1:] + " ")

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
                        SimResultsFILE.write(str(self.Lambda[bodyIndex*2] * 1e-3)[1:-1:] + " " + str(self.Lambda[bodyIndex*2 + 1] * 1e-3)[1:-1:] + " ")

            # Compute kinetic and potential energies in Joules
            totKinEnergy = 0
            for bodyIndex in range(1, self.numBodies):
                kinEnergy = 0.5e-6 * (
                        (self.massArrayNp[(bodyIndex-1) * 3] *
                         (self.worldDotNp[bodyIndex, 0] ** 2 + self.worldDotNp[bodyIndex, 1] ** 2)) +
                        (self.massArrayNp[(bodyIndex - 1) * 3 + 2] * (self.phiDotNp[bodyIndex] ** 2)))

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
            for forceIndex in range(self.numForces):
                forceObj = self.forceObjList[forceIndex]
                # Potential Energy
                potEnergy = 0
                if forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Gravity"]:
                    for bodyIndex in range(1, self.numBodies):
                        potEnergy = -self.WeightNp[bodyIndex].dot(self.worldNp[bodyIndex]) * 1e-6 - self.potEnergyZeroPointNp[bodyIndex]
                        totPotEnergy += potEnergy
                        if timeIndex == 0:
                            self.potEnergyZeroPointNp[bodyIndex] = potEnergy
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

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Spring"] or \
                    forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Linear Spring Damper"]:
                    # potEnergy += 0.5 * forceObj.k * delta**2
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring"] or \
                        forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring Damper"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Force Local to Body"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Global Force"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Torque about a Point"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Contact Friction"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Unilateral Spring Damper"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Motor"]:
                    pass

                elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Motor with Air Friction"]:
                    pass

            if timeIndex == 0:
                VerticalCounter = 0
            else:
                SimResultsFILE.write(str(totKinEnergy) + " ")
                SimResultsFILE.write(str(totPotEnergy) + " ")
                SimResultsFILE.write(str(totKinEnergy + totPotEnergy) + " ")
                SimResultsFILE.write("\n")
                VerticalCounter += 1
        # Next timeIndex

        SimResultsFILE.close()
    #  -------------------------------------------------------------------------
    def makeForceArray(self):
        if Debug:
            ST.Mess("SimMainC - makeForceArray")

        # Reset all forces and moments to zero
        for bodyIndex in range(1, self.numBodies):
            self.sumForcesNp[bodyIndex] = np.zeros((2,), dtype=np.float64)
            self.sumMomentsNp[bodyIndex] = np.zeros((1,), dtype=np.float64)

        # Add up all the body force vectors for all the bodies
        for forceIndex in range(self.numForces):
            forceObj = self.forceObjList[forceIndex]
            if forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Gravity"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'weight'}
                #            for Bi=1:nB
                #                Bodies(Bi).f = Bodies(Bi).f + Bodies(Bi).wgt;
                #            end
                for bodyIndex in range(1, self.numBodies):
                    self.sumForcesNp[bodyIndex] += self.WeightNp[bodyIndex]

            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Spring"] or \
                    forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Linear Spring Damper"]:
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

                diffNp = self.pointXYWorldNp[forceObj.body_I_Index, forceObj.point_i_Index] - \
                       self.pointXYWorldNp[forceObj.body_J_Index, forceObj.point_j_Index]
                diffDotNp = self.pointWorldDotNp[forceObj.body_I_Index, forceObj.point_i_Index] - \
                       self.pointWorldDotNp[forceObj.body_J_Index, forceObj.point_j_Index]
                length = np.sqrt(diffNp.dot(diffNp))
                lengthDot = (diffNp.dot(diffDotNp))/length
                delta = length - forceObj.LengthAngle0
                unitVecNp = diffNp/length
                # Find the component of the force in the direction of
                # the vector between the head and the tail of the force
                force = forceObj.Stiffness * delta + forceObj.DampingCoeff * lengthDot + forceObj.ForceMagnitude
                forceUnitNp = unitVecNp * force
                if forceObj.body_I_Index != 0:
                    self.sumForcesNp[forceObj.body_I_Index] -= forceUnitNp
                    self.sumMomentsNp[forceObj.body_I_Index] -= (self.pointXYrelCoGrotNp[forceObj.body_I_Index, forceObj.point_i_Index]).dot(forceUnitNp)
                if forceObj.body_J_Index != 0:
                    self.sumForcesNp[forceObj.body_J_Index] += forceUnitNp
                    self.sumMomentsNp[forceObj.body_J_Index] += self.pointXYrelCoGrotNp[forceObj.body_J_Index, forceObj.point_j_Index].dot(forceUnitNp)
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring"] or \
                    forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Rotational Spring Damper"]:
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
                if forceObj.body_I_Index == 0:
                    theta = -self.phiNp[forceObj.body_J_Index]
                    thetaDot = -self.phiDotNp[forceObj.body_J_Index]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.sumMomentsNp[forceObj.body_J_Index] += Torque
                elif forceObj.body_J_Index == 0:
                    theta = self.phiNp[forceObj.body_I_Index]
                    thetaDot = self.phiDotNp[forceObj.body_I_Index]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.sumMomentsNp[forceObj.body_I_Index] -= Torque
                else:
                    theta = self.phiNp[forceObj.body_I_Index] - self.phiNp[forceObj.body_J_Index]
                    thetaDot = self.phiDotNp[forceObj.body_I_Index] - self.phiDotNp[forceObj.body_J_Index]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.sumMomentsNp[forceObj.body_I_Index] -= Torque
                    self.sumMomentsNp[forceObj.body_J_Index] += Torque
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Unilateral Spring Damper"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                ST.Console.PrintError("Still in development\n")
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Force Local to Body"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'flocal'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).f = Bodies(Bi).f + Bodies(Bi).A*Forces(Fi).flocal;
                self.sumForcesNp[forceObj.body_I_Index] += self.RotMatPhiNp[forceObj.body_I_Index] @ forceObj.constLocalForce
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Global Force"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'f'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).f = Bodies(Bi).f + Forces(Fi).f;
                self.sumForcesNp[forceObj.body_I_Index, 0] += forceObj.constWorldForce[0]
                self.sumForcesNp[forceObj.body_I_Index, 1] += forceObj.constWorldForce[1]
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Constant Torque about a Point"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'T'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).n = Bodies(Bi).n + Forces(Fi).T;
                self.sumMomentsNp[forceObj.body_I_Index] += forceObj.constTorque
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Contact Friction"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                ST.Console.PrintError("Still in development\n")
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Motor"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                ST.Console.PrintError("Still in development\n")
            elif forceObj.actuatorType == ST.FORCE_TYPE_DICTIONARY["Motor with Air Friction"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                # TODO: Future implementation - not explicitly handled by Nikravesh
                CAD.Console.PrintError("Still in development\n")
            else:
                CAD.Console.PrintError("Unknown Force type - this should never occur\n")
        # Next forceIndex

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
            self.forceArrayNp[(bodyIndex - 1) * 3: bodyIndex * 3 - 1] = self.sumForcesNp[bodyIndex]
            self.forceArrayNp[bodyIndex * 3 - 1] = self.sumMomentsNp[bodyIndex]
        if Debug:
            ST.MessNoLF("Force Array:  ")
            ST.PrintNp1D(True, self.forceArrayNp)
    #  =========================================================================
    def initNumPyArrays(self, maxNumPoints):
        # Initialize all the NumPy arrays with zeros

        # Parameters for each body
        self.MassNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.WeightNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.momentInertiaNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.sumForcesNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.sumMomentsNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.worldNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.worldRotNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.worldDotNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.worldDotRotNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.worldDotDotNp = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.phiNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.phiDotNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.phiDotDotNp = np.zeros((self.numBodies,), dtype=np.float64)
        self.RotMatPhiNp = np.zeros((self.numBodies, 2, 2,), dtype=np.float64)
        self.potEnergyZeroPointNp = np.zeros((self.numBodies,), dtype=np.float64)

        # Parameters for each point within a body, for each body
        # Vector from CoG to the point in body local coordinates
        self.pointXiEtaNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from CoG to the point in world coordinates
        self.pointXYrelCoGNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.pointXYrelCoGrotNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.pointXYrelCoGdotNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from the origin to the point in world coordinates
        self.pointXYWorldNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.pointWorldRotNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.pointWorldDotNp = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)

        # Unit vector (if applicable) of the first body of the joint in body local coordinates
        self.jointUnit_I_XiEtaNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # Unit vector (if applicable) of the first body of the joint in world coordinates
        self.jointUnit_I_WorldNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_I_WorldRotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_I_WorldDotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_I_WorldDotRotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # Second unit vector (if applicable) of the second body of the joint in body local coordinates
        self.jointUnit_J_XiEtaNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # second unit vector (if applicable) of the second body of the joint in world coordinates
        self.jointUnit_J_WorldNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_J_WorldRotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_J_WorldDotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.jointUnit_J_WorldDotRotNp = np.zeros((self.numJoints, 2,), dtype=np.float64)

        self.forceArrayNp = np.zeros((self.numMovBodiesx3,), dtype=np.float64)
    #  -------------------------------------------------------------------------
    def __load__(self):
        if Debug:
            ST.Mess("TaskPanelSimMainClass-__load__")
        return self.Type
    #  -------------------------------------------------------------------------
    def __dump__(self, state):
        if Debug:
            ST.Mess("TaskPanelSimMainClass-__dump__")
        if state:
            self.Type = state
    #  =========================================================================
"""# Convert body object Dictionary to a Body List to ensure being ordered
        bodyObjDict = ST.getDictionary("SimBody")
        self.bodyObjList = []
        # Get the dictionary of dictionary of Points
        # i.e. {<body name> : {<point name> : <its index in the body object's list of points>}}
        DictionaryOfPoints = ST.getDictionaryOfBodyPoints()
        # Convert to a list of point dictionaries at the same index as its parent body object
        self.pointDictList = []
        bodyIndex = 0
        for bodyName in bodyObjDict:
            self.bodyObjList.append(bodyObjDict[bodyName])
            self.pointDictList.append(DictionaryOfPoints[bodyName])
            # Make sure all indices point to the correct body in case a body
            # has been subsequently deleted after joint/force definition
            # This is done by matching the index to the Body Name
            self.cleanUpIndices(bodyName, bodyIndex)
            bodyIndex += 1
        self.numBodies = bodyIndex
        self.numMovBodiesx3 = (self.numBodies-1) * 3

        # Get the plane normal rotation matrix from the main Sim container
        # This will rotate all the coordinates in the model, to be in the X-Y plane
        xyzToXYRotation = CAD.Rotation(CAD.Vector(0.0, 0.0, 1.0), ST.getContainerObject().movementPlaneNormal)

        # Find the global maximum number of points in any of the bodies
        # We will need this so we can initialise large enough NumPy arrays
        maxNumberPoints = 0
        for bodyIndex in range(self.numBodies):
            if maxNumberPoints < len(self.pointDictList[bodyIndex]):
                maxNumberPoints = len(self.pointDictList[bodyIndex])
        # Initialise the size of all the NumPy arrays and fill with zeros
        self.initNumPyArrays(maxNumberPoints)

        # Transfer all the 3D stuff into the NumPy arrays while doing the projection onto the X-Y plane
        for bodyIndex in range(self.numBodies):
            bodyObj = self.bodyObjList[bodyIndex]
            # Bring the body Mass, CoG, MoI and Weight up-to-date
            # It was already calculated after the Materials definition
            # but do it again, just in case something has changed since
            ST.computeCoGAndMomentInertia(bodyObj)
            # All Mass and moment of inertia stuff
            self.MassNp[bodyIndex] = bodyObj.Mass
            self.momentInertiaNp[bodyIndex] = bodyObj.momentInertia
            npVec = ST.CADVecToNumPy(xyzToXYRotation.toMatrix().multVec(bodyObj.weightVector))
            self.WeightNp[bodyIndex] = npVec

            # Change the local vectors to be relative to the CoG, rather than the body origin
            # The CoG in world coordinates are the world coordinates of the body
            # All points in the body are relative to this point

            # World
            CoG = xyzToXYRotation.toMatrix().multVec(bodyObj.centreOfGravity)
            npCoG = ST.CADVecToNumPy(CoG)
            self.worldNp[bodyIndex, 0:2] = npCoG
            self.worldRotNp[bodyIndex, 0:2] = ST.Rot90NumPy(npCoG.copy())
            # WorldDot
            npWorldDot = ST.CADVecToNumPy(xyzToXYRotation.toMatrix().multVec(bodyObj.worldDot))
            self.worldDotNp[bodyIndex, 0:2] = npWorldDot
            self.worldDotRotNp[bodyIndex, 0:2] = ST.Rot90NumPy(npWorldDot.copy())
            # WorldDotDot
            self.worldDotDotNp[bodyIndex, 0:2] = np.zeros((1, 2))

            # Transform the points from model Placement to World X-Y plane relative to the CoG
            vectorsRelativeCoG = bodyObj.pointLocals.copy()
            for localIndex in range(len(vectorsRelativeCoG)):
                vectorsRelativeCoG[localIndex] = xyzToXYRotation.toMatrix(). \
                    multiply(bodyObj.world.toMatrix()). \
                    multVec(vectorsRelativeCoG[localIndex]) - CoG

            # Take some trouble to make phi as nice an angle as possible
            # Because the user will maybe use it manually later and will appreciate more simplicity
            if bodyIndex == 0:
                self.phiNp[bodyIndex] = 0.0
            else:
                self.phiNp[bodyIndex] = ST.nicePhiPlease(vectorsRelativeCoG)

            # The phiDot axis vector is by definition perpendicular to the movement plane,
            # so we don't have to do any rotating from the phiDot value set in bodyObj
            self.phiDotNp[bodyIndex] = bodyObj.phiDot

            # We will now calculate the rotation matrix and use it to find the coordinates of the points
            self.RotMatPhiNp[bodyIndex] = ST.RotationMatrixNp(self.phiNp[bodyIndex])
            if Debug:
                ST.PrintNp2D(vectorsRelativeCoG)
            for pointIndex in range(len(vectorsRelativeCoG)):
                # Point Local - vector from module body CoG to the point, in body LCS coordinates
                # [This is what we needed phi for, to fix the orientation of the body]
                npVec = ST.CADVecToNumPy(vectorsRelativeCoG[pointIndex])
                self.pointXiEtaNp[bodyIndex, pointIndex, 0:2] = npVec @ self.RotMatPhiNp[bodyIndex]
                # Point Vector - vector from body CoG to the point in world coordinates
                self.pointXYrelCoGNp[bodyIndex, pointIndex, 0:2] = npVec
                self.pointXYrelCoGrotNp[bodyIndex][pointIndex] = ST.Rot90NumPy(npVec.copy())
                # Point Vector Dot
                self.pointXYrelCoGdotNp[bodyIndex][pointIndex] = np.zeros((1, 2))
                # Point World - coordinates of the point relative to the system origin - in world coordinates
                npVec += npCoG
                self.pointXYWorldNp[bodyIndex, pointIndex] = npVec
                self.pointWorldRotNp[bodyIndex, pointIndex] = ST.Rot90NumPy(npVec.copy())
                # Point World Dot
                self.pointWorldDotNp[bodyIndex][pointIndex] = np.zeros((1, 2))
            # Next pointIndex
        # Next bodyIndex

        # Print out what we have calculated for debugging
        if Debug:
            ST.Mess("Point Dictionary: ")
            for bodyIndex in range(self.numBodies):
                ST.Mess(self.pointDictList[bodyIndex])
            ST.Mess("Mass: [g]")
            ST.PrintNp1D(True, self.MassNp * 1.0e3)
            ST.Mess("")
            ST.Mess("Mass: [kg]")
            ST.PrintNp1D(True, self.MassNp)
            ST.Mess("")
            ST.Mess("Weight Vector: [kg mm /s^2 = mN]")
            ST.PrintNp2D(self.WeightNp)
            ST.Mess("")
            ST.Mess("MomentInertia: [kg.mm^2]")
            ST.Mess(self.momentInertiaNp)
            ST.Mess("")
            ST.Mess("Forces: [kg.mm/s^2 = mN]")
            ST.PrintNp2D(self.sumForcesNp)
            ST.Mess("")
            ST.Mess("Moments: [N.mm]")
            ST.PrintNp1D(True, self.sumMomentsNp)
            ST.Mess("")
            ST.Mess("World [CoG]: [mm]")
            ST.PrintNp2D(self.worldNp)
            ST.Mess("")
            ST.Mess("WorldDot: [mm/s]")
            ST.PrintNp2D(self.worldDotNp)
            ST.Mess("")
            ST.MessNoLF("phi: [deg]")
            ST.PrintNp1Ddeg(True, self.phiNp)
            ST.Mess("")
            ST.MessNoLF("phi: [rad]")
            ST.PrintNp1D(True, self.phiNp)
            ST.Mess("")
            ST.MessNoLF("phiDot: [deg/sec]")
            ST.PrintNp1Ddeg(True, self.phiDotNp)
            ST.Mess("")
            ST.MessNoLF("phiDot: [rad/sec]")
            ST.PrintNp1D(True, self.phiDotNp)
            ST.Mess("")
            ST.MessNoLF("Number of Points: ")
            ST.Mess(len(self.pointDictList[bodyIndex]))
            ST.Mess("")
            ST.Mess("PointLocal: [mm]")
            ST.PrintNp3D(self.pointXiEtaNp)
            ST.Mess("")
            ST.Mess("PointVector: [mm]")
            ST.PrintNp3D(self.pointXYrelCoGNp)
            ST.Mess("")
            ST.Mess("PointWorld: [mm]")
            ST.PrintNp3D(self.pointXYWorldNp)
            ST.Mess("")
"""