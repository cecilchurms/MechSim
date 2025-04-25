import FreeCAD as CAD

import os
import time
import numpy as np
from scipy.integrate import solve_ivp
import math

import SimTools as ST
import SimFunction
from SimTools import findBodyPhi

Debug = False
DebugArrays = False
# =============================================================================
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

        #"Revolute": 0,
        #"Revolute-Revolute": 1,
        #"Fixed": 2,
        #"Translation": 3,
        #"Translation-Revolute": 4,
        #"Disc": 5,

        # Dictionary of the pointers for Dynamic calling of the Acceleration functions
        self.dictAccelerationFunctions = {
            0: self.Revolute_gamma,
            1: self.Revolute_Revolute_gamma,
            2: self.Fixed_gamma,
            3: self.Translation_gamma,
            4: self.Translation_Revolute_gamma,
        }
        """
            5: self.Disc_gamma,
        6: self.Driven_Revolute_gamma,
        7: self.Driven_Translation_gamma,
        """
        # Dictionary of the pointers for Dynamic calling of the constraint functions
        self.dictconstraintFunctions = {
            0: self.Revolute_constraint,
            1: self.Revolute_Revolute_constraint,
            2: self.Fixed_constraint,
            3: self.Translation_constraint,
            4: self.Translation_Revolute_constraint,
        }
        """
            5: self.Disc_constraint,
        6: self.Driven_Revolute_constraint,
        7: self.Driven_Translation_constraint,
        """
        # Dictionary of the pointers for Dynamic calling of the Jacobian functions
        self.dictJacobianFunctions = {
            0: self.Revolute_Jacobian,
            1: self.Revolute_Revolute_Jacobian,
            2: self.Fixed_Jacobian,
            3: self.Translation_Jacobian,
            4: self.Translation_Revolute_Jacobian,
        }
        """
            5: self.Disc_Jacobian,
        6: self.Driven_Revolute_Jacobian,
        7: self.Driven_Translation_Jacobian,
        """
        
        # Store the required accuracy figures
        self.relativeTolerance = 10 ** (-Accuracy)
        self.absoluteTolerance = 10 ** (-Accuracy-4)

        # Counter of function evaluations
        self.Counter = 0

        # We get the objects and set global constants
        # Main container
        self.simGlobalObj = CAD.ActiveDocument.findObjects(Name="^SimGlobal$")[0]

        # Solver
        self.solverObj = CAD.ActiveDocument.findObjects(Name="^SimSolver$")[0]

        # Bodies
        self.bodyGroup = CAD.ActiveDocument.findObjects(Name="^LinkGroup")
        self.numBodies = len(self.bodyGroup)
        self.numMovBodiesx3 = (self.numBodies-1) * 3

        # Joints
        self.jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0].Group
        self.numJoints = len(self.jointGroup)

        # Forces
        self.forceList = CAD.ActiveDocument.findObjects(Name="^SimForce")
        self.numForces = len(self.forceList)

        # Set a variable to flag whether we have reached the end error-free
        # It will be available to SimSolver as an instance variable
        self.initialised = False

        # Get the plane normal rotation matrix from SimGlobal
        # This will rotate all the coordinates in the model, to be in the X-Y plane
        # ToDo Currently this is not implemented elsewhere
        self.xyzToXYRotation = CAD.Rotation(CAD.Vector(0.0, 0.0, 1.0), self.simGlobalObj.movementPlaneNormal)

        # Find the maximum number of points in any of the bodies
        # We will need this so we can initialise large enough NumPy arrays
        maxNumBodyJoints = 0
        for bodyObj in self.bodyGroup:
            if maxNumBodyJoints < len(bodyObj.jointIndexList):
                maxNumBodyJoints = len(bodyObj.jointIndexList)

        # ********************************************************************************

        # Initialise the size of all the NumPy arrays and fill with zeros
        self.initNumPyArrays(maxNumBodyJoints)

        # Transfer all the 3D stuff into the NumPy arrays
        bodyIndex = -1
        for bodyObj in self.bodyGroup:
            bodyIndex += 1
            self.NPnumJointPointsInBody[bodyIndex] = len(bodyObj.jointIndexList)

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
            self.NPbody_r[bodyIndex, 0:2] = npCoG
            self.NPbody_r90[bodyIndex, 0:2] = ST.Rot90NumPy(npCoG)

            # WorldDot
            npvec = ST.CADVecToNumPy(bodyObj.worldDot)
            self.NPbody_drdt[bodyIndex, 0:2] = npvec
            self.NPbody_drdt90[bodyIndex, 0:2] = ST.Rot90NumPy(npvec)

            # WorldDotDot
            self.NPbody_d2rdt2[bodyIndex, 0:2] = np.zeros((1, 2))

            # The angle phi which the body makes with the x-axis
            self.NPbody_phi[bodyIndex] = ST.findBodyPhi(bodyObj)

            # The phiDot axis vector is by definition perpendicular to the movement plane,
            # so we don't have to do any rotating from the phiDot value set in bodyObj
            self.NPbody_dphidt[bodyIndex] = bodyObj.phiDot

            # We will now calculate the rotation matrix
            # and use it to find the local coordinates of the points
            self.NPRotMatrixPhi[bodyIndex] = ST.CalculateRotationMatrix(self.NPbody_phi[bodyIndex])

            for pointIndex in range(len(bodyObj.jointIndexList)):
                # Point Local - vector from body CoG to the point, in body local (LCS) coordinates
                npVec = ST.CADVecToNumPy(bodyObj.PointPlacementList[pointIndex].Base)
                self.NPpoint_r[bodyIndex, pointIndex] = npVec
                self.NPpoint_r90[bodyIndex, pointIndex] = ST.Rot90NumPy(npVec)

                # Point Vector - vector from body CoG to the point in world coordinates
                npVec -= npCoG
                self.NPpoint_s[bodyIndex, pointIndex, 0:2] = npVec
                self.NPpoint_s90[bodyIndex][pointIndex] = ST.Rot90NumPy(npVec)

                # Local coordinates of the point in Eta/Xi frame
                self.NPpoint_sXiEta[bodyIndex, pointIndex, 0:2] = npVec @ self.NPRotMatrixPhi[bodyIndex]

                # Point Vector Dot
                self.NPpoint_dsdt[bodyIndex][pointIndex] = np.zeros((1, 2))

                # Point World Dot
                self.NPpoint_drdt[bodyIndex][pointIndex] = np.zeros((1, 2))

            # Next pointIndex
        # Next bodyObj

        # Print out what we have populated for debugging
        if DebugArrays:
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
            ST.Mess("World [vector r]: [mm relative to CoG]")
            ST.PrintNp2D(self.NPbody_r)
            ST.Mess("")
            ST.Mess("WorldDot [dr/dt]: [mm/s]")
            ST.PrintNp2D(self.NPbody_drdt)
            ST.Mess("")
            ST.Mess("phi: [deg]")
            ST.PrintNp1Ddeg(True, self.NPbody_phi)
            ST.Mess("")
            ST.Mess("phiDot [dphi/dt]: [deg/sec]")
            ST.PrintNp1Ddeg(True, self.NPbody_dphidt)
            ST.Mess("")
            ST.Mess("Rotation Matrix:")
            ST.PrintNp3D(self.NPRotMatrixPhi)
            ST.Mess("")
            ST.Mess("Number of Joint points in body:")
            ST.PrintNp1D(True, self.NPnumJointPointsInBody)
            ST.Mess("")
            ST.Mess("PointLocal [s(Xi) s(Eta)]: [mm]")
            ST.PrintNp3D(self.NPpoint_sXiEta)
            ST.Mess("")
            ST.Mess("Vector from CoG to point in World coordinates [s-point]: [mm]")
            ST.PrintNp3D(self.NPpoint_s)
            ST.Mess("")
            ST.Mess("Vector from Origin to point in World Coordinates [r-point]: [mm]")
            ST.PrintNp3D(self.NPpoint_r)
            ST.Mess("")
            ST.Mess("Velocity of Vector from Origin to point [dr-point/dt]:")
            ST.PrintNp3D(self.NPpoint_drdt)
            ST.Mess("")

        # Make an array with the respective body Mass and moment of inertia
        self.NPMassArray = np.zeros(self.numMovBodiesx3)
        for bodyIndex in range(1, len(self.bodyGroup)):
            self.NPMassArray[(bodyIndex-1)*3:bodyIndex*3] = (
                self.NPMasskg[bodyIndex],
                self.NPMasskg[bodyIndex],
                self.NPmomentOfInertia[bodyIndex])

        # Transfer the joint UNIT vector coordinates to the NumPy arrays
        Ji = -1
        for jointObj in self.jointGroup:
            Ji += 1

            # Ignore the joints without defined code
            if hasattr(jointObj, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                # Unit vector on Head body in world coordinates
                self.NPunit_i[Ji] = jointObj.bodyHeadUnit[:2]
                self.NPunit_i90[Ji] = ST.Rot90NumPy(self.NPunit_i[Ji])
                self.NPunit_i_vel[Ji] = CAD.Vector()[:2]
                self.NPunit_i_vel90[Ji] = CAD.Vector()[:2]

                # Unit vector on the Head body in body local coordinates
                self.NPunit_i_XiEta[Ji] = jointObj.bodyHeadUnit[:2] @ self.NPRotMatrixPhi[bodyIndex]

                # Unit vector on Tail body in world coordinates
                self.NPunit_j[Ji] = jointObj.bodyTailUnit[:2]
                self.NPunit_j90[Ji] = ST.Rot90NumPy(self.NPunit_j[Ji])
                self.NPunit_j_vel[Ji] = CAD.Vector()[:2]
                self.NPunit_j_vel90[Ji] = CAD.Vector()[:2]

                # Unit vector on the Tail body in body local coordinates
                self.NPunit_j_XiEta[Ji] = jointObj.bodyTailUnit[:2] @ self.NPRotMatrixPhi[bodyIndex]

                # PinInSlot stuff
                #unitPinInSlot = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
                #                self.NPpoint_r[jointObj.Bj, jointObj.Pj]
                #length = np.sqrt(unitPinInSlot[0]**2 + unitPinInSlot[1]**2)

                #dotProduct = self.NPunit_i[Ji].dot(unitPinInSlot)
                #if dotProduct < 0.0:
                #    jointObj.lengthLink = -length
                #else:
                #    jointObj.lengthLink = length

        # Next jointObj
        ST.PrintNp2D(self.NPunit_i)
        ST.PrintNp2D(self.NPunit_j)
        ST.PrintNp2D(self.NPunit_i_XiEta)
        ST.PrintNp2D(self.NPunit_j_XiEta)

        #J = -1
        #for jointObj in self.jointGroup:
            #J += 1
            # If the body is attached to ground then its unit vector local coordinates are world coordinates
            # Viewed alternatively, Xi / Eta are parallel to X / Y
            #if jointObj.Bi == 0:
            #    self.NPunit_i_XiEta[J] = self.NPunit_i[J]
            #    self.NPunit_i90[J] = ST.Rot90NumPy(self.NPunit_i[J])
            #if jointObj.Bj == 0:
            #    self.NPunit_j_XiEta[J] = self.NPunit_j[J]
            #    self.NPunit_j90[J] = ST.Rot90NumPy(self.NPunit_j[J])

        # Assign number of constraints and number of bodies
        # to each defined joint according to its type
        Ji = -1
        for jointObj in self.jointGroup:
            Ji += 1
            # Only allow joints currently included in the code
            if hasattr(jointObj, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:

                # ***********************************************
                # Revolute joint
                if jointObj.SimJoint == "Revolute":
                    if jointObj.FunctType == -1:
                        jointObj.mConstraints = 2
                        jointObj.nBodies = 2
                        if jointObj.fixDof is True:
                            jointObj.mConstraints = 3
                            # Set the initial angle phi0 
                            if jointObj.Bi == 0:
                                self.NPphi0[jointObj.Ji] = -self.NPbody_phi[jointObj.Bj]
                            elif jointObj.Bj == 0:
                                self.NPphi0[jointObj.Ji] = self.NPbody_phi[jointObj.Bi]
                            else:
                                self.NPphi0[jointObj.Ji] = (
                                        self.NPbody_phi[jointObj.Bi] 
                                        - self.NPbody_phi[jointObj.Bj])

                # ***********************************************
                elif jointObj.SimJoint == "Fixed":
                    jointObj.mConstraints = 3
                    jointObj.nBodies = 2
                    if jointObj.Bi == 0:
                        V = -self.NPRotMatrixPhi[jointObj.Bj].T @ self.NPbody_r[jointObj.Bj]
                        self.NPphi0[jointObj.Ji] = -self.NPbody_phi[jointObj.Bj]
                    elif jointObj.Bj == 0:
                        V = self.NPbody_r[jointObj.Bi]
                        self.NPphi0[jointObj.Ji] = self.NPbody_phi[jointObj.Bi]
                    else:
                        V = self.NPRotMatrixPhi[jointObj.Bj].T @ (self.NPbody_r[jointObj.Bi] -
                                                                          self.NPbody_r[jointObj.Bj])
                        self.NPphi0[jointObj.Ji] = self.NPbody_phi[jointObj.Bi] - \
                                        self.NPbody_phi[jointObj.Bj]
                    self.NPd0[jointObj.Ji, 0] = V[0]
                    self.NPd0[jointObj.Ji, 1] = V[1]

                # ***********************************************
                elif jointObj.SimJoint == "Translation":
                    jointObj.mConstraints = 2
                    jointObj.nBodies = 2
                    if jointObj.fixDof is True:
                        jointObj.mConstraints = 3
                        if jointObj.Bi == 0:
                            vec = (+ self.NPpoint_r[jointObj.Bi, jointObj.Pi]
                                   - self.NPbody_r[jointObj.Bj]
                                   - self.NPRotMatrixPhi[jointObj.Bj] @ self.NPpoint_sXiEta[jointObj.Bj, jointObj.Pj])
                        elif jointObj.Bj == 0:
                            vec = (- self.NPpoint_r[jointObj.Bj, jointObj.Pj]
                                   + self.NPbody_r[jointObj.Bi]
                                   + self.NPRotMatrixPhi[jointObj.Bi] @ self.NPpoint_sXiEta[jointObj.Bi, Pi])
                        else:
                            vec = (+ self.NPbody_r[jointObj.Bi]
                                   + self.NPRotMatrixPhi[jointObj.Bi] @ self.NPpoint_sXiEta[jointObj.Bi, jointObj.Pi]
                                   - self.NPbody_r[jointObj.Bj]
                                   - self.NPRotMatrixPhi[jointObj.Bj] @ self.NPpoint_sXiEta[jointObj.Bj, jointObj.Pj])
                        jointObj.phi0 = np.sqrt(vec.dot(vec))

                # ***********************************************
                elif jointObj.SimJoint == "Revolute-Revolute":
                    jointObj.mConstraints = 1
                    jointObj.nBodies = 2
                    jointObj.lengthLink = jointObj.Distance

                # ***********************************************
                elif jointObj.SimJoint == "Translation-Revolute":
                    jointObj.mConstraints = 1
                    jointObj.nBodies = 2
                    jointObj.lengthLink = jointObj.Distance

                else:
                    CAD.Console.PrintError("Unknown Joint Type - this should never occur" + str(jointObj.SimJoint) + "\n")
            # end of is valid SimJoint
        # Next Joint Object

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
            radiusVector = self.NPpoint_sXiEta[jointObj.Bi, jointObj.Pi] - \
                           self.NPpoint_sXiEta[jointObj.Bi, jointObj.Pj]
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
        if DebugArrays:
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
                self.NPbody_drdt[bodyIndex, 0], \
                self.NPbody_drdt[bodyIndex, 1], \
                self.NPbody_dphidt[bodyIndex]
        # Solve for velocity at time = 0
        # Unless the joint is Driven-Revolute or Driven-Translation
        # RHSVel = [0,0,...]   (i.e. a list of zeros)
        solution = np.linalg.solve(Jacobian @ Jacobian.T, (Jacobian @ velArrayNp)) # ToDo - self.RHSVel(0))
        deltaVel = -Jacobian.T @ solution
        if DebugArrays:
            ST.Mess("Velocity Array: ")
            ST.PrintNp1D(True, velArrayNp)
            ST.Mess("Velocity Correction Solution: ")
            ST.PrintNp1D(True, solution)
            ST.Mess("Delta velocity: ")
            ST.PrintNp1D(True, deltaVel)
            
        # Move corrected velocities back into the system
        for bodyIndex in range(1, self.numBodies):
            self.NPbody_drdt[bodyIndex, 0] += deltaVel[(bodyIndex-1)*3]
            self.NPbody_drdt[bodyIndex, 1] += deltaVel[(bodyIndex-1)*3 + 1]
            self.NPbody_dphidt[bodyIndex] += deltaVel[(bodyIndex-1)*3 + 2]
        # Report corrected coordinates and velocities
        if DebugArrays:
            ST.Mess("Corrected Positions: [mm]")
            ST.PrintNp2D(self.NPbody_r)
            ST.Mess("Corrected Phi:")
            ST.PrintNp1Ddeg(True, self.NPbody_phi)
            ST.Mess("Corrected Velocities [mm/s]:")
            ST.PrintNp2D(self.NPbody_drdt)
            ST.Mess("Corrected Angular Velocities:")
            ST.PrintNp1Ddeg(True, self.NPbody_dphidt)

        ##############################
        # START OF THE SOLUTION PROPER
        ##############################
        # Pack coordinates and velocities into the NumPy NParray
        NParray = np.zeros((self.numMovBodiesx3 * 2,), dtype=np.float64)
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            NParray[index1:index1+2] = self.NPbody_r[bodyIndex]
            NParray[index1+2] = self.NPbody_phi[bodyIndex]
            NParray[index2:index2+2] = self.NPbody_drdt[bodyIndex]
            NParray[index2+2] = self.NPbody_dphidt[bodyIndex]
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
        startTime = time.time()
        solution = solve_ivp(self.Dynamics,
                             (0.0, self.simEnd),
                             NParray,
                             method = self.solverObj.SolverType,
                             t_eval=self.Tspan,
                             rtol=self.relativeTolerance,
                             atol=self.absoluteTolerance)
        self.elapsedTime = time.time() - startTime

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
        if DebugArrays:
            ST.Mess("Input to 'Dynamics'")
            ST.PrintNp1D(True, NParray)

        # Unpack NParray into world coordinate and world velocity sub-arrays
        # [X, Y, phi, ..... velX, velY, phidot .....]
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            self.NPbody_r[bodyIndex, 0] = NParray[index1]
            self.NPbody_r[bodyIndex, 1] = NParray[index1+1]
            self.NPbody_phi[bodyIndex] = NParray[index1+2]
            self.NPbody_drdt[bodyIndex, 0] = NParray[index2]
            self.NPbody_drdt[bodyIndex, 1] = NParray[index2+1]
            self.NPbody_dphidt[bodyIndex] = NParray[index2+2]
            index1 += 3
            index2 += 3

        if DebugArrays:
            ST.Mess("Dynamics - World CoG:")
            ST.PrintNp2D(self.NPbody_r)
            ST.Mess("Dynamics - Phi:")
            ST.PrintNp1Ddeg(True, self.NPbody_phi)
            ST.Mess("Dynamics - Velocity CoG:")
            ST.PrintNp2D(self.NPbody_drdt)
            ST.Mess("Dynamics - Angular Velocity:")
            ST.PrintNp1Ddeg(True, self.NPbody_dphidt)

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
            if DebugArrays:
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
            if DebugArrays:
                ST.Mess("Dynamics - Jacobian-MassDiagonal-JacobianT Array")
                ST.PrintNp2D(JacMasJac)

            # get r-h-s of acceleration constraints at this time
            rhsAccel = self.RHSAcc(tick)
            if DebugArrays:
                ST.Mess("Dynamics - rhsAccel:")
                ST.PrintNp1D(True, rhsAccel)
                
            # Combine Force Array and rhs of Acceleration constraints into one 1D array
            rhs = np.zeros((numBodPlusConstr,), dtype=np.float64)
            rhs[0: self.numMovBodiesx3] = self.NPforceArray
            rhs[self.numMovBodiesx3:] = rhsAccel
            if DebugArrays:
                ST.Mess("Dynamics - rhs:")
                ST.PrintNp1D(True, rhs)
                
            # Solve the JacMasJac augmented with the rhs
            solvedVector = np.linalg.solve(JacMasJac, rhs)
            
            # First half of solution are the acceleration values
            accel = solvedVector[: self.numMovBodiesx3]
            
            # Second half is Lambda which is reported in the output results routine
            self.Lambda = solvedVector[self.numMovBodiesx3:]
            if DebugArrays:
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
            self.NPbody_d2rdt2[bodyIndex] = accel[accelIndex], accel[accelIndex + 1]
            self.NPbody_d2phidt2[bodyIndex] = accel[accelIndex + 2]
            
        index1 = 0
        index2 = self.numMovBodiesx3
        for bodyIndex in range(1, self.numBodies):
            NPDotArray[index1:index1+2] = self.NPbody_drdt[bodyIndex]
            NPDotArray[index1+2] = self.NPbody_dphidt[bodyIndex]
            NPDotArray[index2:index2+2] = self.NPbody_d2rdt2[bodyIndex]
            NPDotArray[index2+2] = self.NPbody_d2phidt2[bodyIndex]
            index1 += 3
            index2 += 3

        # Increment number of function evaluations
        self.Counter += 1

        return NPDotArray
    #  -------------------------------------------------------------------------
    def correctInitialConditions(self):
        """This function corrects the supplied initial conditions by making
        the body coordinates and velocities consistent with the constraints"""

        if Debug: ST.Mess("SimMain-correctInitialConditions")
        # Try Newton-Raphson iteration for n up to 20
        for n in range(20):
            # Update the points positions
            self.updatePointPositions()

            # Evaluate Deltaconstraint of the constraints at time=0
            Deltaconstraints = self.Getconstraints(0)
            if DebugArrays:
                ST.Mess("Delta constraints Result:")
                ST.PrintNp1D(True, Deltaconstraints)
                
            # Evaluate Jacobian
            Jacobian = self.GetJacobian()
            if DebugArrays:
                ST.Mess("Correction Jacobian:")
                ST.PrintNp2D(Jacobian)

            # Determine any redundancy between constraints
            redundant = np.linalg.matrix_rank(Jacobian) 
            if redundant < self.numConstraints:
                CAD.Console.PrintError('The constraints exhibit Redundancy while correcting initial conditions\n')
                return False

            # We have successfully converged if the ||Deltaconstraint|| is very small
            DeltaconstraintLengthSq = 0
            for index in range(self.numConstraints):
                DeltaconstraintLengthSq += Deltaconstraints[index] ** 2
            if Debug: ST.Mess("Total constraint Error: " + str(math.sqrt(DeltaconstraintLengthSq)))
            if DeltaconstraintLengthSq < 1.0e-16:
                return True

            # Solve for the new corrections
            solution = np.linalg.solve(Jacobian @ Jacobian.T, Deltaconstraints)
            delta = - Jacobian.T @ solution
            # Correct the estimates
            for bodyIndex in range(1, self.numBodies):
                self.NPbody_r[bodyIndex, 0] += delta[(bodyIndex-1)*3]
                self.NPbody_r[bodyIndex, 1] += delta[(bodyIndex-1)*3+1]
                self.NPbody_phi[bodyIndex] += delta[(bodyIndex-1)*3+2]
                
        CAD.Console.PrintError("Newton-Raphson Correction failed to converge\n\n")
        return True
    #  -------------------------------------------------------------------------
    def updatePointPositions(self):
        """Here we update the positions as the bodies move and rotate"""

        for bodyIndex in range(1, self.numBodies):

            # Compute the Rotation Matrix
            self.NPRotMatrixPhi[bodyIndex] = ST.CalculateRotationMatrix(self.NPbody_phi[bodyIndex])

            if DebugArrays:
                ST.Mess("Update Point Positions:")
                ST.MessNoLF("In Xi-Eta Coordinates           ")
                ST.MessNoLF("Relative to CoG                 ")
                ST.MessNoLF("Relative to CoG Rotated 90      ")
                ST.Mess("World Coordinates               ")
            for pointIndex in range(self.NPnumJointPointsInBody[bodyIndex]):
                pointVector = self.NPRotMatrixPhi[bodyIndex] @ self.NPpoint_sXiEta[bodyIndex, pointIndex]
                self.NPpoint_s[bodyIndex, pointIndex] = pointVector
                self.NPpoint_s90[bodyIndex][pointIndex] = ST.Rot90NumPy(pointVector)
                self.NPpoint_r[bodyIndex, pointIndex] = self.NPbody_r[bodyIndex] + pointVector
                if DebugArrays:
                    ST.PrintNp1D(False, self.NPpoint_sXiEta[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.NPpoint_s[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(False, self.NPpoint_s90[bodyIndex][pointIndex])
                    ST.MessNoLF("   ")
                    ST.PrintNp1D(True, self.NPpoint_r[bodyIndex][pointIndex])
        # Next bodyIndex
    #  -------------------------------------------------------------------------
    def updatePointVelocities(self):
        if Debug: ST.Mess("SimMain-updatePointVelocities")
        for bodyIndex in range(1, self.numBodies):
            for pointIndex in range(self.NPnumJointPointsInBody[bodyIndex]):
                deltaVelVector = self.NPpoint_s90[bodyIndex, pointIndex] * self.NPbody_dphidt[bodyIndex]
                self.NPpoint_dsdt[bodyIndex, pointIndex] = deltaVelVector
                self.NPpoint_drdt[bodyIndex, pointIndex] = self.NPbody_drdt[bodyIndex] + deltaVelVector
        # for forceObj in self.NPforceObjList:
        #   if forceObj.forceType != 0:
        #        if forceObj.Bi != 0:
        #            forceObj.FUnitHeadWorldDot = ST.Rot90NumPy(forceObj.FUnitHeadWorld) * self.NPbody_dphidt[forceObj.Bi]
    #  -------------------------------------------------------------------------
    def Getconstraints(self, tick):
        """Returns a numConstraints-long vector which contains the current deviation
        from the defined constraints"""
        if Debug: ST.Mess("SimMain-constraints")

        deltaConstraintNp = np.zeros((self.numConstraints,), dtype=np.float64)
        
        # Call the applicable function which is pointed to by the constraint function dictionary
        for jointObj in self.jointGroup:
            if hasattr(jointObj, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                constraint = self.dictconstraintFunctions[ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint]](jointObj, tick)
                deltaConstraintNp[jointObj.rowStart: jointObj.rowEnd] = constraint

        return deltaConstraintNp
    #  =========================================================================
    def GetJacobian(self):
        """Returns the Jacobian matrix numConstraints X (3 x numMovBodies)"""
        if Debug: ST.Mess("SimMain-Jacobian")
        Jacobian = np.zeros((self.numConstraints, self.numMovBodiesx3,))
        for jointObj in self.jointGroup:
            if ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint] < ST.MAXJOINTS:
                # Call the applicable function which is pointed to by the Jacobian dictionary
                JacobianHead, JacobianTail = self.dictJacobianFunctions[ST.JOINT_TYPE_DICTIONARY[jointObj.SimJoint]](jointObj)
                # Fill in the values in the Jacobian
                if jointObj.Bi != 0:
                    columnHeadStart = (jointObj.Bi-1) * 3
                    columnHeadEnd = jointObj.Bi * 3
                    Jacobian[jointObj.rowStart: jointObj.rowEnd, columnHeadStart: columnHeadEnd] = JacobianHead
                if jointObj.Bj != 0:
                    columnTailStart = (jointObj.Bj-1) * 3
                    columnTailEnd = jointObj.Bj * 3
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

        constraintNp = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
                       self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        if DebugArrays:
            ST.Mess('Revolute Constraint:')
            ST.MessNoLF('    Point I: ')
            ST.PrintNp1D(True, self.NPpoint_r[jointObj.Bi, jointObj.Pi])
            ST.MessNoLF('    Point J: ')
            ST.PrintNp1D(True, self.NPpoint_r[jointObj.Bj, jointObj.Pj])
            ST.MessNoLF('    Difference Vector: ')
            ST.PrintNp1D(True, constraintNp)

        if jointObj.fixDof:
            if jointObj.Bi == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (-self.NPbody_phi[jointObj.Bj] - self.NPphi0[jointObj.Ji])])
            elif jointObj.Bj == 0:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.NPbody_phi[jointObj.Bi] - self.NPphi0[jointObj.Ji])])
            else:
                constraintNp = np.array([constraintNp[0],
                                         constraintNp[1],
                                         (self.NPbody_phi[jointObj.Bi]
                                          - self.NPbody_phi[jointObj.Bj]
                                          - self.NPphi0[jointObj.Ji])])
        return constraintNp
    #  -------------------------------------------------------------------------
    def Revolute_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Revolute joint
        if Debug: ST.Mess("SimMain-Revolute_Jacobian")

        if jointObj.fixDof is False:
            JacobianHead = np.array([
                [1.0, 0.0, self.NPpoint_s90[jointObj.Bi, jointObj.Pi, 0]],
                [0.0, 1.0, self.NPpoint_s90[jointObj.Bi, jointObj.Pi, 1]]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.NPpoint_s90[jointObj.Bj, jointObj.Pj, 0]],
                [0.0, -1.0, -self.NPpoint_s90[jointObj.Bj, jointObj.Pj, 1]]])
        else:
            JacobianHead = np.array([
                [1.0, 0.0, self.NPpoint_s90[jointObj.Bi, jointObj.Pi, 0]],
                [0.0, 1.0, self.NPpoint_s90[jointObj.Bi, jointObj.Pi, 1]],
                [0.0, 0.0, 1.0]])
            JacobianTail = np.array([
                [-1.0, 0.0, -self.NPpoint_s90[jointObj.Bj, jointObj.Pj, 0]],
                [0.0, -1.0, -self.NPpoint_s90[jointObj.Bj, jointObj.Pj, 1]],
                [0.0, 0.0, -1.0]])
        if DebugArrays:
            ST.Mess("Jacobian Head")
            ST.PrintNp2D(JacobianHead)
            ST.Mess("Jacobian Tail")
            ST.PrintNp2D(JacobianTail)

        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Revolute_gamma(self, jointObj, tick):
        # Evaluate gamma for a Revolute joint
        if Debug: ST.Mess("SimMain-Revolute_gamma")
        # ==================================
        if jointObj.Bi == 0:
            gammai = np.array([0.0, 0.0])
        else:
            gammai = -ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi]) * \
                     self.NPbody_dphidt[jointObj.Bi]

        if jointObj.Bj == 0:
            gammaj = np.array([0.0, 0.0])
        else:
            gammaj = ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj]) * \
                     self.NPbody_dphidt[jointObj.Bj]

        gamma = gammai + gammaj

        if jointObj.fixDof:
            gamma = np.array([gamma[0], gamma[1], 0.0])

        return gamma

    #  =========================================================================
    def Revolute_Revolute_constraint(self, jointObj, tick):
        """ Evaluate the constraints for a Revolute-Revolute joint """
        if Debug: ST.Mess("SimMain-Revolute_Revolute_constraint")

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        return np.array([(diff.dot(diff) - jointObj.lengthLink ** 2) / 2.0])

    #  -------------------------------------------------------------------------
    def Revolute_Revolute_Jacobian(self, jointObj):
        """ Evaluate the Jacobian for a Revolute-Revolute joint """
        if Debug: ST.Mess("SimMain-Revolute_Revolute_Jacobian")

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        jointUnitVec = diff / jointObj.lengthLink

        JacobianHead = np.array([jointUnitVec[0], jointUnitVec[1], jointUnitVec.dot(self.NPpoint_s90[jointObj.Bi, jointObj.Pi])])
        JacobianTail = np.array([-jointUnitVec[0], -jointUnitVec[1], -jointUnitVec.dot(self.NPpoint_s90[jointObj.Bj, jointObj.Pj])])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Revolute_Revolute_gamma(self, jointObj, tick):
        """ Evaluate gamma for a Revolute-Revolute joint """
        if Debug: ST.Mess("SimMain-Revolute_Revolute_gamma")

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        diffDot = self.NPpoint_drdt[jointObj.Bi, jointObj.Pi] - self.NPpoint_drdt[jointObj.Bj, jointObj.Pj]
        jointUnitVec = diff / jointObj.lengthLink
        jointUnitVecDot = diffDot / jointObj.lengthLink

        gamma = -jointUnitVecDot.dot(diffDot)

        if jointObj.Bi == 0:
            gammai = np.array([0.0, 0.0])
        else:
            gammai = ST.Rot90NumPy(jointUnitVec).dot(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi] *
                                                    self.NPbody_dphidt[jointObj.Bi])

        if jointObj.Bj == 0:
            gammaj = np.array([0.0, 0.0])
        else:
            gammaj = ST.Rot90NumPy(jointUnitVec).dot(-self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj] *
                                                      self.NPbody_dphidt[jointObj.Bj])

        return gamma + gammai + gammaj

    #  =========================================================================
    def Fixed_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Fixed joint
        if Debug: ST.Mess("SimMain-Fixed_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #
        #    if Bi == 0
        #        f = [ -(Bodies(Bj).r + Bodies(Bj).A*Joints(Ji).d0)
        #              -Bodies(Bj).phi - Joints(Ji).phi0];
        #    elseif Bj == 0
        #        f = [Bodies(Bi).r - Joints(Ji).d0
        #             Bodies(Bi).phi - Joints(Ji).phi0];
        #    else
        #        f = [Bodies(Bi).r - (Bodies(Bj).r + Bodies(Bj).A*Joints(Ji).d0)
        #             Bodies(Bi).phi - Bodies(Bj).phi - Joints(Ji).phi0];
        #    end
        # ==================================
        if jointObj.Bi == 0:
            return np.array([-(self.NPbody_r[jointObj.Bj] +
                               self.NPRotMatrixPhi[jointObj.Bj] @ self.NPd0[jointObj.Ji]),
                             -self.NPbody_phi[jointObj.Bj] - self.NPphi0[jointObj.Ji]])
        elif jointObj.Bj == 0:
            return np.array([self.NPbody_r[jointObj.Bi] - self.NPd0[jointObj.Ji],
                             self.NPbody_phi[jointObj.Bi] - self.NPphi0[jointObj.Ji]])
        else:
            return np.array([self.NPbody_r[jointObj.Bi] -
                             (self.NPbody_r[jointObj.Bj] +
                              self.NPRotMatrixPhi[jointObj.Bj] @ self.NPd0[jointObj.Ji]),
                             self.NPbody_phi[jointObj.Bi] -
                             self.NPbody_phi[jointObj.Bj] -
                             self.NPphi0[jointObj.Ji]])
    #  -------------------------------------------------------------------------
    def Fixed_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Fixed joint
        if Debug: ST.Mess("SimMain-Fixed_Jacobian")
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
        if jointObj.Bj != 0:
            tailVector = ST.Rot90NumPy(self.NPRotMatrixPhi[jointObj.Bj] @ self.NPd0[jointObj.Ji])
            JacobianTail = np.array([[-1.0, 0.0, -tailVector[0]],
                                     [0.0, -1.0, -tailVector[1]],
                                     [0.0, 0.0, -1.0]])
        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Fixed_gamma(self, jointObj, tick):
        # Evaluate gamma for a Fixed joint
        if Debug: ST.Mess("SimMain-Fixed_gamma")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Bj = Joints(Ji).jBindex;
        #
        #    f = [0; 0; 0];
        #    if Bj ~= 0
        #        f = [-Bodies(Bj).A*Joints(Ji).d0*Bodies(Bj).dphidt^2; 0];
        #
        #    end
        # ==================================
        if jointObj.Bj != 0:
            tailVector = -self.NPRotMatrixPhi[jointObj.Bj] @ (self.NPd0[jointObj.Ji] *
                                                                     (self.NPbody_dphidt[jointObj.Bj]**2))
            return np.array([tailVector[0], tailVector[1], 0.0])
        else:
            return np.array([0.0, 0.0, 0.0])
    #  =========================================================================
    def Translation_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Translation joint
        if Debug: ST.Mess("SimMain-Translation_constraint")

        jointUnitJRot = self.NPunit_j90[jointObj.Ji]
        jointUnitIVec = self.NPunit_i[jointObj.Ji]

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        if DebugArrays:
            ST.Mess('Translation Constraint:')
            ST.MessNoLF('    Unit I Vector: ')
            ST.PrintNp1D(True, jointUnitIVec)
            ST.MessNoLF('    Unit J Vector Rotated: ')
            ST.PrintNp1D(True, jointUnitJRot)
            ST.MessNoLF('    World I: ')
            ST.PrintNp1D(True, self.NPpoint_r[jointObj.Bi, jointObj.Pi])
            ST.MessNoLF('    World J: ')
            ST.PrintNp1D(True, self.NPpoint_r[jointObj.Bj, jointObj.Pj])
            ST.MessNoLF('    Difference vector: ')
            ST.PrintNp1D(True, diff)

        if jointObj.fixDof is False:
            if DebugArrays:
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
    def Translation_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Translation joint
        if Debug: ST.Mess("SimMain-Translation_Jacobian")

        jointUnitJVec = self.NPunit_j[jointObj.Ji]
        jointUnitJRot = self.NPunit_j90[jointObj.Ji]
        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        if jointObj.fixDof is False:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.NPpoint_s[jointObj.Bi, jointObj.Pi])],
                                     [0.0, 0.0, 1.0]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.NPpoint_s[jointObj.Bj, jointObj.Pj] + diff)],
                                     [0.0, 0.0, -1.0]])
        else:
            JacobianHead = np.array([[jointUnitJRot[0], jointUnitJRot[1],
                                      jointUnitJVec.dot(self.NPpoint_s[jointObj.Bi, jointObj.Pi])],
                                     [0.0, 0.0, 1.0],
                                     [jointUnitJVec[0], jointUnitJVec[1],
                                      jointUnitJVec.dot(self.NPpoint_s90[jointObj.Bi, jointObj.Pi])]])
            JacobianTail = np.array([[-jointUnitJRot[0], -jointUnitJRot[1],
                                      -jointUnitJVec.dot(self.NPpoint_s[jointObj.Bj, jointObj.Pj] + diff)],
                                     [0.0, 0.0, -1.0],
                                     [-jointUnitJVec[0], -jointUnitJVec[1],
                                      -jointUnitJVec.dot(self.NPpoint_s90[jointObj.Bj, jointObj.Pj])]])
        return JacobianHead, JacobianTail

    #  -------------------------------------------------------------------------
    def Translation_gamma(self, jointObj, tick):
        # Evaluate gamma for a Translation joint
        if Debug: ST.Mess("SimMain-Translation_gamma")

        jointUnitJDotVec = self.NPunit_j_vel[jointObj.Ji]
        jointUnitJDotRot = ST.Rot90NumPy(jointUnitJDotVec)
        if jointObj.Bi == 0 or jointObj.Bj == 0:
            f2 = 0
        else:
            f2 = jointUnitJDotVec.dot(self.NPbody_r[jointObj.Bi] - self.NPbody_r[jointObj.Bj]) * \
                 self.NPbody_dphidt[jointObj.Bi] - \
                 2 * jointUnitJDotRot.dot(self.NPbody_drdt[jointObj.Bi] - self.NPbody_drdt[jointObj.Bj])

        if jointObj.fixDof is False:
            return np.array([f2, 0.0])
        else:
            diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
                   self.NPpoint_r[jointObj.Bj, jointObj.Pj]
            diffDot = self.NPpoint_drdt[jointObj.Bi, jointObj.Pi] - \
                      self.NPpoint_drdt[jointObj.Bj, jointObj.Pj]
            jointUnitVec = diff/jointObj.phi0
            jointUnitVecDot = diffDot/jointObj.phi0
            f3 = -jointUnitVecDot.dot(diffDot)
            if jointObj.Bi == 0:
                f3 += jointUnitVec.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj]) *
                                       self.NPbody_dphidt[jointObj.Bj])
            elif jointObj.Bj == 0:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi]) *
                                       self.NPbody_dphidt[jointObj.Bi])
            else:
                f3 -= jointUnitVec.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi] *
                                                     self.NPbody_dphidt[jointObj.Bi] -
                                                     self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj] *
                                                     self.NPbody_dphidt[jointObj.Bj]))
            return np.array([f2, 0.0, f3])

    #  =========================================================================
    def Translation_Revolute_constraint(self, jointObj, tick):
        """ Evaluate the constraints for a Translation-Revolute joint """
        if Debug: ST.Mess("SimMain-Translation_Revolute_constraint")

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        return np.array([self.NPunit_i[jointObj.Bi].dot(diff) - jointObj.lengthLink])
    #  -------------------------------------------------------------------------
    def Translation_Revolute_Jacobian(self, jointObj):
        """ Evaluate the Jacobian for a Translation-Revolute joint """
        if Debug: ST.Mess("SimMain-Translation_Revolute_Jacobian")

        jointUnitVec = self.NPunit_i[jointObj.Ji]
        jointUnitVecRot = self.NPunit_i90[jointObj.Ji]
        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        JacobianHead = np.array([jointUnitVecRot[0], jointUnitVecRot[1],
                                 jointUnitVec.dot(self.NPpoint_s[jointObj.Bi, jointObj.Pi] - diff)])
        JacobianTail = np.array([-jointUnitVecRot[0], -jointUnitVecRot[1],
                                 -jointUnitVec.dot(self.NPpoint_s[jointObj.Bj, jointObj.Pj])])

        return JacobianHead, JacobianTail
    #  -------------------------------------------------------------------------
    def Translation_Revolute_gamma(self, jointObj, tick):
        """ Evaluate gamma for a Translation-Revolute joint """
        if Debug: ST.Mess("SimMain-Translation_Revolute_gamma")

        jointUnitVec = self.NPunit_i[jointObj.Ji]
        jointUnitVecDot = self.NPunit_i_vel[jointObj.Ji]

        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        diffDot = self.NPpoint_drdt[jointObj.Bi, jointObj.Pi] - self.NPpoint_drdt[jointObj.Bj, jointObj.Pj]

        if jointObj.Bi == 0:
            gammai = 0.0
        else:
            gammai = jointUnitVecDot.dot(diff * self.NPbody_dphidt[jointObj.Bi] + 2 * ST.Rot90NumPy(diffDot)) - \
                     jointUnitVec.dot(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi] * self.NPbody_dphidt[jointObj.Bi])

        if jointObj.Bj == 0:
            gammaj= 0.0
        else:
            gammaj = jointUnitVec.dot(self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj] * self.NPbody_dphidt[jointObj.Bj])

        return gammai + gammaj
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
        #        f = -Bodies(Bj).phi - fun;
        #    elseif Bj == 0
        #        f =  Bodies(Bi).phi - fun;
        #    else
        #        f =  Bodies(Bi).phi - Bodies(Bj).phi - fun;
        #    end
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        if jointObj.Bi == 0:
            f = -self.NPbody_phi[jointObj.Bj] - func
        elif jointObj.Bj == 0:
            f = self.NPbody_phi[jointObj.Bi] - func
        else:
            f = self.NPbody_phi[jointObj.Bi] - self.NPbody_phi[jointObj.Bj] - func
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
    def Driven_Revolute_gamma(self, jointObj, tick):
        # Evaluate gamma for a Driven Revolute joint
        if Debug:
            ST.Mess("SimMain-Driven_Revolute_gamma")
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
    def Driven_Translation_constraint(self, jointObj, tick):
        # Evaluate the constraints for a Driven Translation joint
        if Debug:
            ST.Mess("SimMain-Driven_Translation_constraint")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).Pr - Points(Pj).Pr;
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #        f = (d'*d - fun^2)/2;
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        return np.array([(diff.dot(diff) - func ** 2) / 2])
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Translation_Jacobian(self, jointObj):
        # Evaluate the Jacobian for a Driven Translation joint
        if Debug:
            ST.Mess("SimMain-Driven_Translation_Jacobian")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    d  = Points(Pi).Pr - Points(Pj).Pr;
        #        Di = [ d'  d'*Points(Pi).Ps_90];
        #        Dj = [-d' -d'*Points(Pj).Ps_90];
        # ==================================
        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]

        JacobianHead = np.array([diff[0], diff[1],
                                 diff.dot(self.NPpoint_s90[jointObj.Bi, jointObj.Pi])])
        JacobianTail = np.array([-diff[0], -diff[1],
                                 -diff.dot(self.NPpoint_s90[jointObj.Bj, jointObj.Pj])])

        return JacobianHead, JacobianTail
        """
    #  -------------------------------------------------------------------------
    """
    def Driven_Translation_gamma(self, jointObj, tick):
        # Evaluate gamma for a Driven Translation joint
        if Debug:
            ST.Mess("SimMain-Drven_Translation_gamma")
        # ==================================
        # Matlab Code from Nikravesh: DAP_BC
        # ==================================
        #    Pi = Joints(Ji).iPindex;
        #    Pj = Joints(Ji).jPindex;
        #    Bi = Joints(Ji).iBindex;
        #    Bj = Joints(Ji).jBindex;
        #    d  = Points(Pi).Pr - Points(Pj).Pr;
        #    d_d  = Points(Pi).Pdrdt - Points(Pj).Pdrdt;
        #    [fun, fun_d, fun_dd] = functs(Joints(Ji).iFunct, t);
        #
        #    f = fun*fun_dd + fun_d^2;
        #    if Bi == 0
        #        f = f + d'*s_rot(Points(Pj).Pdsdt)*Bodies(Bj).dphidt;
        #    elseif Bj == 0
        #        f = f - d'*s_rot(Points(Pi).Pdsdt)*Bodies(Bi).dphidt - d_d'*d_d;
        #    else
        #        f = f + d'*s_rot(Points(Pj).Pdsdt)*Bodies(Bj).dphidt ...
        #              - d'*s_rot(Points(Pi).Pdsdt)*Bodies(Bi).dphidt - d_d'*d_d;
        #    end
        # ==================================
        [func, funcDot, funcDotDot] = self.driverObjDict[jointObj.Name].getFofT(jointObj.FunctType, tick)
        diff = self.NPpoint_r[jointObj.Bi, jointObj.Pi] - \
               self.NPpoint_r[jointObj.Bj, jointObj.Pj]
        diffDot = self.NPpoint_drdt[jointObj.Bi, jointObj.Pi] - \
                  self.NPpoint_drdt[jointObj.Bj, jointObj.Pj]
        f = func * funcDotDot + funcDot**2
        if jointObj.Bi == 0:
            f += diff.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj])) * \
                 self.NPbody_dphidt[jointObj.Bj]
        elif jointObj.Bj == 0:
            f -= diff.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi])) * \
                 self.NPbody_dphidt[jointObj.Bi] + \
                 diffDot.dot(diffDot)
        else:
            f += diff.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bj, jointObj.Pj])) * \
                 self.NPbody_dphidt[jointObj.Bj] - \
                 diff.dot(ST.Rot90NumPy(self.NPpoint_dsdt[jointObj.Bi, jointObj.Pi])) * \
                 self.NPbody_dphidt[jointObj.Bi] - \
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
        #           Joints(Ji).R*(Bodies(Bi).phi - Joints(Ji).phi0))];
        # ==================================
        return np.array([(self.NPbody_r[jointObj.Bi, 1] - jointObj.Radius),
                         ((self.NPbody_r[jointObj.Bi, 0] - jointObj.x0) +
                          jointObj.Radius * (self.NPbody_phi[jointObj.Bi] - jointObj.phi0))])
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
    def Disc_gamma(self, jointObj, tick):
        #Evaluate gamma for a Disc joint
        if Debug:
            ST.Mess("SimMain-Disc_gamma")
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
                #  d  = Points(Pi).Pr - Points(Pj).Pr;
                #  d_dot = Points(Pi).Pdrdt - Points(Pj).Pdrdt;
                #  L  = sqrt(d'*d);
                #  L_dot = d'*d_dot/L;
                #  del = L - Forces(Fi).L0;
                #  u = d/L;
                #
                #  f = Forces(Fi).k*del + Forces(Fi).dc*L_dot + Forces(Fi).f_a;
                #  fi = f*u;
                #  if Bi ~= 0
                #    Bodies(Bi).f = Bodies(Bi).f - fi;
                #    Bodies(Bi).n = Bodies(Bi).n - Points(Pi).Ps_90'*fi;
                #  end
                #  if Bj ~= 0
                #    Bodies(Bj).f = Bodies(Bj).f + fi;
                #    Bodies(Bj).n = Bodies(Bj).n + Points(Pj).Ps_90'*fi;
                #  end

                diffNp = self.NPpoint_r[forceObj.Bi, forceObj.Pi] - \
                       self.NPpoint_r[forceObj.Bj, forceObj.Pj]
                diffDotNp = self.NPpoint_drdt[forceObj.Bi, forceObj.Pi] - \
                       self.NPpoint_drdt[forceObj.Bj, forceObj.Pj]
                length = np.sqrt(diffNp.dot(diffNp))
                lengthDot = (diffNp.dot(diffDotNp))/length
                delta = length - forceObj.LengthAngle0
                unitVecNp = diffNp/length
                # Find the component of the force in the direction of
                # the vector between the head and the tail of the force
                force = forceObj.Stiffness * delta + forceObj.DampingCoeff * lengthDot + forceObj.ForceMagnitude
                forceUnitNp = unitVecNp * force
                if forceObj.Bi != 0:
                    self.NPsumForces[forceObj.Bi] -= forceUnitNp
                    self.NPsumMoments[forceObj.Bi] -= (self.NPpoint_s90[forceObj.Bi, forceObj.Pi]).dot(forceUnitNp)
                if forceObj.Bj != 0:
                    self.NPsumForces[forceObj.Bj] += forceUnitNp
                    self.NPsumMoments[forceObj.Bj] += self.NPpoint_s90[forceObj.Bj, forceObj.Pj].dot(forceUnitNp)
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
                #        theta   = -Bodies(Bj).phi;
                #        theta_d = -Bodies(Bj).dphidt;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bj).n = Bodies(Bj).n + T;
                #    elseif Bj == 0
                #        theta   = Bodies(Bi).phi;
                #        theta_d = Bodies(Bi).dphidt;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bi).n = Bodies(Bi).n - T;
                #    else
                #        theta   = Bodies(Bi).phi - Bodies(Bj).phi;
                #        theta_d = Bodies(Bi).dphidt - Bodies(Bj).dphidt;
                #        T = Forces(Fi).k*(theta - Forces(Fi).theta0) + ...
                #            Forces(Fi).dc*theta_d + Forces(Fi).T_a;
                #        Bodies(Bi).n = Bodies(Bi).n - T;
                #        Bodies(Bj).n = Bodies(Bj).n + T;
                #    end
                if forceObj.Bi == 0:
                    theta = -self.NPbody_phi[forceObj.Bj]
                    thetaDot = -self.NPbody_dphidt[forceObj.Bj]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.Bj] += Torque
                elif forceObj.Bj == 0:
                    theta = self.NPbody_phi[forceObj.Bi]
                    thetaDot = self.NPbody_dphidt[forceObj.Bi]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.Bi] -= Torque
                else:
                    theta = self.NPbody_phi[forceObj.Bi] - self.NPbody_phi[forceObj.Bj]
                    thetaDot = self.NPbody_dphidt[forceObj.Bi] - self.NPbody_dphidt[forceObj.Bj]
                    Torque = forceObj.Stiffness * (theta - forceObj.LengthAngle0) + \
                             forceObj.DampingCoeff * thetaDot + \
                             forceObj.TorqueMagnitude
                    self.NPsumMoments[forceObj.Bi] -= Torque
                    self.NPsumMoments[forceObj.Bj] += Torque
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
                self.NPsumForces[forceObj.Bi] += self.NPRotMatrixPhi[forceObj.Bi] @ forceObj.constLocalForce
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Global Force"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'f'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).f = Bodies(Bi).f + Forces(Fi).f;
                self.NPsumForces[forceObj.Bi, 0] += forceObj.constWorldForce[0]
                self.NPsumForces[forceObj.Bi, 1] += forceObj.constWorldForce[1]
                """
            """
            elif forceObj.forceType == ST.FORCE_TYPE_DICTIONARY["Constant Torque about a Point"]:
                # ==================================
                # Matlab Code from Nikravesh: DAP_BC
                # ==================================
                #        case {'T'}
                #            Bi = Forces(Fi).iBindex;
                #            Bodies(Bi).n = Bodies(Bi).n + Forces(Fi).T;
                self.NPsumMoments[forceObj.Bi] += forceObj.constTorque
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

        if DebugArrays:
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
        self.NPbody_r = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPbody_r90 = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPbody_drdt = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPbody_drdt90 = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPbody_d2rdt2 = np.zeros((self.numBodies, 2,), dtype=np.float64)
        self.NPbody_phi = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPbody_dphidt = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPbody_d2phidt2 = np.zeros((self.numBodies,), dtype=np.float64)
        self.NPRotMatrixPhi = np.zeros((self.numBodies, 2, 2,), dtype=np.float64)
        self.NPnumJointPointsInBody = np.zeros((self.numBodies,), dtype=np.integer)

        self.NPphi0 = np.zeros((self.numJoints), dtype=np.float64)
        self.NPd0 = np.zeros((self.numJoints, 2), dtype=np.float64)

        self.NPpotEnergyZeroPoint = np.zeros((self.numBodies,), dtype=np.float64)

        # Parameters for each point within a body, for each body
        # Vector from CoG to the point in body local coordinates
        self.NPpoint_sXiEta = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from CoG to the point in world coordinates
        self.NPpoint_s = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpoint_s90 = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpoint_dsdt = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        # Vector from the origin to the point in world coordinates
        self.NPpoint_r = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpoint_r90 = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)
        self.NPpoint_drdt = np.zeros((self.numBodies, maxNumPoints, 2,), dtype=np.float64)

        # Unit vector (if applicable) of the first body of the joint in body local coordinates
        self.NPunit_i_XiEta = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # Unit vector (if applicable) of the first body of the joint in world coordinates
        self.NPunit_i = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_i90 = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_i_vel = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_i_vel90 = np.zeros((self.numJoints, 2,), dtype=np.float64)

        # Second unit vector (if applicable) of the second body of the joint in body local coordinates
        self.NPunit_j_XiEta = np.zeros((self.numJoints, 2,), dtype=np.float64)
        # second unit vector (if applicable) of the second body of the joint in world coordinates
        self.NPunit_j = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_j90 = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_j_vel = np.zeros((self.numJoints, 2,), dtype=np.float64)
        self.NPunit_j_vel90 = np.zeros((self.numJoints, 2,), dtype=np.float64)

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
            if threeLines == "first":
                SimResultsFILE.write("Time: ")
            elif threeLines == "second":
                SimResultsFILE.write(self.solverObj.SolverType+" ")
            else:
                SimResultsFILE.write(str(int(self.elapsedTime)) + " ")
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
                    for Ji in range(self.NPnumJointPointsInBody[bodyIndex]):
                        if threeLines == "first":
                            VerticalHeaders.append(bodyObj.jointNameList[Ji])
                            SimResultsFILE.write("J" + str(Ji+1) + " x y dx dy ")
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
                        SimResultsFILE.write("K" + str(bodyIndex) + " Lin Ang ")
                    elif threeLines == "second":
                        SimResultsFILE.write("- Kin Kin ")
                    else:
                        SimResultsFILE.write(VerticalHeaders[ColumnCounter] + " - - ")
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
                SimResultsFILE.write("Total Total\n")
            elif threeLines == "second":
                SimResultsFILE.write("Kinet Poten\n")
            else:
                SimResultsFILE.write(" - - \n")
        # Next threeLines

        ##################################################################3
        # THE DATA
        # Do the calculations for each point in time
        # Plus an extra one at time=-1 (with no printing)
        # The t=-1 enables us to calculate a zero-point potential energy
        VerticalCounter = 0
        TickRange = range(-1, numTicks)
        for timeIndex in TickRange:
            if timeIndex >= 0:
                tick = timeValues[timeIndex]
            else:
                tick = timeValues[0]
            ColumnCounter = 0

            # Do the Dynamics on the uResults at this tick
            if timeIndex >= 0:
                self.Dynamics(tick, uResults[timeIndex])

            # Write Time
            if timeIndex >= 0:
                SimResultsFILE.write(str(tick) + " ")

            # Write All the Bodies position, positionDot, positionDotDot
            for bodyIndex in range(1, self.numBodies):
                if timeIndex >= 0:
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
                    SimResultsFILE.write(str(self.NPbody_r[bodyIndex])[1:-1] + " ")
                    # Phi (rad)
                    SimResultsFILE.write(str(self.NPbody_phi[bodyIndex]) + " ")
                    # Phi (deg)
                    SimResultsFILE.write(str(self.NPbody_phi[bodyIndex] * 180.0 / math.pi) + " ")
                    # Xdot Ydot
                    SimResultsFILE.write(str(self.NPbody_drdt[bodyIndex])[1:-1] + " ")
                    # PhiDot (rad)
                    SimResultsFILE.write(str(self.NPbody_dphidt[bodyIndex]) + " ")
                    # PhiDot (deg)
                    SimResultsFILE.write(str(self.NPbody_dphidt[bodyIndex] * 180.0 / math.pi) + " ")
                    # Xdotdot Ydotdot
                    SimResultsFILE.write(str(self.NPbody_d2rdt2[bodyIndex])[1:-1] + " ")
                    # PhiDotDot (rad)
                    SimResultsFILE.write(str(self.NPbody_d2phidt2[bodyIndex]) + " ")
                    # PhiDotDot (deg)
                    SimResultsFILE.write(str(self.NPbody_d2phidt2[bodyIndex] * 180.0 / math.pi) + " ")
                # End of if timeIndex >= 0

                # Write all the points position and positionDot in the body
                for index in range(self.NPnumJointPointsInBody[bodyIndex]):
                    if timeIndex >= 0:
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
                        SimResultsFILE.write(str(self.NPpoint_r[bodyIndex, index])[1:-1] + " ")
                        # Point Xdot Ydot
                        SimResultsFILE.write(str(self.NPpoint_drdt[bodyIndex, index])[1:-1] + " ")
                    # end of if timeIndex >= 0

            # Next bodyIndex

            # Write the Lambdas
            if self.numConstraints > 0:
                if timeIndex >= 0:
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
                        SimResultsFILE.write(str(self.Lambda[bodyIndex*2]) + " " + str(self.Lambda[bodyIndex*2 + 1]) + " ")
                # end of if timeIndex >= 0

            # Compute kinetic and potential energies in micro-Joules
            # REMEMBER: we work in mm-kg-s system
            totKinEnergy = 0
            for bodyIndex in range(1, self.numBodies):
                # Kinetic Energy 1/2 mv^2 + 1/2 I omega^2
                linKinEnergy = 0.5 * (
                             self.NPMassArray[(bodyIndex - 1) * 3] *
                            (self.NPbody_drdt[bodyIndex, 0] ** 2 +
                             self.NPbody_drdt[bodyIndex, 1] ** 2))
                angKinEnergy = 0.5 * (
                            self.NPMassArray[(bodyIndex - 1) * 3 + 2] *
                            self.NPbody_dphidt[bodyIndex] ** 2)

                if timeIndex >= 0:
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
                    SimResultsFILE.write(str(linKinEnergy) + " ")
                    SimResultsFILE.write(str(angKinEnergy) + " ")
                # end of if timeIndex >= 0

                totKinEnergy += linKinEnergy + angKinEnergy
            # Next bodyIndex

            # Currently, calculate only gravitational potential energy
            # Calculate it at t=0 (time index == -1) so that we have a zero reference
            totPotEnergy = 0
            forceIndex = -1
            for forceObj in self.forceList:
                forceIndex += 1
                # Potential Energy
                if forceObj.forceType == "Gravity":
                    for bodyIndex in range(1, self.numBodies):
                        potEnergy = -self.NPWeight[bodyIndex].dot(self.NPbody_r[bodyIndex])
                        if timeIndex == -1:
                            self.NPpotEnergyZeroPoint[bodyIndex] = potEnergy
                        else:
                            potEnergy -= self.NPpotEnergyZeroPoint[bodyIndex]
                            totPotEnergy += potEnergy

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
                # End of if force is gravity
            # next forceObject

            if timeIndex == -1:
                VerticalCounter = 0
            else:
                SimResultsFILE.write(str(totKinEnergy) + " ")
                SimResultsFILE.write(str(totPotEnergy) + " ")
                SimResultsFILE.write("\n")
                VerticalCounter += 1
        # Next timeIndex

        # Print out the joint errors
        SimResultsFILE.write("\n")
        SimResultsFILE.write("Error: \n")
        for twoLines in ["first", "second"]:
            if twoLines == "first":
                SimResultsFILE.write("ABS: ")
            else:
                SimResultsFILE.write("REL: ")
            bodyIndex = -1
            for bodyObj in self.bodyGroup:
                bodyIndex += 1
                if bodyIndex > 0:
                    SimResultsFILE.write("- - - - - - - - - - - - - ")

                    # Write all the points position and positionDot in the body
                    for Ji in range(self.NPnumJointPointsInBody[bodyIndex]):
                        SimResultsFILE.write("- ")
                        # Find the same joint in the other body
                        otherIndex = -1
                        for otherBody in self.bodyGroup:
                            otherIndex += 1
                            if bodyIndex != otherIndex:
                                for otherJoint in range(self.NPnumJointPointsInBody[otherIndex]):
                                    if bodyObj.jointNameList[Ji] == otherBody.jointNameList[otherJoint]:
                                        diff1 = math.fabs(self.NPpoint_r[bodyIndex, Ji][0] - \
                                                self.NPpoint_r[otherIndex, otherJoint][0])
                                        if twoLines == "second":
                                            diff1 /= math.fabs(self.NPpoint_r[bodyIndex, Ji][0] + \
                                                    self.NPpoint_r[otherIndex, otherJoint][0])
                                            diff1 *= 50

                                        diff2 = math.fabs(self.NPpoint_r[bodyIndex, Ji][1] - \
                                               self.NPpoint_r[otherIndex, otherJoint][1])
                                        if twoLines == "second":
                                            diff2 /= math.fabs(self.NPpoint_r[bodyIndex, Ji][1] + \
                                                   self.NPpoint_r[otherIndex, otherJoint][1])
                                            diff2 *= 50

                                        diff3 = math.fabs(self.NPpoint_drdt[bodyIndex, Ji][0] - \
                                               self.NPpoint_drdt[otherIndex, otherJoint][0])
                                        if twoLines == "second":
                                            diff3 /= math.fabs(self.NPpoint_drdt[bodyIndex, Ji][0] + \
                                                   self.NPpoint_drdt[otherIndex, otherJoint][0])
                                            diff3 *= 50

                                        diff4 = math.fabs(self.NPpoint_drdt[bodyIndex, Ji][1] - \
                                               self.NPpoint_drdt[otherIndex, otherJoint][1])
                                        if twoLines == "second":
                                            diff4 /= math.fabs(self.NPpoint_drdt[bodyIndex, Ji][1] + \
                                                   self.NPpoint_drdt[otherIndex, otherJoint][1])
                                            diff4 *= 50

                                        SimResultsFILE.write(str(diff1) + " ")
                                        SimResultsFILE.write(str(diff2) + " ")
                                        SimResultsFILE.write(str(diff3) + " ")
                                        SimResultsFILE.write(str(diff4) + " ")
                                # next otherJoint
                        # next otherBody
            # Next bodyObj
            SimResultsFILE.write("\n")
        # Next twoLines
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
            if self.NPjointObjList[jointNum].bodyHeadName == bodyName:
                self.NPjointObjList[jointNum].Bi = bodyIndex
            if self.NPjointObjList[jointNum].bodyTailName == bodyName:
                self.NPjointObjList[jointNum].Bj = bodyIndex
        # Clean up force Indices in case the body order has been altered
        for forceNum in range(self.numForces):
            if self.NPforceObjList[forceNum].bodyHeadName == bodyName:
                self.NPforceObjList[forceNum].bodyForceHeadIndex = bodyIndex
            if self.NPforceObjList[forceNum].bodyTailName == bodyName:
                self.NPforceObjList[forceNum].bodyForceTailIndex = bodyIndex
                """
    #  -------------------------------------------------------------------------
    """
    def clearZombieBodies(self, bodyObjDict):
        # Clean up any zombie body names
        for jointNum in range(self.numJoints):
            if self.NPjointObjList[jointNum].bodyHeadName not in bodyObjDict:
                self.NPjointObjList[jointNum].bodyHeadName = ""
                self.NPjointObjList[jointNum].bodyHeadLabel = ""
                self.NPjointObjList[jointNum].Bi = 0
            if self.NPjointObjList[jointNum].bodyTailName not in bodyObjDict:
                self.NPjointObjList[jointNum].bodyTailName = ""
                self.NPjointObjList[jointNum].bodyTailLabel = ""
                self.NPjointObjList[jointNum].Bj = 0
        for forceNum in range(self.numForces):
            if self.NPforceObjList[forceNum].bodyHeadName not in bodyObjDict:
                self.NPforceObjList[forceNum].bodyHeadName = ""
                self.NPforceObjList[forceNum].bodyHeadLabel = ""
                self.NPforceObjList[forceNum].bodyForceHeadIndex = 0
            if self.NPforceObjList[forceNum].bodyTailName not in bodyObjDict:
                self.NPforceObjList[forceNum].bodyTailName = ""
                self.NPforceObjList[forceNum].bodyTailLabel = ""
                self.NPforceObjList[forceNum].bodyForceTailIndex = 0
                """
    #  =========================================================================
