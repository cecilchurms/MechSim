import FreeCAD as CAD
import FreeCADGui as CADGui
import SimTools as ST

Debug = False
# =============================================================================
class SimGlobalClass:
    """The Sim analysis simGlobal class"""
    if Debug: ST.Mess("SimGlobalClass-CLASS")
    #  -------------------------------------------------------------------------
    def __init__(self, simGlobalObject):
        """Initialise on entry"""
        if Debug: ST.Mess("SimGlobalClass-__init__")

        simGlobalObject.Proxy = self

        self.addPropertiesToObjects(simGlobalObject)
        self.populateProperties(simGlobalObject)
    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, simGlobalObject):
        if Debug:  ST.Mess("SimGlobalClass-onDocumentRestored")

        self.addPropertiesToObjects(simGlobalObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObjects(self, simGlobalObject):
        """Run by '__init__'  and 'onDocumentRestored' to initialise the Sim system parameters"""
        if Debug: ST.Mess("SimGlobalClass-addPropertiesToObject")

        # Add properties to the simGlobal object
        ST.addObjectProperty(simGlobalObject, "movementPlaneNormal", CAD.Vector(0, 0, 1),            "App::PropertyVector", "", "Defines the movement plane in this MechSim run")
        ST.addObjectProperty(simGlobalObject, "gravityVector",       CAD.Vector(0.0, -9810.0, 0.0),  "App::PropertyVector", "", "Gravitational acceleration Components")
        ST.addObjectProperty(simGlobalObject, "gravityValid",        False,                          "App::PropertyBool",   "", "Flag to verify that the gravity Vector is applicable")
        ST.addObjectProperty(simGlobalObject, "SimResultsValid",     False,    "App::PropertyBool", "", "Flag that the calculation has been performed successfully")

        self.jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0].Group

        jointIndex = -1
        for joint in self.jointGroup:
            # Tag each joint with an index number
            jointIndex += 1
            # Add these two properties to all of the joints in jointGroup
            ST.addObjectProperty(joint, "jointIndex", jointIndex, "App::PropertyInteger", "Bodies and constraints", "Index of the joint")
            ST.addObjectProperty(joint, "SimJoint", "Undefined", "App::PropertyString", "Bodies and constraints", "Type of joint as seen by the simulator")

            # Add these properties to only joints having a valid MechSim joint type
            if hasattr(joint, "JointType") and ST.JOINT_TYPE_DICTIONARY[joint.JointType] < ST.MAXJOINTS:
                # Transfer a copy of the JointType property to the SimJoint property
                setattr(joint, "SimJoint" , joint.JointType)

                ST.addObjectProperty(joint, "bodyHeadIndex", -1, "App::PropertyInteger", "JointPoints", "The index of the body containing the head of the joint")
                ST.addObjectProperty(joint, "pointHeadIndex", -1, "App::PropertyInteger", "JointPoints", "The index of the head point in the body")
                ST.addObjectProperty(joint, "bodyTailIndex", -1, "App::PropertyInteger", "JointPoints", "The index of the body containing the tail of the joint")
                ST.addObjectProperty(joint, "pointTailIndex", -1, "App::PropertyInteger", "JointPoints", "The index of the tail point in the body")

                ST.addObjectProperty(joint, "bodyHeadUnit", CAD.Vector(), "App::PropertyVector", "JointPoints", "The unit vector at the head of the joint")
                ST.addObjectProperty(joint, "bodyTailUnit", CAD.Vector(), "App::PropertyVector", "JointPoints", "The unit vector at the tail of the joint")

                ST.addObjectProperty(joint, "nBodies", -1, "App::PropertyInteger", "Bodies and constraints", "Number of moving bodies involved")
                ST.addObjectProperty(joint, "mConstraints", -1, "App::PropertyInteger", "Bodies and constraints", "Number of rows (constraints)")
                ST.addObjectProperty(joint, "fixDof", False, "App::PropertyBool", "Bodies and constraints", "Fix the Degrees of Freedom")
                ST.addObjectProperty(joint, "FunctType", -1, "App::PropertyInteger", "Function Driver", "Analytical function type")
                ST.addObjectProperty(joint, "rowStart", -1, "App::PropertyInteger", "Bodies and constraints", "Row starting index")
                ST.addObjectProperty(joint, "rowEnd", -1, "App::PropertyInteger", "Bodies and constraints", "Row ending index")

                ST.addObjectProperty(joint, "lengthLink", 1.0, "App::PropertyFloat", "", "Link length")

        # Add properties to all of the linked bodies
        bodyIndex = -1
        for linkedBody in simGlobalObject.Document.Objects:
            if hasattr(linkedBody, "TypeId") and linkedBody.TypeId == 'App::LinkGroup':
                bodyIndex += 1
                ST.addObjectProperty(linkedBody, "bodyIndex", bodyIndex, "App::PropertyInteger", "linkedBody", "Centre of gravity")
                ST.addObjectProperty(linkedBody, "worldCoG", CAD.Vector(), "App::PropertyVector", "linkedBody", "Centre of gravity")
                ST.addObjectProperty(linkedBody, "worldDot", CAD.Vector(), "App::PropertyVector", "X Y Z Phi", "Time derivative of x y z")
                ST.addObjectProperty(linkedBody, "phi", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")
                ST.addObjectProperty(linkedBody, "phiDot", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")

                ST.addObjectProperty(linkedBody, "densitygpcm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in g/cm3")
                ST.addObjectProperty(linkedBody, "densitykgpm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in kg/m3")

                ST.addObjectProperty(linkedBody, "volumemm3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in cm3")
                ST.addObjectProperty(linkedBody, "volumem3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in m3")
                ST.addObjectProperty(linkedBody, "massg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in g")
                ST.addObjectProperty(linkedBody, "masskg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in kg")
                ST.addObjectProperty(linkedBody, "momentOfInertia", 0.1, "App::PropertyFloat", "linkedBody", "Moment of inertia in kg mm^2")

                # Add all the joints to the linkedBody
                ST.addObjectProperty(linkedBody, "JointNameList", [], "App::PropertyStringList", "JointPoints", "The name of the joint object")
                ST.addObjectProperty(linkedBody, "JointTypeList", [], "App::PropertyStringList", "JointPoints", "The type of joint (Rev, Trans etc)")
                ST.addObjectProperty(linkedBody, "JointIndexList", [], "App::PropertyIntegerList", "JointPoints", "The type of joint (Rev, Trans etc)")
                ST.addObjectProperty(linkedBody, "PointRelWorldList", [], "App::PropertyVectorList", "JointPoints", "World vector of the joint point")

                # Tentatively calculate the CoG etc
                ST.updateCoGMoI(linkedBody)
        # Next linkedBody

        # Add properties to the forces

        # Add enough objects for all the forces
        CAD.ActiveDocument.addObject("Part::FeaturePython", "SimForce")

        self.forceList = CAD.ActiveDocument.findObjects(Name="^SimForce$")
        forceIndex = -1
        for forceObject in self.forceList:
            if hasattr(forceObject, "Name") and forceObject.Name == "SimForce":
                forceIndex += 1
                ST.addObjectProperty(forceObject, "forceIndex", forceIndex, "App::PropertyInteger", "", "Index of the forceObject")
                ST.addObjectProperty(forceObject, "forceType", "Gravity", "App::PropertyString", "", "Type of the actuator/force")
                ST.addObjectProperty(forceObject, "newForce", True, "App::PropertyBool", "", "Flag to show if this is a new or old force definition")
                
                ST.addObjectProperty(forceObject, "bodyForceHeadName", "", "App::PropertyString", "Bodies", "Name of the head body")
                ST.addObjectProperty(forceObject, "bodyForceHeadLabel", "", "App::PropertyString", "Bodies", "Label of the head body")
                ST.addObjectProperty(forceObject, "bodyForceHeadIndex", -1, "App::PropertyInteger", "Bodies", "Index of the head body in the NumPy array")
                
                ST.addObjectProperty(forceObject, "pointForceHeadName", "", "App::PropertyString", "Points", "Name of the first point of the force")
                ST.addObjectProperty(forceObject, "pointForceHeadLabel", "", "App::PropertyString", "Points", "Label of the first point of the force")
                ST.addObjectProperty(forceObject, "pointForceHeadIndex", -1, "App::PropertyInteger", "Points", "Index of the first point of the force in the NumPy array")
                
                ST.addObjectProperty(forceObject, "bodyForceTailName", "", "App::PropertyString", "Bodies", "Name of the tail body")
                ST.addObjectProperty(forceObject, "bodyForceTailLabel", "", "App::PropertyString", "Bodies", "Label of the tail body")
                ST.addObjectProperty(forceObject, "bodyForceTailIndex", -1, "App::PropertyInteger", "Bodies", "Index of the tail body in the NumPy array")
                
                ST.addObjectProperty(forceObject, "pointForceTailName", "", "App::PropertyString", "Points", "Name of the second point of the force")
                ST.addObjectProperty(forceObject, "pointForceTailLabel", "", "App::PropertyString", "Points", "Label of the second point of the force")
                ST.addObjectProperty(forceObject, "pointForceTailIndex", 0, "App::PropertyInteger", "Points", "Index of the second point of the force in the NumPy array")
                
                ST.addObjectProperty(forceObject, "Stiffness", 0.0, "App::PropertyFloat", "Values", "Spring Stiffness")
                ST.addObjectProperty(forceObject, "LengthAngle0", 0.0, "App::PropertyFloat", "Values", "Un-deformed Length/Angle")
                ST.addObjectProperty(forceObject, "DampingCoeff", 0.0, "App::PropertyFloat", "Values", "Damping coefficient")
                ST.addObjectProperty(forceObject, "constLocalForce", CAD.Vector(), "App::PropertyVector", "Values", "Constant force in local frame")
                ST.addObjectProperty(forceObject, "constWorldForce", CAD.Vector(), "App::PropertyVector", "Values", "Constant force in x-y frame")
                ST.addObjectProperty(forceObject, "constTorque", 0.0, "App::PropertyFloat", "Values", "Constant torque in x-y frame")
        # Next forceObject
        self.numForces = forceIndex

    #  -------------------------------------------------------------------------
    def populateProperties(self, simGlobalObject):

        # We populate the properties JOINT by JOINT
        jointIndex = -1
        for joint in self.jointGroup:
            jointIndex += 1

            # Mark the applicable SimJoints so we can ignore
            # Fixed joints which are internal to a body
            ST.markSimJoints(simGlobalObject, joint)

            # Now only consider joints for which MechSim has code to handle them
            if hasattr(joint, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[joint.SimJoint] < ST.MAXJOINTS:

                # Fill in the Head first, then the tail
                Head = True

                # We now search through all the bodies for references to this joint
                SimBodyIndex = -1
                for SimBody in simGlobalObject.Document.Objects:
                    if hasattr(SimBody, "TypeId") and SimBody.TypeId == 'App::LinkGroup':
                        SimBodyIndex += 1
                        # Find if this joint is present in this body (linkedGroup)
                        for subBody in SimBody.ElementList:
                            ReferenceNum = -1
                            thisPlacement = CAD.Placement()
                            unitVec = CAD.Vector()
                            if ST.getReferenceName(joint.Reference1) == subBody.Name:
                                unitVec = (ST.getReferencePoints(joint.Reference1, subBody))
                                if unitVec != CAD.Vector():
                                    unitVec.normalize()
                                thisPlacement = joint.Placement1
                                ReferenceNum = 0
                            elif ST.getReferenceName(joint.Reference2) == subBody.Name:
                                unitVec = (ST.getReferencePoints(joint.Reference2, subBody))
                                if unitVec != CAD.Vector():
                                    unitVec.normalize()
                                thisPlacement = joint.Placement2
                                ReferenceNum = 1

                            # Found this joint in this body
                            # So record that in the body
                            if ReferenceNum != -1:
                                # Joint Index
                                t = SimBody.JointIndexList
                                t.append(jointIndex)
                                SimBody.JointIndexList = t

                                # Joint Type
                                t = SimBody.JointTypeList
                                t.append(joint.JointType)
                                SimBody.JointTypeList = t

                                # Joint Name
                                t = SimBody.JointNameList
                                t.append(joint.Name)
                                SimBody.JointNameList = t

                                # Store the values in the joint object
                                if Head:
                                    joint.bodyHeadIndex = SimBodyIndex
                                    joint.pointHeadIndex = len(SimBody.JointIndexList) - 1
                                    joint.bodyHeadUnit = unitVec
                                    Head = False
                                else:
                                    joint.bodyTailIndex = SimBodyIndex
                                    joint.pointTailIndex = len(SimBody.JointIndexList) - 1
                                    joint.bodyTailUnit = unitVec
                                    Head = True

                                # Find the sub-body in the linked body's element list
                                # and apply its placement to the joint placement
                                # to calculate the world placement of the joint point
                                body = CAD.ActiveDocument.findObjects(Name="^" + subBody.Name + "$")[0]
                                t = SimBody.PointRelWorldList
                                t.append((body.Placement * thisPlacement).Base)
                                SimBody.PointRelWorldList = t
                            # end of if ReferenceNum != -1
                        # Next subBody
                # Next SimBody
            # end of if it has SimJoint attribute
        # Next Joint
                        
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug: ST.Mess("SimGlobalClass-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug: ST.Mess("SimGlobalClass-__setstate__")
# =============================================================================
class SimSolverClass:
    if Debug:
        ST.Mess("SimSolverClass-CLASS")
    #  -------------------------------------------------------------------------
    def __init__(self, solverObject):
        """Initialise on instantiation of a new Sim solver object"""
        if Debug:
            ST.Mess("SimSolverClass-__init__")
        solverObject.Proxy = self

        self.addPropertiesToObject(solverObject)

    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, solverObject):
        if Debug:
            ST.Mess("SimSolverClass-onDocumentRestored")
        self.addPropertiesToObject(solverObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObject(self, solverObject):
        """Initialise all the properties of the solver object"""
        if Debug:
            ST.Mess("SimSolverClass-addPropertiesToObject")

        ST.addObjectProperty(solverObject, "FileName",        "",    "App::PropertyString",     "", "FileName to save data under")
        ST.addObjectProperty(solverObject, "Directory",       "",    "App::PropertyString",     "", "Directory to save data")
        ST.addObjectProperty(solverObject, "Accuracy",        3.0,   "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "TimeLength",      10.0,  "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "SolverType",      "RK45",  "App::PropertyString",      "", "Type of solver in LAPACK")
        ST.addObjectProperty(solverObject, "DeltaTime",       0.01,  "App::PropertyFloat",      "", "Length of time steps")
        ST.addObjectProperty(solverObject, "BodyNames",       [],    "App::PropertyStringList", "", "List of Body Names")

    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug:
            ST.Mess("SimSolverClass-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug:
            ST.Mess("SimSolverClass-__setstate__")
# ==============================================================================
#class SimBodyClass:
#    if Debug:
#        ST.Mess("SimBodyClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, bodyObject):
#        """Initialise an instantiation of a new Sim body object"""
#        if Debug:
#            ST.Mess("SimBodyClass-__init__")
#        bodyObject.Proxy = self
#        self.addPropertiesToObject(bodyObject)
#    #  -------------------------------------------------------------------------
#    def onDocumentRestored(self, bodyObject):
#        if Debug:
#            ST.Mess("SimBodyClass-onDocumentRestored")
#        self.addPropertiesToObject(bodyObject)
#    #  -------------------------------------------------------------------------
#    def addPropertiesToObject(self, bodyObject):
#        """Initialise the properties on instantiation of a new body object or on Document Restored
#        All the sub-part bodies [including the main body] are included in the point lists as extra points
#        All points/bodies are local and relative to world placement above"""
#        if Debug:
#            ST.Mess("SimBodyClass-addPropertiesToObject")
#
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("SimBodyClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("SimBodyClass-__setstate__")
## =============================================================================
#class SimMaterialClass:
#    """Defines the Sim material class"""
#    if Debug:
#        CAD.Console.PrintMessage("SimMaterialClass-CLASS\n")
#    #  -------------------------------------------------------------------------
#    def __init__(self, materialObject):
#        """Initialise on instantiation of a new Sim Material object"""
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-__init__\n")
#        materialObject.Proxy = self
#        self.addPropertiesToObject(materialObject)
#    #  -------------------------------------------------------------------------
#    def onDocumentRestored(self, materialObject):
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-onDocumentRestored\n")
#        self.addPropertiesToObject(materialObject)
#    #  -------------------------------------------------------------------------
#    def addPropertiesToObject(self, materialObject):
#        """Called by __init__ and onDocumentRestored functions"""
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-addPropertiesToObject\n")
#
#        ST.addObjectProperty(materialObject, "solidsNameList",       [],   "App::PropertyStringList", "", "List of Solid Part Names")
#        ST.addObjectProperty(materialObject, "materialsNameList",    [],   "App::PropertyStringList", "", "List of matching Material Names")
#        ST.addObjectProperty(materialObject, "materialsDensityList", [],   "App::PropertyFloatList",  "", "List of matching Density values")
#        ST.addObjectProperty(materialObject, "kgm3ORgcm3",           True, "App::PropertyBool",       "", "Density units in the Dialog - kg/m^3 or g/cm^3")
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-__getstate__\n")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-__setstate__\n")
#    # --------------------------------------------------------------------------
#    def __str__(self):
#        return str(self.__dict__)
## ==============================================================================
#class SimJointClass:
#    if Debug:
#        ST.Mess("SimJointClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, jointObject):
#        """Initialise an instantiation of a new Sim Joint object"""
#        if Debug:
#            ST.Mess("SimJointClass-__init__")
#        jointObject.Proxy = self
#        self.addPropertiesToObject(jointObject)
#    #  -------------------------------------------------------------------------
#    def onDocumentRestored(self, jointObject):
#        if Debug:
#            ST.Mess("SimJointClass-onDocumentRestored")
#        self.addPropertiesToObject(jointObject)
#    #  -------------------------------------------------------------------------
#    def addPropertiesToObject(self, jointObject):
#        """Initialise all the properties of the joint object"""
#        if Debug:
#            ST.Mess("SimJointClass-addPropertiesToObject")
#
#        ST.addObjectProperty(jointObject, "JointType", -1, "App::PropertyInteger", "Joint", "Type of Joint")
#        ST.addObjectProperty(jointObject, "JointNumber", 0, "App::PropertyInteger", "Joint", "Number of this joint")
#        ST.addObjectProperty(jointObject, "fixDof", False, "App::PropertyBool", "Joint", "Fix the Degrees of Freedom")
#
#        ST.addObjectProperty(jointObject, "bodyHeadName", "", "App::PropertyString", "Points", "Name of Body at A of Joint")
#        ST.addObjectProperty(jointObject, "bodyHeadLabel", "", "App::PropertyString", "Points", "Label of Body at A of Joint")
#        ST.addObjectProperty(jointObject, "bodyHeadIndex", -1, "App::PropertyInteger", "Points", "Index of the head body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "bodyTailName", "", "App::PropertyString", "Points", "Name of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "bodyTailLabel", "", "App::PropertyString", "Points", "Label of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "bodyTailIndex", -1, "App::PropertyInteger", "Points", "Index of the tail body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointHeadName", "", "App::PropertyString", "Points", "Name of Point at head of joint")
#        ST.addObjectProperty(jointObject, "pointHeadLabel", "", "App::PropertyString", "Points", "Label of Point at head of joint")
#        ST.addObjectProperty(jointObject, "pointHeadIndex", -1, "App::PropertyInteger", "Points", "Index of the head point in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointHeadUnitName", "", "App::PropertyString", "Points", "Name of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointHeadUnitLabel", "", "App::PropertyString", "Points", "Label of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointHeadUnitIndex", -1, "App::PropertyInteger", "Points", "Index of the head point of the 2nd unit vector in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointTailName", "", "App::PropertyString", "Points", "Name of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "pointTailLabel", "", "App::PropertyString", "Points", "Label of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "pointTailIndex", -1, "App::PropertyInteger", "Points", "Index of the tail point in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointTailUnitName", "", "App::PropertyString", "Points", "Name of Point at tail of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointTailUnitLabel", "", "App::PropertyString", "Points", "Label of Point at tail of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointTailUnitIndex", -1, "App::PropertyInteger", "Points", "Index of the tail point of the 2nd unit vector in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "FunctClass", "", "App::PropertyPythonObject", "Driver", "A machine which is set up to generate a driver function")
#        ST.addObjectProperty(jointObject, "FunctType", -1, "App::PropertyInteger", "Driver", "Analytical function type")
#        ST.addObjectProperty(jointObject, "Coeff0", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c0'")
#        ST.addObjectProperty(jointObject, "Coeff1", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c1'")
#        ST.addObjectProperty(jointObject, "Coeff2", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c2'")
#        ST.addObjectProperty(jointObject, "Coeff3", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c3'")
#        ST.addObjectProperty(jointObject, "Coeff4", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c4'")
#        ST.addObjectProperty(jointObject, "Coeff5", 0.0, "App::PropertyFloat", "Driver", "Drive Function coefficient 'c5'")
#
#        ST.addObjectProperty(jointObject, "startTimeDriveFunc", 0.0, "App::PropertyFloat", "Driver", "Drive Function Start time")
#        ST.addObjectProperty(jointObject, "endTimeDriveFunc", 0.0, "App::PropertyFloat", "Driver", "Drive Function End time")
#        ST.addObjectProperty(jointObject, "startValueDriveFunc", 0.0, "App::PropertyFloat", "Driver", "Drive Func value at start")
#        ST.addObjectProperty(jointObject, "endValueDriveFunc", 0.0, "App::PropertyFloat", "Driver", "Drive Func value at end")
#        ST.addObjectProperty(jointObject, "endDerivativeDriveFunc", 0.0, "App::PropertyFloat", "Driver", "Drive Func derivative at end")
#        ST.addObjectProperty(jointObject, "lengthLink", 0.0, "App::PropertyFloat", "Starting Values", "Link length")
#        ST.addObjectProperty(jointObject, "Radius", 0.0, "App::PropertyFloat", "Starting Values", "Disc Radius")
#        ST.addObjectProperty(jointObject, "world0", CAD.Vector(), "App::PropertyVector", "Starting Values",  "Initial condition for disc")
#        ST.addObjectProperty(jointObject, "d0", CAD.Vector(), "App::PropertyVector", "Starting Values", "Initial condition (Fixed)")
#
#        ST.addObjectProperty(jointObject, "nBodies", -1, "App::PropertyInteger", "Bodies & constraints", "Number of moving bodies involved")
#        ST.addObjectProperty(jointObject, "mConstraints", -1, "App::PropertyInteger", "Bodies & constraints", "Number of rows (constraints)")
#        ST.addObjectProperty(jointObject, "rowStart", -1, "App::PropertyInteger", "Bodies & constraints", "Row starting index")
#        ST.addObjectProperty(jointObject, "rowEnd", -1, "App::PropertyInteger", "Bodies & constraints", "Row ending index")
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("SimJointClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("SimJointClass-__setstate__")
"""
ST.addObjectProperty(joint, "pointHeadUnitIndex", -1, "App::PropertyInteger", "Points",
                     "Index of the head point of the 2nd unit vector in the NumPy array")
ST.addObjectProperty(joint, "pointHeadUnitName", "", "App::PropertyString", "Points",
                     "Name of Point at head of 2nd unit vector")

ST.addObjectProperty(joint, "pointTailIndex", -1, "App::PropertyInteger", "Points",
                     "Index of the tail point in the NumPy array")
ST.addObjectProperty(joint, "pointTailName", "", "App::PropertyString", "Points",
                     "Name of Point at tail of joint")

ST.addObjectProperty(joint, "pointTailUnitIndex", -1, "App::PropertyInteger", "Points",
                     "Index of the tail point of the 2nd unit vector in the NumPy array")
ST.addObjectProperty(joint, "pointTailUnitName", "", "App::PropertyString", "Points",
                     "Name of Point at tail of 2nd unit vector")

# Add the properties needed for other types of joint-stuff
if joint.JointType == "Complex":
    ST.addObjectProperty(joint, "lengthLink", 1.0, "App::PropertyFloat", "", "Link length")

    ST.addObjectProperty(joint, "FunctClass", "", "App::PropertyPythonObject", "Function Driver", " A machine which is set up to generate a driver function")
    ST.addObjectProperty(joint, "Coeff0", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c0'")
    ST.addObjectProperty(joint, "Coeff1", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c1'")
    ST.addObjectProperty(joint, "Coeff2", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c2'")
    ST.addObjectProperty(joint, "Coeff3", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c3'")
    ST.addObjectProperty(joint, "Coeff4", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c4'")
    ST.addObjectProperty(joint, "Coeff5", 0, "App::PropertyFloat", "Function Driver", "Drive Func coefficient 'c5'")
    ST.addObjectProperty(joint, "startTimeDriveFunc", 0, "App::PropertyFloat", "Function Driver", "Drive Function Start time")
    ST.addObjectProperty(joint, "endTimeDriveFunc", 0, "App::PropertyFloat", "Function Driver", "Drive Function End time")
    ST.addObjectProperty(joint, "startValueDriveFunc", 0, "App::PropertyFloat", "Function Driver", "Drive Func value at start")
    ST.addObjectProperty(joint, "endValueDriveFunc", 0, "App::PropertyFloat", "Function Driver", "Drive Func value at end")
    ST.addObjectProperty(joint, "endDerivativeDriveFunc", 0, "App::PropertyFloat", "Function Driver", "Drive Func derivative at end")
    ST.addObjectProperty(joint, "Radius", 1.0, "App::PropertyFloat", "Starting Values", "Body Radius")
    ST.addObjectProperty(joint, "world0", CAD.Vector(), "App::PropertyVector", "Starting Values", "initial condition for disc")
    ST.addObjectProperty(joint, "phi0", 0, "App::PropertyFloat", "Starting Values", "initial condition for disc")
    ST.addObjectProperty(joint, "d0", CAD.Vector(), "App::PropertyVector", "Starting Values", "initial condition (Fixed)")
# end of if Complex
"""
