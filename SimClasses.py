import FreeCAD as CAD
import FreeCADGui as CADGui
import SimTools as ST

Debug = False
# =============================================================================
class SimContainerClass:
    """The Sim analysis container class"""
    if Debug:
        ST.Mess("SimContainerClass-CLASS")
    #  -------------------------------------------------------------------------
    def __init__(self, containerObject):
        """Initialise on entry"""
        if Debug:
            ST.Mess("SimContainerClass-__init__")
        containerObject.Proxy = self
        self.addPropertiesToObject(containerObject)
    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, containerObject):
        if Debug:
            ST.Mess("SimContainerClass-onDocumentRestored")
        self.addPropertiesToObject(containerObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObject(self, containerObject):
        """Run by '__init__'  and 'onDocumentRestored' to initialise the empty container members"""
        if Debug:
            ST.Mess("SimContainerClass-addPropertiesToObject")

        # Add the Actual Container object to the document - Typically global properties
        ST.addObjectProperty(containerObject, "movementPlaneNormal", CAD.Vector(0, 0, 1),            "App::PropertyVector", "", "Defines the movement plane in this NikraSim run")
        ST.addObjectProperty(containerObject, "gravityVector",       CAD.Vector(0.0, -9810.0, 0.0),  "App::PropertyVector", "", "Gravitational acceleration Components")
        ST.addObjectProperty(containerObject, "gravityValid",        False,                          "App::PropertyBool",   "", "Flag to verify that the gravity Vector is applicable")
        ST.addObjectProperty(containerObject, "SimResultsValid",     False, "App::PropertyBool",       "", "")

        # We now mark all the joints which are between two bodies
        # [Not part of a compound body] as SimJoints
        # This is to be able to ignore Fixed joints which are part of a body
        for jointGroup in containerObject.Document.Objects:
            # Find the group which contains all the joint objects
            if hasattr(jointGroup, "TypeId") and jointGroup.TypeId == "Assembly::JointGroup":
                for joint in jointGroup.Group:
                    if hasattr(joint, "JointType"):
                        # Add a "SimJoint" property to the joint object
                        # if it is not there yet
                        if "SimJoint" not in joint.PropertiesList:
                            joint.addProperty("App::PropertyString", "SimJoint")

                        # Check if a fixed joint just glues the body together
                        # i.e. is between two parts of the same body
                        if joint.JointType != "Fixed":
                            setattr(joint, "SimJoint", joint.JointType)
                        else:
                            foundInternalJoint = False
                            jointHEADname = ST.getReferenceName(joint.Reference1)
                            jointTAILname = ST.getReferenceName(joint.Reference2)
                            # Run through all the bodies (linked groups)
                            # And find any one which has both parts of the joint in it
                            for linkedGroup in containerObject.Document.Objects:
                                if hasattr(linkedGroup, "TypeId") and linkedGroup.TypeId == 'App::LinkGroup':
                                    foundHEAD = False
                                    foundTAIL = False
                                    for element in linkedGroup.ElementList:
                                        if element.Name == jointHEADname:
                                            foundHEAD = True
                                        if element.Name == jointTAILname:
                                            foundTAIL = True
                                    if foundHEAD and foundTAIL:
                                        foundInternalJoint = True
                            if foundInternalJoint:
                                setattr(joint, "SimJoint", "Internal")
                            else:
                                setattr(joint, "SimJoint", "Fixed")

                        # Add the properties to each joint except internal joints
                        if joint.SimJoint != "Internal":
                            ST.addObjectProperty(joint, "nMovBodies", 2, "App::PropertyInteger", "Bodies and constraints", "Number of moving bodies involved")
                            ST.addObjectProperty(joint, "mConstraints", 2, "App::PropertyInteger", "Bodies and constraints", "Number of rows (constraints)")
                            ST.addObjectProperty(joint, "rowStart", -1, "App::PropertyInteger", "Bodies and constraints", "Row starting index")
                            ST.addObjectProperty(joint, "rowEnd", -1, "App::PropertyInteger", "Bodies and constraints", "Row ending index")

                            # TODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODOTODO
                            # Add the properties needed for other types of joint-stuff
                            if joint.JointType == "Complex":
                                ST.addObjectProperty(joint, "fixDof", False, "App::PropertyBool", "Bodies and constraints", "Fix the Degrees of Freedom")
                                ST.addObjectProperty(joint, "lengthLink", 1.0, "App::PropertyFloat", "", "Link length")

                                ST.addObjectProperty(joint, "FunctClass", "", "App::PropertyPythonObject", "Function Driver", " A machine which is set up to generate a driver function")
                                ST.addObjectProperty(joint, "FunctType", -1, "App::PropertyInteger", "Function Driver", "Analytical function type")
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
                                ST.addObjectProperty(joint, "d0", CAD.Vector(), "App::PropertyVector", "Starting Values", "initial condition (rigid)")
                            # end of if Complex
                        # end of joint is not internal
                # Next Joint

        # Add the other stuff to all of the linked bodies
        for linkedBody in containerObject.Document.Objects:
            if hasattr(linkedBody, "TypeId") and linkedBody.TypeId == 'App::LinkGroup':

                ST.addObjectProperty(linkedBody, "worldCoG", CAD.Vector(), "App::PropertyVector", "linkedBody", "Centre of gravity")
                ST.addObjectProperty(linkedBody, "worldDot", CAD.Vector(), "App::PropertyVector", "X Y Z Phi", "Time derivative of x y z")
                ST.addObjectProperty(linkedBody, "phi", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")
                ST.addObjectProperty(linkedBody, "phiDot", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")

                ST.addObjectProperty(linkedBody, "densitygpcm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in g/cm3")
                ST.addObjectProperty(linkedBody, "densitykgpm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in kg/m3")

                ST.addObjectProperty(linkedBody, "volumecm3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in cm3")
                ST.addObjectProperty(linkedBody, "volumem3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in m3")
                ST.addObjectProperty(linkedBody, "massg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in g")
                ST.addObjectProperty(linkedBody, "masskg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in kg")
                ST.addObjectProperty(linkedBody, "momentOfInertia", 1.0, "App::PropertyFloat", "linkedBody", "Moment of inertia")

                # Add all the joints to the linkedBody
                ST.addObjectProperty(linkedBody, "NumberPoints", 0, "App::PropertyInteger", "JointPoints", "The number of joints in the linkedBody")
                ST.addObjectProperty(linkedBody, "PointNames", [], "App::PropertyStringList", "JointPoints", "The name of the joint object")
                ST.addObjectProperty(linkedBody, "PointTypes", [], "App::PropertyStringList", "JointPoints", "The type of joint (Rev, Trans etc)")
                ST.addObjectProperty(linkedBody, "TailBodies", [], "App::PropertyStringList", "JointPoints", "The name of the respective Tail linkedBody")
                ST.addObjectProperty(linkedBody, "PointLocals", [], "App::PropertyVectorList", "JointPoints", "World vector of the joint point")
                ST.addObjectProperty(linkedBody, "PointWorlds", [], "App::PropertyVectorList", "JointPoints", "World vector of the joint point")

                # Now populate the properties with their respective values
                NumberPoints = 0
                jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0]
                # Run through all the joints in the group
                for joint in jointGroup.Group:
                    # Only add joints which are not "Internal" to the linked body
                    if hasattr(joint, "SimJoint") and joint.SimJoint != "Internal":
                        # Find the occurrence of this joint in this linkedBody
                        for subBody in linkedBody.ElementList:
                            if ST.getReferenceName(joint.Reference1) == subBody.Name or \
                                ST.getReferenceName(joint.Reference2) == subBody.Name:
                                # Found, and with either Reference as the subBody name
                                NumberPoints += 1

                                # Joint Names
                                t = linkedBody.PointNames
                                t.append(joint.Name)
                                linkedBody.PointNames = t

                                # Joint Types
                                t = linkedBody.PointTypes
                                t.append(joint.JointType)
                                linkedBody.PointTypes = t

                                # The joint might be Reference1 or Reference 2
                                if ST.getReferenceName(joint.Reference1) == subBody.Name:
                                    otherReference = joint.Reference2
                                    thisPlacement = joint.Placement1
                                else:
                                    otherReference = joint.Reference1
                                    thisPlacement = joint.Placement2

                                # Tail sub-body names
                                t=linkedBody.TailBodies
                                t.append(ST.getReferenceName(otherReference))
                                linkedBody.TailBodies = t

                                # Find the sub-body in the linked body's element list
                                # and apply its placement to the joint placement
                                # to calculate the world placement of the joint point
                                body = CAD.ActiveDocument.findObjects(Name="^"+subBody.Name+"$")[0]
                                t = linkedBody.PointWorlds
                                t.append((body.Placement * thisPlacement).Base)
                                linkedBody.PointWorlds = t
                                # Make a dummy Joint locals list for later
                                linkedBody.PointLocals = linkedBody.PointWorlds
                        # Next subBody
                # Next joint
                linkedBody.NumberPoints = NumberPoints
        # Next body (linked group)
    #  -------------------------------------------------------------------------
    def __load__(self):
        if Debug:
            ST.Mess("SimContainerClass-__load__")
        return self.Type
    #  -------------------------------------------------------------------------
    def __dump__(self, state):
        if Debug:
            ST.Mess("SimContainerClass-__dump__")
        if state:
            self.Type = state
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
        ST.addObjectProperty(solverObject, "Accuracy",        5.0,   "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "TimeLength",      10.0,  "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "DeltaTime",       0.01,  "App::PropertyFloat",      "", "Length of time steps")

    #  -------------------------------------------------------------------------
    def __load__(self):
        if Debug:
            ST.Mess("SimSolverClass-__load__")
        return self.Type
    #  -------------------------------------------------------------------------
    def __dump__(self, state):
        if Debug:
            ST.Mess("SimSolverClass-__dump__")
        if state:
            self.Type = state
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
#    def __load__(self):
#        if Debug:
#            ST.Mess("SimBodyClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("SimBodyClass-__dump__")
#        if state:
#            self.Type = state
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
#    def __load__(self):
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-__load__\n")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            CAD.Console.PrintMessage("SimMaterialClass-__dump__\n")
#        if state:
#            self.Type = state
#    # --------------------------------------------------------------------------
#    def __str__(self):
#        return str(self.__dict__)
## =============================================================================
#class SimForceClass:
#    if Debug:
#        ST.Mess("SimForceClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, forceObject):
#        """Initialise an instantiation of a new Sim force object"""
#        if Debug:
#            ST.Mess("SimForceClass-__init__")
#        forceObject.Proxy = self
#        self.addPropertiesToObject(forceObject)
#    #  -------------------------------------------------------------------------
#    def onDocumentRestored(self, forceObject):
#        if Debug:
#            ST.Mess("SimForceClass-onDocumentRestored")
#        self.addPropertiesToObject(forceObject)
#    #  -------------------------------------------------------------------------
#    def addPropertiesToObject(self, forceObject):
#        """Initialise all the properties of the force object"""
#        if Debug:
#            ST.Mess("SimForceClass-addPropertiesToObject")
#
#        ST.addObjectProperty(forceObject, "actuatorType",         0,            "App::PropertyInteger", "",           "Type of the actuator/force")
#        ST.addObjectProperty(forceObject, "newForce",             True,         "App::PropertyBool",    "",           "Flag to show if this is a new or old force definition")
#
#        ST.addObjectProperty(forceObject, "body_I_Name",          "",           "App::PropertyString",  "Bodies",     "Name of the head body")
#        ST.addObjectProperty(forceObject, "body_I_Label",         "",           "App::PropertyString",  "Bodies",     "Label of the head body")
#        ST.addObjectProperty(forceObject, "body_I_Index",         0,            "App::PropertyInteger", "Bodies",     "Index of the head body in the NumPy array")
#
#        ST.addObjectProperty(forceObject, "point_i_Name",         "",           "App::PropertyString",  "Points",     "Name of the first point of the force")
#        ST.addObjectProperty(forceObject, "point_i_Label",        "",           "App::PropertyString",  "Points",     "Label of the first point of the force")
#        ST.addObjectProperty(forceObject, "point_i_Index",        0,            "App::PropertyInteger", "Points",     "Index of the first point of the force in the NumPy array")
#
#        ST.addObjectProperty(forceObject, "body_J_Name",          "",           "App::PropertyString",  "Bodies",     "Name of the tail body")
#        ST.addObjectProperty(forceObject, "body_J_Label",         "",           "App::PropertyString",  "Bodies",     "Label of the tail body")
#        ST.addObjectProperty(forceObject, "body_J_Index",         0,            "App::PropertyInteger", "Bodies",     "Index of the tail body in the NumPy array")
#
#        ST.addObjectProperty(forceObject, "point_j_Name",         "",           "App::PropertyString",  "Points",     "Name of the second point of the force")
#        ST.addObjectProperty(forceObject, "point_j_Label",        "",           "App::PropertyString",  "Points",     "Label of the second point of the force")
#        ST.addObjectProperty(forceObject, "point_j_Index",        0,            "App::PropertyInteger", "Points",     "Index of the second point of the force in the NumPy array")
#
#        ST.addObjectProperty(forceObject, "Stiffness",            0.0,          "App::PropertyFloat",   "Values",     "Spring Stiffness")
#        ST.addObjectProperty(forceObject, "LengthAngle0",         0.0,          "App::PropertyFloat",   "Values",     "Un-deformed Length/Angle")
#        ST.addObjectProperty(forceObject, "DampingCoeff",         0.0,          "App::PropertyFloat",   "Values",     "Damping coefficient")
#        ST.addObjectProperty(forceObject, "constLocalForce",      CAD.Vector(), "App::PropertyVector",  "Values",     "Constant force in local frame")
#        ST.addObjectProperty(forceObject, "constWorldForce",      CAD.Vector(), "App::PropertyVector",  "Values",     "Constant force in x-y frame")
#        ST.addObjectProperty(forceObject, "constTorque",          0.0,          "App::PropertyFloat",   "Values",     "Constant torque in x-y frame")
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            ST.Mess("SimForceClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("SimForceClass-__dump__")
#        if state:
#            self.Type = state
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
#        ST.addObjectProperty(jointObject, "body_I_Name", "", "App::PropertyString", "Points", "Name of Body at A of Joint")
#        ST.addObjectProperty(jointObject, "body_I_Label", "", "App::PropertyString", "Points", "Label of Body at A of Joint")
#        ST.addObjectProperty(jointObject, "body_I_Index", -1, "App::PropertyInteger", "Points", "Index of the head body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "body_J_Name", "", "App::PropertyString", "Points", "Name of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "body_J_Label", "", "App::PropertyString", "Points", "Label of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "body_J_Index", -1, "App::PropertyInteger", "Points", "Index of the tail body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "point_I_i_Name", "", "App::PropertyString", "Points", "Name of Point at head of joint")
#        ST.addObjectProperty(jointObject, "point_I_i_Label", "", "App::PropertyString", "Points", "Label of Point at head of joint")
#        ST.addObjectProperty(jointObject, "point_I_i_Index", -1, "App::PropertyInteger", "Points", "Index of the head point in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "point_I_j_Name", "", "App::PropertyString", "Points", "Name of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "point_I_j_Label", "", "App::PropertyString", "Points", "Label of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "point_I_j_Index", -1, "App::PropertyInteger", "Points", "Index of the head point of the 2nd unit vector in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "point_J_i_Name", "", "App::PropertyString", "Points", "Name of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "point_J_i_Label", "", "App::PropertyString", "Points", "Label of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "point_J_i_Index", -1, "App::PropertyInteger", "Points", "Index of the tail point in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "point_J_j_Name", "", "App::PropertyString", "Points", "Name of Point at tail of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "point_J_j_Label", "", "App::PropertyString", "Points", "Label of Point at tail of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "point_J_j_Index", -1, "App::PropertyInteger", "Points", "Index of the tail point of the 2nd unit vector in the NumPy array")
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
#        ST.addObjectProperty(jointObject, "phi0", 0.0, "App::PropertyFloat", "Starting Values", "Initial condition for disc")
#        ST.addObjectProperty(jointObject, "d0", CAD.Vector(), "App::PropertyVector", "Starting Values", "Initial condition (Rigid)")
#
#        ST.addObjectProperty(jointObject, "nMovBodies", -1, "App::PropertyInteger", "Bodies & constraints", "Number of moving bodies involved")
#        ST.addObjectProperty(jointObject, "mConstraints", -1, "App::PropertyInteger", "Bodies & constraints", "Number of rows (constraints)")
#        ST.addObjectProperty(jointObject, "rowStart", -1, "App::PropertyInteger", "Bodies & constraints", "Row starting index")
#        ST.addObjectProperty(jointObject, "rowEnd", -1, "App::PropertyInteger", "Bodies & constraints", "Row ending index")
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            ST.Mess("SimJointClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("SimJointClass-__dump__")
#        if state:
#            self.Type = state
