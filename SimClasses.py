# ********************************************************************************
# *                                                                              *
# *   This program is free software; you can redistribute it and/or modify       *
# *   it under the terms of the GNU Lesser General Public License (LGPL)         *
# *   as published by the Free Software Foundation; either version 3 of          *
# *   the License, or (at your option) any later version.                        *
# *   for detail see the LICENCE text file.                                      *
# *                                                                              *
# *   This program is distributed in the hope that it will be useful,            *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
# *   See the GNU Lesser General Public License for more details.                *
# *                                                                              *
# *   You should have received a copy of the GNU Lesser General Public           *
# *   License along with this program; if not, write to the Free Software        *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston,                      *
# *   MA 02111-1307, USA                                                         *
# *_____________________________________________________________________________ *
# *                                                                              *
# *        ##########################################################            *
# *      #### MechSim - FreeCAD WorkBench - Revision 1.0 (c) 2025: ####          *
# *        ##########################################################            *
# *                                                                              *
# *               This program suite is an expansion of the                      *
# *                  "Nikra-DAP" workbench for FreeCAD                           *
# *                                                                              *
# *                         Software Development:                                *
# *                     Cecil Churms <churms@gmail.com>                          *
# *                                                                              *
# *             It is based on the MATLAB code Complementary to                  *
# *                  Chapters 7 and 8 of the textbook:                           *
# *                                                                              *
# *                     "PLANAR MULTIBODY DYNAMICS                               *
# *         Formulation, Programming with MATLAB, and Applications"              *
# *                          Second Edition                                      *
# *                         by P.E. Nikravesh                                    *
# *                          CRC Press, 2018                                     *
# *                                                                              *
# *     The original project (Nikra-DAP) was the vision of Lukas du Plessis      *
# *                      <lukas.duplessis@uct.ac.za>                             *
# *              who facilitated its development from the start                  *
# *                                                                              *
# *                     With the advent of FreeCAD 1.x,                          *
# *        the Nikra-DAP software was no longer compatible with the new,         *
# *                    built-in, Assembly functionality.                         *
# *               Nikra-DAP was thus radically adapted and enlarged              *
# *                   into the Mechanical Simulator: "MechSim"                   *
# *                                                                              *
# *               The initial stages of this project were funded by:             *
# *                 Engineering X, an international collaboration                *
# *                founded by the Royal Academy of Engineering and               *
# *                        Lloyd's Register Foundation.                          *
# *                                                                              *
# *                 An early version of the software was written by:             *
# *            Alfred Bogaers (EX-MENTE) <alfred.bogaers@ex-mente.co.za>         *
# *                          with contributions from:                            *
# *                 Dewald Hattingh (UP) <u17082006@tuks.co.za>                  *
# *                 Varnu Govender (UP) <govender.v@tuks.co.za>                  *
# *                                                                              *
# *                          Copyright (c) 2025                                  *
# *_____________________________________________________________________________ *
# *                                                                              *
# *             Please refer to the Documentation and README for                 *
# *         more information regarding this WorkBench and its usage              *
# *                                                                              *
# ********************************************************************************
import FreeCAD as CAD
import FreeCADGui as CADGui
import SimTools as ST
from SimTools import MessNoLF

# =============================================================================
class SimGlobalClass:
    """The Sim analysis simGlobal class"""
    #  -------------------------------------------------------------------------
    def __init__(self, simGlobalObject):
        """Initialise on entry"""

        simGlobalObject.Proxy = self

        self.addPropertiesToObjects(simGlobalObject)
        self.populateProperties(simGlobalObject)
    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, simGlobalObject):

        self.addPropertiesToObjects(simGlobalObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObjects(self, simGlobalObject):
        """Run by '__init__'  and 'onDocumentRestored' to initialise the Sim system parameters"""

        # Add properties to the simGlobal object
        ST.addObjectProperty(simGlobalObject, "movementPlaneNormal", CAD.Vector(0, 0, 1),            "App::PropertyVector", "", "Defines the movement plane in this MechSim run")
        ST.addObjectProperty(simGlobalObject, "gravityVector",       CAD.Vector(0.0, -9810.0, 0.0),  "App::PropertyVector", "", "Gravitational acceleration Components")
        ST.addObjectProperty(simGlobalObject, "gravityValid",        False,                          "App::PropertyBool",   "", "Flag to verify that the gravity Vector is applicable")
        ST.addObjectProperty(simGlobalObject, "SimResultsValid",     False,    "App::PropertyBool", "", "Flag that the calculation has been performed successfully")

        self.jointGroup = CAD.ActiveDocument.findObjects(Name="^Joints$")[0].Group

        Ji = -1
        for joint in self.jointGroup:
            # Tag each joint with an index number
            Ji += 1
            # Add these two properties to all the joints in jointGroup
            ST.addObjectProperty(joint, "Ji", Ji, "App::PropertyInteger", "Bodies and constraints", "Index of the joint")
            ST.addObjectProperty(joint, "SimJoint", "Undefined", "App::PropertyString", "Bodies and constraints", "Type of joint as seen by the simulator")

            # Add these properties to only joints having a valid MechSim joint type
            if hasattr(joint, "JointType") and ST.JOINT_TYPE_DICTIONARY[joint.JointType] < ST.MAXJOINTS:
                # Transfer a copy of the JointType property to the SimJoint property
                setattr(joint, "SimJoint" , joint.JointType)

                ST.addObjectProperty(joint, "Bi", -1, "App::PropertyInteger", "JointPoints", "The index of the body containing the head of the joint")
                ST.addObjectProperty(joint, "Pi", -1, "App::PropertyInteger", "JointPoints", "The index of the head point in the body")
                ST.addObjectProperty(joint, "Bj", -1, "App::PropertyInteger", "JointPoints", "The index of the body containing the tail of the joint")
                ST.addObjectProperty(joint, "Pj", -1, "App::PropertyInteger", "JointPoints", "The index of the tail point in the body")

                ST.addObjectProperty(joint, "bodyHeadUnit", CAD.Vector(), "App::PropertyVector", "JointPoints", "The unit vector at the head of the joint")
                ST.addObjectProperty(joint, "bodyTailUnit", CAD.Vector(), "App::PropertyVector", "JointPoints", "The unit vector at the tail of the joint")
                ST.addObjectProperty(joint, "headInPlane", False, "App::PropertyBool", "JointPoints", "If the head unit vector lies in the x-y plane")
                ST.addObjectProperty(joint, "tailInPlane", False, "App::PropertyBool", "JointPoints", "If the tail unit vector lies in the x-y plane")

                ST.addObjectProperty(joint, "nBodies", -1, "App::PropertyInteger", "Bodies and constraints", "Number of moving bodies involved")
                ST.addObjectProperty(joint, "mConstraints", -1, "App::PropertyInteger", "Bodies and constraints", "Number of rows (constraints)")
                ST.addObjectProperty(joint, "fixDof", False, "App::PropertyBool", "Bodies and constraints", "Fix the Degrees of Freedom")
                ST.addObjectProperty(joint, "FunctType", -1, "App::PropertyInteger", "Function Driver", "Analytical function type")
                ST.addObjectProperty(joint, "rowStart", -1, "App::PropertyInteger", "Bodies and constraints", "Row starting index")
                ST.addObjectProperty(joint, "rowEnd", -1, "App::PropertyInteger", "Bodies and constraints", "Row ending index")

                ST.addObjectProperty(joint, "lengthLink", 1.0, "App::PropertyFloat", "", "Link length")
                ST.addObjectProperty(joint, "phi0", 0.0, "App::PropertyFloat", "", "Original rotational angle")
                ST.addObjectProperty(joint, "x0", 0.0, "App::PropertyFloat", "", "Original x position")

        # Add properties to all the linked bodies
        bodyIndex = -1
        for bodyObj in simGlobalObject.Document.Objects:
            if hasattr(bodyObj, "TypeId") and bodyObj.TypeId == 'App::LinkGroup':
                bodyIndex += 1
                ST.addObjectProperty(bodyObj, "bodyIndex", bodyIndex, "App::PropertyInteger", "linkedBody", "Centre of gravity")
                ST.addObjectProperty(bodyObj, "worldCoG", CAD.Vector(), "App::PropertyVector", "linkedBody", "Centre of gravity")
                ST.addObjectProperty(bodyObj, "worldDot", CAD.Vector(), "App::PropertyVector", "X Y Z Phi", "Time derivative of x y z")
                ST.addObjectProperty(bodyObj, "phi", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")
                ST.addObjectProperty(bodyObj, "phiDot", 0.0, "App::PropertyFloat", "X Y Z Phi", "Angular velocity of phi")

                ST.addObjectProperty(bodyObj, "densitygpcm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in g/cm3")
                ST.addObjectProperty(bodyObj, "densitykgpm3", 1.0, "App::PropertyFloat", "linkedBody", "Density in kg/m3")

                ST.addObjectProperty(bodyObj, "volumemm3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in cm3")
                ST.addObjectProperty(bodyObj, "volumem3", 1.0, "App::PropertyFloat", "linkedBody", "Volume in m3")
                ST.addObjectProperty(bodyObj, "massg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in g")
                ST.addObjectProperty(bodyObj, "masskg", 1.0, "App::PropertyFloat", "linkedBody", "Mass in kg")
                ST.addObjectProperty(bodyObj, "momentOfInertia", 0.1, "App::PropertyFloat", "linkedBody", "Moment of inertia in kg mm^2")

                # Add all the joints to the linkedBody
                ST.addObjectProperty(bodyObj, "jointNameList", [], "App::PropertyStringList", "JointPoints", "The name of the joint object")
                ST.addObjectProperty(bodyObj, "jointTypeList", [], "App::PropertyStringList", "JointPoints", "The type of joint (Rev, Trans etc)")
                ST.addObjectProperty(bodyObj, "jointIndexList", [], "App::PropertyIntegerList", "JointPoints", "The type of joint (Rev, Trans etc)")
                ST.addObjectProperty(bodyObj, "PointPlacementList", [], "App::PropertyPlacementList", "JointPoints", "Placement of the joint point")

                # Tentatively calculate the CoG etc
                ST.updateCoGMoI(bodyObj)
        # Next bodyObj

        # Add properties to the forces

        # Add enough objects for all the forces
        CAD.ActiveDocument.addObject("Part::FeaturePython", "SimForce")

        self.forceList = CAD.ActiveDocument.findObjects(Name="^SimForce$")
        forceIndex = -1
        for forceObj in self.forceList:
            if hasattr(forceObj, "Name") and forceObj.Name == "SimForce":
                forceIndex += 1
                ST.addObjectProperty(forceObj, "forceIndex", forceIndex, "App::PropertyInteger", "", "Index of the forceObject")
                ST.addObjectProperty(forceObj, "forceType", "Gravity", "App::PropertyString", "", "Type of the actuator/force")
                ST.addObjectProperty(forceObj, "newForce", True, "App::PropertyBool", "", "Flag to show if this is a new or old force definition")
                
                ST.addObjectProperty(forceObj, "bodyForceHeadName", "", "App::PropertyString", "Bodies", "Name of the head body")
                ST.addObjectProperty(forceObj, "bodyForceHeadLabel", "", "App::PropertyString", "Bodies", "Label of the head body")
                ST.addObjectProperty(forceObj, "bodyForceHeadIndex", -1, "App::PropertyInteger", "Bodies", "Index of the head body in the NumPy array")
                
                ST.addObjectProperty(forceObj, "pointForceHeadName", "", "App::PropertyString", "Points", "Name of the first point of the force")
                ST.addObjectProperty(forceObj, "pointForceHeadLabel", "", "App::PropertyString", "Points", "Label of the first point of the force")
                ST.addObjectProperty(forceObj, "pointForceHeadIndex", -1, "App::PropertyInteger", "Points", "Index of the first point of the force in the NumPy array")
                
                ST.addObjectProperty(forceObj, "bodyForceTailName", "", "App::PropertyString", "Bodies", "Name of the tail body")
                ST.addObjectProperty(forceObj, "bodyForceTailLabel", "", "App::PropertyString", "Bodies", "Label of the tail body")
                ST.addObjectProperty(forceObj, "bodyForceTailIndex", -1, "App::PropertyInteger", "Bodies", "Index of the tail body in the NumPy array")
                
                ST.addObjectProperty(forceObj, "pointForceTailName", "", "App::PropertyString", "Points", "Name of the second point of the force")
                ST.addObjectProperty(forceObj, "pointForceTailLabel", "", "App::PropertyString", "Points", "Label of the second point of the force")
                ST.addObjectProperty(forceObj, "pointForceTailIndex", 0, "App::PropertyInteger", "Points", "Index of the second point of the force in the NumPy array")
                
                ST.addObjectProperty(forceObj, "Stiffness", 0.0, "App::PropertyFloat", "Values", "Spring Stiffness")
                ST.addObjectProperty(forceObj, "LengthAngle0", 0.0, "App::PropertyFloat", "Values", "Un-deformed Length/Angle")
                ST.addObjectProperty(forceObj, "DampingCoeff", 0.0, "App::PropertyFloat", "Values", "Damping coefficient")
                ST.addObjectProperty(forceObj, "constLocalForce", CAD.Vector(), "App::PropertyVector", "Values", "Constant force in local frame")
                ST.addObjectProperty(forceObj, "constWorldForce", CAD.Vector(), "App::PropertyVector", "Values", "Constant force in x-y frame")
                ST.addObjectProperty(forceObj, "constTorque", 0.0, "App::PropertyFloat", "Values", "Constant torque in x-y frame")
        # Next forceObj
        self.numForces = forceIndex

    #  -------------------------------------------------------------------------
    def populateProperties(self, simGlobalObject):

        for SimBody in simGlobalObject.Document.Objects:
            if hasattr(SimBody, "TypeId") and SimBody.TypeId == 'App::LinkGroup':
                # Initialise parameters
                SimBody.jointIndexList = []
                SimBody.jointTypeList = []
                SimBody.jointNameList = []
                SimBody.PointPlacementList = []

        # We populate the properties JOINT by JOINT
        Ji = -1
        for joint in self.jointGroup:
            """if hasattr(joint, "Placement1"):
                ST.MessNoLF(joint.Name)
                ST.MessNoLF(" - ")
                ST.Mess(joint.Placement1)
            if hasattr(joint, "Placement2"):
                ST.MessNoLF(joint.Name)
                ST.MessNoLF(" - ")
                ST.Mess(joint.Placement2)
                """

            Ji += 1

            # Mark the applicable SimJoints with new names
            # e.g. Fixed joints which are internal to a body
            ST.markMechSimJoints(simGlobalObject, joint)

            # Now only consider joints for which MechSim has code to handle them
            if hasattr(joint, "SimJoint") and ST.JOINT_TYPE_DICTIONARY[joint.SimJoint] < ST.MAXJOINTS:
                # Initialise parameters
                joint.bodyHeadUnit = CAD.Vector()
                joint.bodyTailUnit = CAD.Vector()


                # We will fill in the Head first, then the tail
                Head = True

                # We now search through all the bodies for references to this joint
                SimBodyIndex = -1
                for SimBody in simGlobalObject.Document.Objects:
                    if hasattr(SimBody, "TypeId") and SimBody.TypeId == 'App::LinkGroup':
                        SimBodyIndex += 1
                        # Find if this joint is present inside a sub-body
                        # in the element list of this body (linkedGroup)
                        for subBody in SimBody.ElementList:
                            foundThisJoint = False
                            # Start out with null placement and unit vector
                            thisPlacement = CAD.Placement()
                            if ST.getReferenceName(joint.Reference1) == subBody.Name:
                                thisPlacement = joint.Placement1
                                foundThisJoint = True
                            elif ST.getReferenceName(joint.Reference2) == subBody.Name:
                                thisPlacement = joint.Placement2
                                foundThisJoint = True
                            #if unitVec1 != CAD.Vector() and unitVec2 != CAD.Vector():
                                #ST.Mess(str(unitVec1.dot(unitVec2)))

                            # Found this joint in this body
                            # So record that in the body's joint lists
                            if foundThisJoint:
                                # add the Joint Index
                                t = SimBody.jointIndexList
                                t.append(Ji)
                                SimBody.jointIndexList = t

                                # add the Joint Type
                                t = SimBody.jointTypeList
                                t.append(joint.SimJoint)
                                SimBody.jointTypeList = t

                                # add the Joint Name
                                t = SimBody.jointNameList
                                t.append(joint.Name)
                                SimBody.jointNameList = t

                                # and apply its placement to the joint placement
                                # to calculate the world placement of the joint point
                                body = CAD.ActiveDocument.findObjects(Name="^" + subBody.Name + "$")[0]

                                placed = body.Placement * thisPlacement
                                t = SimBody.PointPlacementList
                                t.append(placed)
                                SimBody.PointPlacementList = t

                                unitVec = placed.Rotation.multVec(CAD.Vector(0.0, 0.0, 1.0)).normalize()

                                # Store the values of the joint and point object into the body
                                # currently the last item in the jointIndexList
                                if Head:
                                    joint.Bi = SimBodyIndex
                                    joint.Pi = len(SimBody.jointIndexList) - 1
                                    joint.bodyHeadUnit = unitVec
                                    joint.headInPlane = abs(unitVec.z) < 0.1
                                    Head = False
                                else:
                                    joint.Bj = SimBodyIndex
                                    joint.Pj = len(SimBody.jointIndexList) - 1
                                    joint.bodyTailUnit = unitVec
                                    joint.tailInPlane = abs(unitVec.z) < 0.1
                                    if joint.SimJoint == "Revolute-Revolute":
                                        if joint.headInPlane or joint.tailInPlane:
                                            joint.SimJoint = "Translation-Revolute"
                            # end of if ReferenceNum != -1
                        # Next subBody
                # Next SimBody
            """
            else:
                for SimBody in simGlobalObject.Document.Objects:
                    if hasattr(SimBody, "ElementList", ):
                        for subBody in SimBody.ElementList:
                            if hasattr(joint, "Reference1"):
                                if ST.getReferenceName(joint.Reference1) == subBody.Name:
                                    v = ST.getFaceVector(joint.Reference1, subBody)
                                    if v != CAD.Vector():
                                        MessNoLF(subBody.Name)
                                        MessNoLF(" Face1: ")
                                        ST.PrintNp1D(True, v)
                                if ST.getReferenceName(joint.Reference2) == subBody.Name:
                                    v = ST.getFaceVector(joint.Reference2, subBody)
                                    if v != CAD.Vector():
                                        MessNoLF(subBody.Name)
                                        MessNoLF(" Face2: ")
                                        ST.PrintNp1D(True, v)
                                if ST.getReferenceName(joint.Reference1) == subBody.Name:
                                    v = ST.getEdgeVector(joint.Reference1, subBody)
                                    if v != CAD.Vector():
                                        MessNoLF(subBody.Name)
                                        MessNoLF(" Edge1: ")
                                        ST.PrintNp1D(True, v)
                                if ST.getReferenceName(joint.Reference2) == subBody.Name:
                                    v = ST.getEdgeVector(joint.Reference2, subBody)
                                    if v != CAD.Vector():
                                        MessNoLF(subBody.Name)
                                        MessNoLF(" Edge2: ")
                                        ST.PrintNp1D(True, v)
                                    """
            # end of if it has SimJoint attribute
        # Next Joint
                        
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
    # =============================================================================
class SimSolverClass:
    #  -------------------------------------------------------------------------
    def __init__(self, solverObject):
        """Initialise on instantiation of a new Sim solver object"""
        solverObject.Proxy = self

        self.addPropertiesToObject(solverObject)
    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, solverObject):
        self.addPropertiesToObject(solverObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObject(self, solverObject):
        """Initialise all the properties of the solver object"""

        ST.addObjectProperty(solverObject, "FileName",        "",    "App::PropertyString",     "", "FileName to save data under")
        ST.addObjectProperty(solverObject, "Directory",       "",    "App::PropertyString",     "", "Directory to save data")
        ST.addObjectProperty(solverObject, "Accuracy",        3.0,   "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "TimeLength",      10.0,  "App::PropertyFloat",      "", "Length of the Analysis")
        ST.addObjectProperty(solverObject, "SolverType",      "RK45",  "App::PropertyString",      "", "Type of solver in LAPACK")
        ST.addObjectProperty(solverObject, "DeltaTime",       0.01,  "App::PropertyFloat",      "", "Length of time steps")
        ST.addObjectProperty(solverObject, "BodyNames",       [],    "App::PropertyStringList", "", "List of Body Names")

    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
# =============================================================================
class SimMaterialClass:
    """Defines the Sim material class"""
    #  -------------------------------------------------------------------------
    def __init__(self, materialObject):
        """Initialise on instantiation of a new Sim Material object"""

        materialObject.Proxy = self
        self.addPropertiesToObject(materialObject)
    #  -------------------------------------------------------------------------
    def onDocumentRestored(self, materialObject):
        self.addPropertiesToObject(materialObject)
    #  -------------------------------------------------------------------------
    def addPropertiesToObject(self, materialObject):
        """Called by __init__ and onDocumentRestored functions"""

        ST.addObjectProperty(materialObject, "solidsNameList",       [],   "App::PropertyStringList", "", "List of Solid Part Names")
        ST.addObjectProperty(materialObject, "materialsNameList",    [],   "App::PropertyStringList", "", "List of matching Material Names")
        ST.addObjectProperty(materialObject, "materialsDensityList", [],   "App::PropertyFloatList",  "", "List of matching Density values")
        ST.addObjectProperty(materialObject, "kgm3ORgcm3",           True, "App::PropertyBool",       "", "Density units in the Dialog - kg/m^3 or g/cm^3")
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
    # --------------------------------------------------------------------------
    def __str__(self):
        return str(self.__dict__)
# ==============================================================================
#class SimBodyClass:
#    #  -------------------------------------------------------------------------
#    def __init__(self, bodyObject):
#        """Initialise an instantiation of a new Sim body object"""
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
#        ST.addObjectProperty(jointObject, "Bi", -1, "App::PropertyInteger", "Points", "Index of the head body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "bodyTailName", "", "App::PropertyString", "Points", "Name of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "bodyTailLabel", "", "App::PropertyString", "Points", "Label of Body at B of Joint")
#        ST.addObjectProperty(jointObject, "Bj", -1, "App::PropertyInteger", "Points", "Index of the tail body in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointHeadName", "", "App::PropertyString", "Points", "Name of Point at head of joint")
#        ST.addObjectProperty(jointObject, "pointHeadLabel", "", "App::PropertyString", "Points", "Label of Point at head of joint")
#        ST.addObjectProperty(jointObject, "Pi", -1, "App::PropertyInteger", "Points", "Index of the head point in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointHeadUnitName", "", "App::PropertyString", "Points", "Name of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointHeadUnitLabel", "", "App::PropertyString", "Points", "Label of Point at head of 2nd unit vector")
#        ST.addObjectProperty(jointObject, "pointHeadUnitIndex", -1, "App::PropertyInteger", "Points", "Index of the head point of the 2nd unit vector in the NumPy array")
#
#        ST.addObjectProperty(jointObject, "pointTailName", "", "App::PropertyString", "Points", "Name of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "pointTailLabel", "", "App::PropertyString", "Points", "Label of Point at tail of joint")
#        ST.addObjectProperty(jointObject, "Pj", -1, "App::PropertyInteger", "Points", "Index of the tail point in the NumPy array")
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

ST.addObjectProperty(joint, "Pj", -1, "App::PropertyInteger", "Points",
                     "Index of the tail point in the NumPy array")
ST.addObjectProperty(joint, "pointTailName", "", "App::PropertyString", "Points",
                     "Name of Point at tail of joint")

ST.addObjectProperty(joint, "pointTailUnitIndex", -1, "App::PropertyInteger", "Points",
                     "Index of the tail point of the 2nd unit vector in the NumPy array")
ST.addObjectProperty(joint, "pointTailUnitName", "", "App::PropertyString", "Points",
                     "Name of Point at tail of 2nd unit vector")

# Add the properties needed for other types of joint-stuff
if joint.JointType == "Complex":
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
