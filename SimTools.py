import FreeCAD as CAD

from os import path
from PySide import QtGui, QtCore
import math
import numpy as np

Debug = False
#  -------------------------------------------------------------------------
# These are the string constants used in various places throughout the code
# These options are included in the code,
# but limited until each has been more thoroughly tested
MAXJOINTS = 2
JOINT_TYPE_DICTIONARY = {
                        "Revolute": 0,
                        "Fixed": 1,
                        "Translation": 2,
                        "Revolute-Revolute": 3,
                        "Translation-Revolute": 4,
                        "Disc": 5,
                        "GroundedJoint": 6,
                        "Internal": 7,
                        "Cylindrical": 8,
                        "Slider": 9,
                        "Ball": 10,
                        "Distance": 11,
                        "Parallel": 12,
                        "Perpendicular": 13,
                        "Angle": 14,
                        "RackPinion": 15,
                        "Screw": 16,
                        "Gears": 17,
                        "Belt": 18,
                        "Undefined": 19,
                        "Driven-Translation": 20,
                        }

# These options are included in the code,
# but limited until each has been more thoroughly tested
MAXFORCES = 1
FORCE_TYPE_DICTIONARY = {"Gravity": 0,
                         "Spring": 1,
                         "Rotational Spring": 2,
                         "Linear Spring Damper": 3,
                         "Rotational Spring Damper": 4,
                         "Unilateral Spring Damper": 5,
                         "Constant Force Local to Body": 6,
                         "Constant Global Force": 7,
                         "Constant Torque about a Point": 8,
                         "Contact Friction": 9,
                         "Motor": 10,
                         "Motor with Air Friction": 11,
                         }
FORCE_TYPE_HELPER_TEXT = [
    "Universal force of attraction between all matter",
    "A device that stores energy when compressed or extended and exerts a force in the opposite direction",
    "A device that stores energy when twisted and exerts a torque in the opposite direction",
    "A device used to limit or retard linear vibration ",
    "Device used to limit movement and vibration caused by rotation",
    "A Device used to dampen vibration in only the one direction",
    "A constant force with direction relative to the Body coordinates",
    "A constant force in a specific global direction",
    "A constant torque about a point on a body",
    "Contact friction between two bodies",
    "A motor with characteristics defined by an equation",
    "A motor defined by an equation, but with air friction associated with body movement"]
#  -------------------------------------------------------------------------
def getsimGlobalObject():
    """Return the simGlobal object"""
    if Debug: Mess("SimTools-getsimGlobalObject")

    for simGlobal in CAD.ActiveDocument.Objects:
        if hasattr(simGlobal, "Name") and simGlobal.Name == "SimGlobal":
            return simGlobal
    return None
#  -------------------------------------------------------------------------
def updateCoGMoI(bodyObj):
    """ Computes:
    1. The world centre of mass of each body based on the weighted sum
    of each solid's centre of mass
    2. The moment of inertia of the whole body, based on the moment of Inertia for the
    solid through its CoG axis + solid mass * (perpendicular distance
    between the axis through the solid's CoG and the axis through the whole body's
    CoG) squared.   Both axes should be normal to the plane of movement and
    will hence be parallel if everything is OK.
    *************************************************************************
    IMPORTANT:  FreeCAD and MechSim work internally with a mm-kg-s system
    *************************************************************************
    """
    if Debug: Mess("SimTools-updateCoGMoI")

    # Get the Material object (i.e. list of densities) which has been defined in the appropriate Sim routine
    # ToDo theMaterialObject = getMaterialObject()

    # Determine the vectors to convert movement in the selected base plane to the X-Y plane
    xyzToXYRotation = CAD.Rotation(CAD.Vector(0, 0, 1), getsimGlobalObject().movementPlaneNormal)

    # Clear the variables for filling
    totalBodyMass = 0
    totalBodyVolume = 0
    CoGWholeBody = CAD.Vector()
    massList = []
    subBodyMoIThroughCoGNormalToMovementPlaneList = []
    subBodyCentreOfGravityMovementPlaneList = []

    # Run through all the subBodies in the assemblyObjectList
    for element in bodyObj.ElementList:

        # Volume of this App::Link Object in cubic cm
        # element.Shape.Volume returns value in cubic mm
        volumemm3 = element.Shape.Volume
        addObjectProperty(element, "volumemm3", volumemm3, "App::PropertyFloat", "Sub-Body",
                             "Volume of sub-body in mm^3")
        totalBodyVolume += volumemm3
        CoG = element.Shape.CenterOfGravity
        addObjectProperty(element, "CoG", CoG, "App::PropertyVector", "Sub-Body",
                             "CoG vector of sub-body")

        # TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
        #index = theMaterialObject.subBodysNameList.index(assemblyPartName)
        #density = theMaterialObject.materialsDensityList[index] * 1e-9
        # Density in kg/mm3
        density = 1.0e-6
        # Calculate the mass in kg
        # density kg/mm3 - Volume mm3 - mass kg
        masskg = density * volumemm3
        totalBodyMass += masskg
        massList.append(masskg)

        # Add the Centre of gravities to a list to use in parallel axis theorem
        subBodyCentreOfGravityMovementPlaneList.append(xyzToXYRotation.toMatrix().multVec(CoG))
        # Squash any component of CoG in the z direction, onto the X-Y plane
        # Remember: [-1] is the last element of the list !!!
        subBodyCentreOfGravityMovementPlaneList[-1].z = 0.0

        # MatrixOfInertia[MoI] around an axis through the CoG of the element
        # and normal to the MovementPlaneNormal
        # ToDo fix up the non-z plane normal
        MoIVec = element.Shape.MatrixOfInertia.multVec(getsimGlobalObject().movementPlaneNormal)
        addObjectProperty(element, "MoI", MoIVec.z * density, "App::PropertyFloat", "Sub-Body",
                             "MoI of sub-body in kg mm^2")
        # MoI calculated in [kg*mm^2]
        subBodyMoIThroughCoGNormalToMovementPlaneList.append(MoIVec.z * density)

        CoGWholeBody += masskg * subBodyCentreOfGravityMovementPlaneList[-1]
    # Next element

    CoGWholeBody /= totalBodyMass
    setattr(bodyObj, "worldCoG", CoGWholeBody)

    bodyCentreOfGravityMovementPlane = xyzToXYRotation.toMatrix().multVec(CoGWholeBody)
    bodyCentreOfGravityMovementPlane.z = 0.0

    setattr(bodyObj, "masskg", totalBodyMass)
    setattr(bodyObj, "volumem3", totalBodyVolume * 1.0e-9)
    setattr(bodyObj, "densitykgpm3", density * 1.0e9)

    setattr(bodyObj, "massg", totalBodyMass * 1000.0)
    setattr(bodyObj, "volumemm3", totalBodyVolume)
    setattr(bodyObj, "densitygpcm3", density * 1.0e6)

    # Using parallel axis theorem to compute the moment of inertia through the CoG
    # of the full body comprised of multiple shapes
    momentInertiaWholeBody = 0
    for MoIIndex in range(len(subBodyMoIThroughCoGNormalToMovementPlaneList)):
        if Debug:
            Mess("Sub-Body MoI: "+str(subBodyMoIThroughCoGNormalToMovementPlaneList[MoIIndex]))
        distanceBetweenAxes = (bodyCentreOfGravityMovementPlane - subBodyCentreOfGravityMovementPlaneList[MoIIndex]).Length
        momentInertiaWholeBody += subBodyMoIThroughCoGNormalToMovementPlaneList[MoIIndex] + massList[MoIIndex] * (distanceBetweenAxes ** 2)
    setattr(bodyObj, "momentOfInertia", momentInertiaWholeBody)

    if Debug:
        Mess("Body Total Mass [kg]:  "+str(totalBodyMass))
        MessNoLF("Body Centre of Gravity [mm]:  ")
        PrintVec(CoGWholeBody)
        Mess("Body moment of inertia [kg mm^2):  "+str(momentInertiaWholeBody))
        Mess("")

#  -------------------------------------------------------------------------
def markSimJoints(simGlobalObject, joint):
    # Check if it is the joint from body to ground
    if hasattr(joint, "ObjectToGround") and hasattr(joint,"SimJoint"):
        setattr(joint, "SimJoint", "GroundedJoint")
        return

    # Alter only the fixed joints if necessary
    elif hasattr(joint, "JointType"):
        # Check if a fixed joint just glues the body together
        # i.e. BOTH joints are inside two sub-bodies of the SAME body
        if joint.JointType != "Fixed":
            setattr(joint, "SimJoint", joint.JointType)
        else:
            # This is what we do to the fixed joints
            foundInternalJoint = False
            jointHEADname = getReferenceName(joint.Reference1)
            jointTAILname = getReferenceName(joint.Reference2)
            # Run through all the bodies (linked groups)
            # And find any one which has both parts of the joint in it
            for SimBody in simGlobalObject.Document.Objects:
                if hasattr(SimBody, "TypeId") and SimBody.TypeId == 'App::LinkGroup':
                    foundHEAD = False
                    foundTAIL = False
                    for element in SimBody.ElementList:
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

#  -------------------------------------------------------------------------
def getReferenceName(ReferenceTuple):
    if Debug: Mess("SimTools-getReferenceName")

    name = ReferenceTuple[1][0]
    period = name.find('.')
    if period == -1:
        return name
    else:
        return name[:period]
#  -------------------------------------------------------------------------
def addObjectProperty(newobject, newproperty, initVal, newtype, *args):
    """Call addObjectProperty on the object if it does not yet exist"""
    #if Debug: Mess("SimTools-addObjectProperty")

    # Only add it if the property does not exist there already
    added = False
    if newproperty not in newobject.PropertiesList:
        added = newobject.addProperty(newtype, newproperty, *args)
    if added:
        setattr(newobject, newproperty, initVal)
        return True
    else:
        return False
#  -------------------------------------------------------------------------
def getSimModulePath(iconDir, iconName):
    """Returns the path where the current Sim module is stored
    Determines where this file is running from, so Sim workbench works regardless of whether
    the module is installed in the app's module directory or the user's app data folder.
    (The second overrides the first.)"""
    if Debug: Mess("SimTools-getSimModulePath")

    return path.join(path.dirname(__file__), iconDir, iconName)
#  -------------------------------------------------------------------------
def CalculateRotationMatrix(phi):
    """ This function computes the rotational transformation matrix
    in the format of a 2X2 NumPy array"""
    if Debug: Mess("SimTools-RotationMatrixNp")

    return np.array([[np.cos(phi), -np.sin(phi)],
                     [np.sin(phi),  np.cos(phi)]])
#  -------------------------------------------------------------------------
def Mess(string):
    CAD.Console.PrintMessage(str(string)+"\n")
#  -------------------------------------------------------------------------
def MessNoLF(string):
    CAD.Console.PrintMessage(str(string))
#  -------------------------------------------------------------------------
def PrintVec(vec):
    CAD.Console.PrintMessage("[" + str(Round(vec.x)) + ":" + str(Round(vec.y)) + ":" + str(Round(vec.z)) + "]\n")
#  -------------------------------------------------------------------------
def PrintNp3D(arr):
    if Debug: Mess("SimTools-PrintNp3D")

    for x in arr:
        for y in x:
            s = "[ "
            for z in y:
                ss = str(Round(z))+"                 "
                s = s + ss[:12]
            s = s + " ]"
            CAD.Console.PrintMessage(s+"\n")
        CAD.Console.PrintMessage("\n")
#  -------------------------------------------------------------------------
def PrintNp2D(arr):
    if Debug: Mess("SimTools-PrintNp2D")

    for x in arr:
        s = "[ "
        for y in x:
            ss = str(Round(y))+"                 "
            s = s + ss[:12]
        s = s + " ]"
        CAD.Console.PrintMessage(s+"\n")
#  -------------------------------------------------------------------------
def PrintNp1D(LF, arr):
    if Debug: Mess("SimTools-PrintNp1D")

    s = "[ "
    for y in arr:
        ss = str(Round(y))+"                 "
        s = s + ss[:12]
    s = s + " ]"
    if LF:
        CAD.Console.PrintMessage(s+"\n")
    else:
        CAD.Console.PrintMessage(s+" ")
#  -------------------------------------------------------------------------
def PrintNp1Ddeg(LF, arr):
    if Debug: Mess("SimTools-PrintNp1Ddeg")

    s = "[ "
    for y in arr:
        ss = str(Round(y*180.0/math.pi))+"                 "
        s = s + ss[:12]
    s = s + " ]"
    if LF:
        CAD.Console.PrintMessage(s+"\n")
    else:
        CAD.Console.PrintMessage(s+" ")
#  -------------------------------------------------------------------------
def Round(num):
    #if Debug: Mess("SimTools-Round")

    if num >= 0.0:
        return int((100.0 * num + 0.5))/100.0
    else:
        return int((100.0 * num - 0.5))/100.0
#  -------------------------------------------------------------------------
def NormalizeNpVec(vecNp):
    if Debug: Mess("SimMainC - NormalizeNpVec")

    mag = np.sqrt(vecNp[0]**2 + vecNp[1]**2)
    if mag > 1.0e-10:
        vecNp /= mag
    else:
        vecNp *= 0.0
    return vecNp
#  -------------------------------------------------------------------------
def Rot90NumPy(a):
    if Debug: Mess("SimTools-Rot90NumPy")

    aa = a.copy()
    bb = np.zeros((2,))
    bb[0], bb[1] = -aa[1], aa[0]
    return bb
#  -------------------------------------------------------------------------
def CADVecToNumPy(CADVec):
    if Debug: Mess("CADvecToNumPy")

    a = np.zeros((2,))
    a[0] = CADVec.x
    a[1] = CADVec.y
    return a
#  -------------------------------------------------------------------------
"""
def nicePhiPlease(vectorsRelativeCoG):
    if Debug:
        Mess("nicePhiPlease")

    # Start off with a big phi to check when a good one hasn't been found yet
    phi = 1e6

    # Approximate phi from the longest local point vector
    maxLength = 0
    maxVectorIndex = 0
    if Debug:
        Mess("---")
        for i in range(len(vectorsRelativeCoG)):
            PrintVec(vectorsRelativeCoG[i])
        Mess("===")
    for localIndex in range(len(vectorsRelativeCoG)):
        if Debug:
            Mess(math.sqrt(vectorsRelativeCoG[localIndex].dot(vectorsRelativeCoG[localIndex])))
        L = vectorsRelativeCoG[localIndex].dot(vectorsRelativeCoG[localIndex])
        if L > maxLength:
            maxLength = L
            maxVectorIndex = localIndex
    x = vectorsRelativeCoG[maxVectorIndex].x
    y = vectorsRelativeCoG[maxVectorIndex].y
    if abs(x) < 1e-8:
        phi = math.pi / 2.0
    elif abs(y) < 1e-8:
        phi = 0
    elif abs(1.0 - abs(y / x)) < 1e-8:
        if y / x >= 0.0:
            phi = math.pi / 4.0
        else:
            phi = -math.pi / 4.0
    elif abs(0.57735026919 - abs(y / x)) < 1e-8:
        if y / x >= 0.0:
            phi = math.pi / 6.0
        else:
            phi = -math.pi / 6.0
    elif abs(1.73205080757 - abs(y / x)) < 1e-8:
        if y / x >= 0:
            phi = math.pi / 3.0
        else:
            phi = -math.pi / 3.0

    # If not, be satisfied with any other nice phi
    # of any other local point vector longer than 1/10 of the max one

    # Try for a multiple of 90 degrees
    if phi == 1e6:
        for localIndex in range(len(vectorsRelativeCoG) - 1):
            length = vectorsRelativeCoG[localIndex].Length
            if length > maxLength / 10.0:
                x = vectorsRelativeCoG[maxVectorIndex].x
                y = vectorsRelativeCoG[maxVectorIndex].y
                if abs(x) < 1e-8:
                    phi = math.pi / 2.0
                    break
                elif abs(y) < 1e-8:
                    phi = 0.0
                    break
    # If not found, try for a multiple of 45 degrees
    if phi == 1e6:
        for localIndex in range(len(vectorsRelativeCoG) - 1):
            length = vectorsRelativeCoG[localIndex].Length
            if length > maxLength / 10.0:
                x = vectorsRelativeCoG[maxVectorIndex].x
                y = vectorsRelativeCoG[maxVectorIndex].y
                if abs(1.0 - abs(y / x)) < 1e-8:
                    if y / x >= 0.0:
                        phi = math.pi / 4.0
                        break
                    else:
                        phi = -math.pi / 4.0
                        break
    # Next try for a multiple of 30 degrees
    if phi == 1e6:
        for localIndex in range(len(vectorsRelativeCoG) - 1):
            length = vectorsRelativeCoG[localIndex].Length
            if length > maxLength / 10.0:
                x = vectorsRelativeCoG[maxVectorIndex].x
                y = vectorsRelativeCoG[maxVectorIndex].y
                if abs(0.57735026919 - abs(y / x)) < 1e-8:
                    if y / x >= 0.0:
                        phi = math.pi / 6.0
                        break
                    else:
                        phi = -math.pi / 6.0
                        break
                if abs(1.73205080757 - abs(y / x)) < 1e-8:
                    if y / x >= 0:
                        phi = math.pi / 3.0
                        break
                    else:
                        phi = -math.pi / 3.0
                        break
    # Give up trying to find a nice one - at least we tried!
    if phi == 1e6:
        phi = math.atan2(vectorsRelativeCoG[maxVectorIndex].y, vectorsRelativeCoG[maxVectorIndex].x)

    # Make -90 < phi < 90
    if abs(phi) > math.pi:
        if phi < 0.0:
            phi += math.pi
        else:
            phi -= math.pi
    if abs(phi) > math.pi / 2.0:
        if phi < 0.0:
            phi += math.pi
        else:
            phi -= math.pi

    return phi
"""
#  -------------------------------------------------------------------------
"""
def Contact(constraintIndex, indexPoint, bodyObj, kConst, eConst, FlagsList, penetrationDot0List,
            contact_LN_or_FM=True):
    penetration = -bodyObj.NPpointWorld[indexPoint].y
    if penetration > 0:
        penetrationDot = -bodyObj.NPpointWorldDot[indexPoint].y
        if FlagsList[constraintIndex] is False:
            penetrationDot0List[constraintIndex] = penetrationDot
            FlagsList[constraintIndex] = True

        if contact_LN_or_FM is True:
            forceY = Contact_LN(penetration, penetrationDot, penetrationDot0List[constraintIndex], kConst, eConst)
        else:
            forceY = Contact_FM(penetration, penetrationDot, penetrationDot0List[constraintIndex], kConst, eConst)

        Friction = CAD.Vector(0.0, forceY, 0.0)
        bodyObj.sumForces = bodyObj.sumForces + Friction
        bodyObj.sumMoments = bodyObj.sumMoments + bodyObj.pVecRot[indexPoint].dot(Friction)
    else:
        FlagsList[constraintIndex] = False
"""
#  -------------------------------------------------------------------------
"""
def Contact_FM(delta, deltaDot, deltaDot0, kConst, eConst):
    return kConst * (delta ** 1.5) * (1 + 8 * (1 - eConst) * deltaDot / (5 * eConst * deltaDot0))
"""
#  -------------------------------------------------------------------------
"""
def Contact_LN(delta, deltaDot, deltaDot0, kConst, eConst):
    return kConst * (delta ** 1.5) * (1 + 3 * (1 - eConst * eConst) * deltaDot / (4 * deltaDot0))
"""
#  -------------------------------------------------------------------------
"""
def Friction_A(mu_s, mu_d, v_s, p, k_t, v, fN):
    return fN * (mu_d + (mu_s - mu_d) * exp(-(abs(v) / v_s) ** p)) * tanh(k_t * v)
"""
#  -------------------------------------------------------------------------
"""
def Friction_B(mu_s, mu_d, mu_v, v_t, fnt, v, fN):
    vr = v / v_t
    return fN * (mu_d * tanh(4 * vr) + (mu_s - mu_d) *
                 vr / (0.25 * vr * vr + 0.75) ** 2) + mu_v * v * tanh(4 * fN / fnt)
"""
#  -------------------------------------------------------------------------
"""
def initComboInFormF(ComboName, LabelList, index):
    ComboName.clear()
    comboLabels = ['Undefined'] + LabelList.copy()
    ComboName.addItems(comboLabels)
    ComboName.setCurrentIndex(index + 1)
    """
#  -------------------------------------------------------------------------
"""
def updateToolTipF(ComboName, LabelList):
    bodyString = ""
    for point in LabelList:
        if point != 'Undefined' or len(LabelList) == 1:
            bodyString = bodyString + parsePoint(point) + "\n========\n"
    ComboName.setToolTip(str(bodyString[:-10]))
"""
# ==============================================================================
"""
def getDictionary(SimName):
    Run through the Active simGlobal group and
    return a dictionary with 'SimName', vs objects
    if Debug:
        Mess("SimToolsClass-getDictionary")
    SimDictionary = {}
    simGlobal = getsimGlobalObject()
    for groupMember in simGlobal.Group:
        if SimName in groupMember.Name:
            SimDictionary[groupMember.Name] = groupMember
    return SimDictionary
"""
#  -------------------------------------------------------------------------
####################################################################
# HERE IS THE 'DICTIONARY' to the  DICTIONARIES in the Sim workbench
# ####################################################################
#
# bodyObjDict:          Sim Body Object simGlobal Name --> Sim Body Object
# jointObjDict:         Sim Joint simGlobal Name --> Sim Joint simGlobal object
# forceObjDict:         Sim Force simGlobal Name --> Sim Force simGlobal object
# DictionaryOfPoints:   Sim Body Object simGlobal Name -->
#                       Dict: FreeCAD point name --> index number in the point list in the Body simGlobal
# driverObjDict:        joint name --> initialised instance of function class
# cardID2cardData:      materialID --> cardData --> data on card
# cardID2cardName:      materiaID --> material name in the card

####################################################################
# List of available buttons in the task dialog
####################################################################
# NoButton        = 0x00000000,     Ok     = 0x00000400,     Save    = 0x00000800,
# SaveAll         = 0x00001000,     Open   = 0x00002000,     Yes     = 0x00004000,
# YesToAll        = 0x00008000,     No     = 0x00010000,     NoToAll = 0x00020000,
# Abort           = 0x00040000,     Retry  = 0x00080000,     Ignore  = 0x00100000,
# Close           = 0x00200000,     Cancel = 0x00400000,     Discard = 0x00800000,
# Help            = 0x01000000,     Apply  = 0x02000000,     Reset   = 0x04000000,
# RestoreDefaults = 0x08000000,
#  -------------------------------------------------------------------------
"""
def getPointsFromBodyName(bodyName, bodyObjDict):
    Generate the Point Label list which corresponds to the given body name
    if Debug:
        Mess("SimTools-getPointsFromBodyName")

    ListNames = []
    ListLabels = []
    if bodyName != "":
        bodyObj = bodyObjDict[bodyName]
        return bodyObj.JointNameList.copy(), bodyObj.pointLabels.copy()
    else:
        return [], []
    """
# --------------------------------------------------------------------------
"""
def condensePoints(JointNameList, pointLabels, pointLocals):
    Condense all the duplicate points into one
    if Debug:
        Mess("SimTools-condensePoints")

    # Condense all the duplicate points in this specific body into one
    numPoints = len(pointLocals)
    i = 0
    while i < numPoints:
        j = i + 1
        while j < numPoints:
            if abs(pointLocals[i].x - pointLocals[j].x) < 1.0e-10:
                if abs(pointLocals[i].y - pointLocals[j].y) < 1.0e-10:
                    if abs(pointLocals[i].z - pointLocals[j].z) < 1.0e-10:
                        # Compare the body names (i.e. the string from beginning to '{'
                        labeli = pointLabels[i][:pointLabels[i].index('{')]
                        labelj = pointLabels[j][:pointLabels[j].index('{')]
                        if labeli == labelj:
                            if Debug:
                                MessNoLF("Combining: ")
                                MessNoLF(pointLabels[i])
                                MessNoLF(" and ")
                                Mess(pointLabels[j])
                            JointNameList[i] = JointNameList[i] + "-" + JointNameList[j][JointNameList[j].index('{'):]
                            pointLabels[i] = pointLabels[i] + "-" + pointLabels[j][pointLabels[j].index('{'):]
                            # Shift the others up to remove the duplicate
                            k = j + 1
                            while k < numPoints:
                                JointNameList[k - 1] = JointNameList[k]
                                pointLabels[k - 1] = pointLabels[k]
                                pointLocals[k - 1] = pointLocals[k]
                                k += 1
                            # Pop the bottom item off the lists
                            JointNameList.pop()
                            pointLabels.pop()
                            pointLocals.pop()
                            numPoints -= 1
            j += 1
        i += 1
    # Now, condense all the duplicate points into one, irrespective of body name
    numPoints = len(pointLocals)
    i = 0
    while i < numPoints:
        j = i + 1
        while j < numPoints:
            if abs(pointLocals[i].x - pointLocals[j].x) < 1.0e-10:
                if abs(pointLocals[i].y - pointLocals[j].y) < 1.0e-10:
                    if abs(pointLocals[i].z - pointLocals[j].z) < 1.0e-10:
                        if Debug:
                            MessNoLF("Combining: ")
                            MessNoLF(pointLabels[i])
                            MessNoLF(" and ")
                            Mess(pointLabels[j])
                        JointNameList[i] = JointNameList[i] + "-" + JointNameList[j]
                        pointLabels[i] = pointLabels[i] + "-" + pointLabels[j]
                        if Debug:
                            Mess(pointLabels[i])
                        # Shift the others up to remove the duplicate
                        k = j + 1
                        while k < numPoints:
                            JointNameList[k - 1] = JointNameList[k]
                            pointLabels[k - 1] = pointLabels[k]
                            pointLocals[k - 1] = pointLocals[k]
                            k += 1
                        # Pop the bottom item off the lists
                        JointNameList.pop()
                        pointLabels.pop()
                        pointLocals.pop()
                        numPoints -= 1
            j += 1
        i += 1

    # If we are debugging, Print out all the body's and point's placements etc
    if Debug:
        for index in range(len(JointNameList)):
            Mess("-------------------------------------------------------------------")
            Mess("Point Name: " + str(JointNameList[index]))
            Mess("Point Label:  " + str(pointLabels[index]))
            Mess("")
            MessNoLF("Point Local Vector:")
            PrintVec(pointLocals[index])
            MessNoLF("Point World Vector:")
            PrintVec(CAD.Vector(BodyPlacementMatrix.multVec(pointLocals[index])))
            Mess("")
"""
#  -------------------------------------------------------------------------
"""
def parsePoint(pointString):
    Split all the LCS points for this component into a tip string
    while True:
        try:
            a = pointString.index("}-{")
        except BaseException as e:
            break
        pointString = pointString[:a]+"\n  -->"+pointString[a+3:]
    while True:
        try:
            a = pointString.index("-{")
        except BaseException as e:
            break
        pointString = pointString[:a]+"\n  -->"+pointString[a+2:]
    while True:
        try:
            a = pointString.index("}-")
        except BaseException as e:
            break
        pointString = pointString[:a]+"\n"+pointString[a+2:]
    while True:
        try:
            a = pointString.index("}")
        except BaseException as e:
            break
        pointString = pointString[:a]
    return pointString
"""
# --------------------------------------------------------------------------
"""def decorateObject(objectToDecorate, body_I_object, body_J_object):
    # Get the world coordinates etc. of the A point
    solidNameAList = []
    solidPlacementAList = []
    solidBoxAList = []
    worldPointA = CAD.Vector()
    if objectToDecorate.pointHeadName != "":
        # Find the bounding boxes of the component solids
        # solidNameList - names of the solids
        # solidPlacementList - The Placement.Base are the world coordinates of the solid origin
        # solidBoxList - The BoundBox values are the rectangular cartesian world coordinates of the bounding box
        Document = CAD.ActiveDocument
        for solidName in body_I_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameAList.append(solidName)
            solidPlacementAList.append(solidObj.Placement)
            solidBoxAList.append(solidObj.Shape.BoundBox)

        # Get the A world Placement of the compound Sim body
        worldPlacement = body_I_object.world
        if Debug:
            MessNoLF("Main Body World A Placement: ")
            Mess(worldPlacement)
        pointIndex = body_I_object.JointNameList.index(objectToDecorate.pointHeadName)
        pointHeadLocal = body_I_object.pointLocals[pointIndex]
        worldPointA = worldPlacement.toMatrix().multVec(pointHeadLocal)

    # Get the world coordinates etc. of the B of the point
    solidNameBList = []
    solidPlacementBList = []
    solidBoxBList = []
    worldPointB = CAD.Vector()
    if objectToDecorate.pointTailName != "":
        # Find the bounding boxes of the component solids
        Document = CAD.ActiveDocument
        for solidName in body_J_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameBList.append(solidName)
            solidPlacementBList.append(solidObj.Placement)
            solidBoxBList.append(solidObj.Shape.BoundBox)

        # Get the B world Placement of the compound Sim body
        worldBPlacement = body_J_object.world
        if Debug:
            MessNoLF("Main Body World B Placement: ")
            Mess(worldBPlacement)
        pointIndex = body_J_object.JointNameList.index(objectToDecorate.pointTailName)
        pointTailLocal = body_J_object.pointLocals[pointIndex]
        worldPointB = worldBPlacement.toMatrix().multVec(pointTailLocal)

    if Debug:
        Mess("Solid lists:")
        for i in range(len(solidNameAList)):
            MessNoLF(solidNameAList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementAList[i])
            MessNoLF(" -- ")
            Mess(solidBoxAList[i])
        for i in range(len(solidNameBList)):
            MessNoLF(solidNameBList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementBList[i])
            MessNoLF(" -- ")
            Mess(solidBoxBList[i])
        MessNoLF("World A point: ")
        PrintVec(worldPointA)
        MessNoLF("World B point: ")
        PrintVec(worldPointB)

    # Identify in which solid bounding box, the A and B points are
    ASolidIndex = BSolidIndex = 1
    for boxIndex in range(len(solidBoxAList)):
        if solidBoxAList[boxIndex].isInside(worldPointA):
            if Debug:
                MessNoLF("A point inside: ")
                Mess(solidNameAList[boxIndex])
            ASolidIndex = boxIndex
    for boxIndex in range(len(solidBoxBList)):
        if solidBoxBList[boxIndex].isInside(worldPointB):
            if Debug:
                MessNoLF("B point inside: ")
                Mess(solidNameBList[boxIndex])
            BSolidIndex = boxIndex

    if objectToDecorate.pointHeadName != "" and objectToDecorate.pointTailName != "":
        # Draw some shapes in the gui, to show the point positions
        CurrentJointType = objectToDecorate.JointType
        planeNormal = getsimGlobalObject().movementPlaneNormal
        pointHead = CAD.Vector(worldPointA)
        pointTail = CAD.Vector(worldPointB)

        # Do some calculation of the torus sizes for Rev and Rev-Rev points
        if CurrentJointType == 0 or CurrentJointType == 2:
            # Depending on the plane normal:
            # Move the two thickness coordinates to their average,
            # Squash the thickness to zero, and
            # Set the point Diameter to the middle value of the Intersection box's xlength ylength zlength
            if planeNormal.x > 1e-6:
                pointHead.x = (worldPointA.x + worldPointB.x) / 2.0
                pointTail.x = point_I_i.x
            elif planeNormal.y > 1e-6:
                point_I_i.y = (worldPointA.y + worldPointB.y) / 2.0
                pointTail.y = point_I_i.y
            elif planeNormal.z > 1e-6:
                point_I_i.z = (worldPointA.z + worldPointB.z) / 2.0
                pointTail.z = point_I_i.z

        # Draw the yellow circular arrow in the case of 'Rev' point
        if CurrentJointType == 0 \
                and objectToDecorate.point_I_iName != "" \
                and objectToDecorate.pointTailName != "":
            pointDiam = minMidMax3(boxIntersection.XLength,
                                   boxIntersection.YLength,
                                   boxIntersection.ZLength,
                                   2)
            # Only draw if the diameter non-zero
            if pointDiam > 1e-6:
                # Draw the Left side
                # False == Left side
                Shape1 = DrawRotArrow(point_I_i, False, pointDiam)
                # Draw the Right side
                # True == Right side
                Shape2 = DrawRotArrow(pointTail, True, pointDiam)
                Shape = Part.makeCompound([Shape1, Shape2])
                objectToDecorate.Shape = Shape
                objectToDecorate.ViewObject.ShapeColor = (1.0, 1.0, 0.5, 1.0)
                objectToDecorate.ViewObject.Transparency = 20

        # Draw a straight red arrow in the case of 'trans' point
        elif CurrentJointType == 1:
            Shape = DrawTransArrow(worldPointB, worldPointA, 15)
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 0.5, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw the two circular arrows and a line in the case of 'Revolute-Revolute' point
        elif CurrentJointType == 2:
            point_I_iDiam = minMidMax3(solidBoxAList[ASolidIndex].XLength,
                                       solidBoxAList[ASolidIndex].YLength,
                                       solidBoxAList[ASolidIndex].ZLength,
                                       2)
            pointTailDiam = minMidMax3(solidBoxBList[BSolidIndex].XLength,
                                        solidBoxBList[BSolidIndex].YLength,
                                        solidBoxBList[BSolidIndex].ZLength,
                                        2)
            # Draw both left and right sides of the torus
            Shape1 = DrawRotArrow(point_I_i, True, point_I_iDiam / 2)
            Shape2 = DrawRotArrow(point_I_i, False, point_I_iDiam / 2)
            Shape3 = DrawRotArrow(pointTail, True, pointTailDiam / 2)
            Shape4 = DrawRotArrow(pointTail, False, pointTailDiam / 2)
            # Draw the arrow
            Shape5 = DrawTransArrow(point_I_i, pointTail, point_I_iDiam / 2)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3, Shape4, Shape5])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 1.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20
            objectToDecorate.lengthLink = (point_I_i - pointTail).Length

        # Draw a circular arrow and a line in the case of 'Revolute-Translation' point
        elif CurrentJointType == 3:
            Shape1 = DrawRotArrow(worldPointA, True, 3)
            Shape2 = DrawRotArrow(worldPointA, False, 3)
            Shape3 = DrawTransArrow(worldPointA, worldPointB, 15)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (0.5, 1.0, 1.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw a Bolt in the case of 'Fixed' point
        elif CurrentJointType == 7:
            point_I_ilen = minMidMax3(boxIntersection.XLength,
                                      boxIntersection.YLength,
                                      boxIntersection.ZLength,
                                      3)
            point_I_idiam = minMidMax3(boxIntersection.XLength,
                                       boxIntersection.YLength,
                                       boxIntersection.ZLength,
                                       2)
            objectToDecorate.Shape = DrawFixedBolt(worldPointA, point_I_idiam / 3, point_I_ilen / 3)
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 0.5, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        else:
            # Add a null shape to the object for the other more fancy types
            # TODO : The appropriate shapes may be added at a later time
            objectToDecorate.Shape = Part.Shape()
"""
#  -------------------------------------------------------------------------
"""def DrawRotArrow(Point, LeftRight, diameter):
    Draw a yellow circle arrow around a Revolute point
    We first draw it relative to the X-Y plane
    and then rotate it relative to the defined movement plane
    radiusRing = diameter
    thicknessRing = diameter / 5
    torus_direction = CAD.Vector(0, 0, 1)
    cone_direction = CAD.Vector(0, 1, 0)

    # Make either a left half or a right half of the torus
    if LeftRight:
        torus = Part.makeTorus(radiusRing, thicknessRing, CAD.Vector(0, 0, radiusRing), torus_direction, -180, 180, 90)
        cone_position = CAD.Vector(radiusRing, -5 * thicknessRing, radiusRing)
        cone = Part.makeCone(0, 2 * thicknessRing, 5 * thicknessRing, cone_position, cone_direction)
    else:
        torus = Part.makeTorus(radiusRing, thicknessRing, CAD.Vector(0, 0, radiusRing), -torus_direction, -180, 180, 90)
        cone_position = CAD.Vector(-radiusRing, -5 * thicknessRing, radiusRing)
        cone = Part.makeCone(0, 2 * thicknessRing, 5 * thicknessRing, cone_position, cone_direction)
    # Make a cone to act as an arrow on the end of the half torus
    torus_w_arrows = Part.makeCompound([torus, cone])
    # Rotate torus to be relative to the defined movement plane
    rotationToMovementPlane = CAD.Rotation(CAD.Vector(0, 0, 1), getsimGlobalObject().movementPlaneNormal)
    torus_w_arrows.applyRotation(rotationToMovementPlane)
    # Translate the torus to be located at the point
    torus_w_arrows.applyTranslation(Point)

    return torus_w_arrows
"""
#  -------------------------------------------------------------------------
"""def DrawFixedBolt(Point, diam, length):
    bolt = Part.makeCylinder(
        diam,
        length * 2,
        Point - CAD.Vector(0, 0, length),
        CAD.Vector(0, 0, 1)
    )

    # FORMAT OF MAKE WEDGE:
    # ====================
    # makeWedge (xbotleftbase, basey, zbotleftbase,
    #            zbotleftroof,xbotleftroof,
    #            xtoprightbase,roofy,ztoprightbase,
    #            ztoprightroof,xtoprightroof

    sin = math.sin(math.pi / 3)
    cos = math.cos(math.pi / 3)

    nuta = Part.makeWedge(-diam, -diam * 2 * sin, -length * 2 / 9,
                          -length * 2 / 9, 0,
                          diam, 0, length * 2 / 9,
                          length * 2 / 9, 0)
    nutb = nuta.copy().mirror(CAD.Vector(0, 0, 0), CAD.Vector(sin, cos, 0))
    nutc = nuta.copy().mirror(CAD.Vector(0, 0, 0), CAD.Vector(sin, -cos, 0))
    nutd = nuta.fuse([nutb, nutc])
    nute = nutd.copy().mirror(CAD.Vector(0, 0, 0), CAD.Vector(0, 1, 0))
    nutf = nutd.fuse([nute])
    nutg = nutf.copy()
    nutf.translate(Point - CAD.Vector(0, 0, length * 0.7))
    nutg.translate(Point + CAD.Vector(0, 0, length * 0.7))

    return bolt.fuse([nutf, nutg])
    """
#  -------------------------------------------------------------------------
"""
def DrawTransArrow(Point1, Point2, diameter):
    # Draw an arrow as long as the distance of the vector between the points
    llen = (Point1 - Point2).Length
    if llen > 1e-6:
        # Direction of the arrow is the direction of the vector between the two points
        lin_move_dir = (Point2 - Point1).normalize()
        cylinder = Part.makeCylinder(
            diameter * 0.3,
            0.60 * llen,
            Point1 + 0.2 * llen * lin_move_dir,
            lin_move_dir,
        )
        # Draw the two arrow heads
        cone1 = Part.makeCone(0, diameter, 0.2 * llen, Point1, lin_move_dir)
        cone2 = Part.makeCone(0, diameter, 0.2 * llen, Point2, -lin_move_dir)
        return Part.makeCompound([cylinder, cone1, cone2])
    return Part.Shape()
"""
# --------------------------------------------------------------------------
"""
def decorateObjectLegacy(objectToDecorate, body_I_object, body_J_object):
     # Get the world coordinates etc. of the A point
    solidNameAList = []
    solidPlacementAList = []
    solidBoxAList = []
    worldPointA = CAD.Vector()
    if objectToDecorate.pointHeadName != "":
        # Find the bounding boxes of the component solids
        # solidNameList - names of the solids
        # solidPlacementList - The Placement.Base are the world coordinates of the solid origin
        # solidBoxList - The BoundBox values are the rectangular cartesian world coordinates of the bounding box
        Document = CAD.ActiveDocument
        for solidName in body_I_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameAList.append(solidName)
            solidPlacementAList.append(solidObj.Placement)
            solidBoxAList.append(solidObj.Shape.BoundBox)

        # Get the A world Placement of the compound Sim body
        worlSimlacement = body_I_object.world
        if Debug:
            MessNoLF("Main Body World A Placement: ")
            Mess(worlSimlacement)
        pointIndex = body_I_object.JointNameList.index(objectToDecorate.pointHeadName)
        pointHeadLocal = body_I_object.pointLocals[pointIndex]
        worldPointA = worlSimlacement.toMatrix().multVec(pointHeadLocal)

    # Get the world coordinates etc. of the B of the point
    solidNameBList = []
    solidPlacementBList = []
    solidBoxBList = []
    worldPointB = CAD.Vector()
    if objectToDecorate.pointTailName != "":
        # Find the bounding boxes of the component solids
        Document = CAD.ActiveDocument
        for solidName in body_J_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameBList.append(solidName)
            solidPlacementBList.append(solidObj.Placement)
            solidBoxBList.append(solidObj.Shape.BoundBox)

        # Get the B world Placement of the compound Sim body
        worldBPlacement = body_J_object.world
        if Debug:
            MessNoLF("Main Body World B Placement: ")
            Mess(worldBPlacement)
        pointIndex = body_J_object.JointNameList.index(objectToDecorate.pointTailName)
        pointTailLocal = body_J_object.pointLocals[pointIndex]
        worldPointB = worldBPlacement.toMatrix().multVec(pointTailLocal)

    if Debug:
        Mess("Solid lists:")
        for i in range(len(solidNameAList)):
            MessNoLF(solidNameAList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementAList[i])
            MessNoLF(" -- ")
            Mess(solidBoxAList[i])
        for i in range(len(solidNameBList)):
            MessNoLF(solidNameBList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementBList[i])
            MessNoLF(" -- ")
            Mess(solidBoxBList[i])
        MessNoLF("World A point: ")
        PrintVec(worldPointA)
        MessNoLF("World B point: ")
        PrintVec(worldPointB)

    # Identify in which solid bounding box, the A and B points are
    ASolidIndex = BSolidIndex = 1
    for boxIndex in range(len(solidBoxAList)):
        if solidBoxAList[boxIndex].isInside(worldPointA):
            if Debug:
                MessNoLF("A point inside: ")
                Mess(solidNameAList[boxIndex])
            ASolidIndex = boxIndex
    for boxIndex in range(len(solidBoxBList)):
        if solidBoxBList[boxIndex].isInside(worldPointB):
            if Debug:
                MessNoLF("B point inside: ")
                Mess(solidNameBList[boxIndex])
            BSolidIndex = boxIndex

    if objectToDecorate.pointHeadName != "" and objectToDecorate.pointTailName != "":
        # Draw some shapes in the gui, to show the point positions
        boxIntersection = solidBoxAList[ASolidIndex].intersected(solidBoxBList[BSolidIndex])
        CurrentJointType = objectToDecorate.JointType
        planeNormal = getsimGlobalObject().movementPlaneNormal
        pointHead = CAD.Vector(worldPointA)
        pointTail = CAD.Vector(worldPointB)

        # Do some calculation of the torus sizes for Rev and Rev-Rev points
        if CurrentJointType == 0 or CurrentJointType == 2:
            # Depending on the plane normal:
            # Move the two thickness coordinates to their average,
            # Squash the thickness to zero, and
            # Set the point Diameter to the middle value of the Intersection box's xlength ylength zlength
            if planeNormal.x > 1e-6:
                pointHead.x = (worldPointA.x + worldPointB.x) / 2.0
                pointTail.x = point_I_i.x
            elif planeNormal.y > 1e-6:
                point_I_i.y = (worldPointA.y + worldPointB.y) / 2.0
                pointTail.y = point_I_i.y
            elif planeNormal.z > 1e-6:
                point_I_i.z = (worldPointA.z + worldPointB.z) / 2.0
                pointTail.z = point_I_i.z

        # Draw the yellow circular arrow in the case of 'Rev' point
        if CurrentJointType == 0 \
                and objectToDecorate.point_I_iName != "" \
                and objectToDecorate.pointTailName != "":
            pointDiam = minMidMax3(boxIntersection.XLength,
                                   boxIntersection.YLength,
                                   boxIntersection.ZLength,
                                   2)
            # Only draw if the diameter non-zero
            if pointDiam > 1e-6:
                # Draw the Left side
                # False == Left side
                Shape1 = DrawRotArrow(point_I_i, False, pointDiam)
                # Draw the Right side
                # True == Right side
                Shape2 = DrawRotArrow(pointTail, True, pointDiam)
                Shape = Part.makeCompound([Shape1, Shape2])
                objectToDecorate.Shape = Shape
                objectToDecorate.ViewObject.ShapeColor = (1.0, 1.0, 0.5, 1.0)
                objectToDecorate.ViewObject.Transparency = 20

        # Draw a straight red arrow in the case of 'trans' point
        elif CurrentJointType == 1:
            Shape = DrawTransArrow(worldPointB, worldPointA, 15)
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 0.5, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw the two circular arrows and a line in the case of 'Revolute-Revolute' point
        elif CurrentJointType == 2:
            point_I_iDiam = minMidMax3(solidBoxAList[ASolidIndex].XLength,
                                       solidBoxAList[ASolidIndex].YLength,
                                       solidBoxAList[ASolidIndex].ZLength,
                                       2)
            pointTailDiam = minMidMax3(solidBoxBList[BSolidIndex].XLength,
                                        solidBoxBList[BSolidIndex].YLength,
                                        solidBoxBList[BSolidIndex].ZLength,
                                        2)
            # Draw both left and right sides of the torus
            Shape1 = DrawRotArrow(point_I_i, True, point_I_iDiam / 2)
            Shape2 = DrawRotArrow(point_I_i, False, point_I_iDiam / 2)
            Shape3 = DrawRotArrow(pointTail, True, pointTailDiam / 2)
            Shape4 = DrawRotArrow(pointTail, False, pointTailDiam / 2)
            # Draw the arrow
            Shape5 = DrawTransArrow(point_I_i, pointTail, point_I_iDiam / 2)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3, Shape4, Shape5])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 1.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20
            objectToDecorate.lengthLink = (point_I_i - pointTail).Length

        # Draw a circular arrow and a line in the case of 'Revolute-Translation' point
        elif CurrentJointType == 3:
            Shape1 = DrawRotArrow(worldPointA, True, 3)
            Shape2 = DrawRotArrow(worldPointA, False, 3)
            Shape3 = DrawTransArrow(worldPointA, worldPointB, 15)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (0.5, 1.0, 1.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw a Bolt in the case of 'Fixed' point
        elif CurrentJointType == 7:
            point_I_ilen = minMidMax3(boxIntersection.XLength,
                                      boxIntersection.YLength,
                                      boxIntersection.ZLength,
                                      3)
            point_I_idiam = minMidMax3(boxIntersection.XLength,
                                       boxIntersection.YLength,
                                       boxIntersection.ZLength,
                                       2)
            objectToDecorate.Shape = DrawFixedBolt(worldPointA, point_I_idiam / 3, point_I_ilen / 3)
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.5, 0.5, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        else:
            # Add a null shape to the object for the other more fancy types
            # TODO : The appropriate shapes may be added at a later time
            objectToDecorate.Shape = Part.Shape()
    """
# --------------------------------------------------------------------------
"""
def OldDecorate():
    # Get the world coordinates etc. of the A point
    solidNameAList = []
    solidPlacementAList = []
    solidBoxAList = []
    worldPointA = CAD.Vector()
    if objectToDecorate.point_I_iName != "":
        # Find the bounding boxes of the component solids
        # solidNameList - names of the solids
        # solidPlacementList - The Placement.Base are the world coordinates of the solid origin
        # solidBoxList - The BoundBox values are the rectangular cartesian world coordinates of the bounding box
        Document = CAD.ActiveDocument
        for solidName in body_I_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameAList.append(solidName)
            solidPlacementAList.append(solidObj.Placement)
            solidBoxAList.append(solidObj.Shape.BoundBox)

        # Get the A world Placement of the compound Sim body
        worlSimlacement = body_I_object.world
        if Debug:
            MessNoLF("Main Body World A Placement: ")
            Mess(worlSimlacement)
        pointIndex = body_I_object.JointNameList.index(objectToDecorate.point_I_iName)
        point_I_iLocal = body_I_object.pointLocals[pointIndex]
        worldPointA = worlSimlacement.toMatrix().multVec(point_I_iLocal)

    # Get the world coordinates etc. of the B of the point
    solidNameBList = []
    solidPlacementBList = []
    solidBoxBList = []
    worldPointB = CAD.Vector()
    if objectToDecorate.pointTailName != "":
        # Find the bounding boxes of the component solids
        Document = CAD.ActiveDocument
        for solidName in body_J_object.ass4SolidsNames:
            solidObj = Document.findObjects(Name="^" + solidName + "$")[0]
            solidNameBList.append(solidName)
            solidPlacementBList.append(solidObj.Placement)
            solidBoxBList.append(solidObj.Shape.BoundBox)

        # Get the B world Placement of the compound Sim body
        worldBPlacement = body_J_object.world
        if Debug:
            MessNoLF("Main Body World B Placement: ")
            Mess(worldBPlacement)
        pointIndex = body_J_object.JointNameList.index(objectToDecorate.pointTailName)
        pointTailLocal = body_J_object.pointLocals[pointIndex]
        worldPointB = worldBPlacement.toMatrix().multVec(pointTailLocal)

    if Debug:
        Mess("Solid lists:")
        for i in range(len(solidNameAList)):
            MessNoLF(solidNameAList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementAList[i])
            MessNoLF(" -- ")
            Mess(solidBoxAList[i])
        for i in range(len(solidNameBList)):
            MessNoLF(solidNameBList[i])
            MessNoLF(" -- ")
            MessNoLF(solidPlacementBList[i])
            MessNoLF(" -- ")
            Mess(solidBoxBList[i])
        MessNoLF("World A point: ")
        PrintVec(worldPointA)
        MessNoLF("World B point: ")
        PrintVec(worldPointB)

    # Identify in which solid bounding box, the A and B points are
    ASolidIndex = BSolidIndex = 1
    for boxIndex in range(len(solidBoxAList)):
        if solidBoxAList[boxIndex].isInside(worldPointA):
            if Debug:
                MessNoLF("A point inside: ")
                Mess(solidNameAList[boxIndex])
            ASolidIndex = boxIndex
    for boxIndex in range(len(solidBoxBList)):
        if solidBoxBList[boxIndex].isInside(worldPointB):
            if Debug:
                MessNoLF("B point inside: ")
                Mess(solidNameBList[boxIndex])
            BSolidIndex = boxIndex

    if objectToDecorate.point_I_iName != "" and objectToDecorate.pointTailName != "":
        # Draw some shapes in the gui, to show the point positions
        boxIntersection = solidBoxAList[ASolidIndex].intersected(solidBoxBList[BSolidIndex])
        CurrentJointType = objectToDecorate.JointType
        planeNormal = getsimGlobalObject().movementPlaneNormal
        point_I_i = CAD.Vector(worldPointA)
        pointTail = CAD.Vector(worldPointB)

        # Do some calculation of the torus sizes for Rev and Rev-Rev points
        if CurrentJointType == 0 or CurrentJointType == 2:
            # Depending on the plane normal:
            # Move the two thickness coordinates to their average,
            # Squash the thickness to zero, and
            # Set the point Diameter to the middle value of the Intersection box's xlength ylength zlength
            if planeNormal.x > 1e-6:
                point_I_i.x = (worldPointA.x + worldPointB.x) / 2.0
                pointTail.x = point_I_i.x
            elif planeNormal.y > 1e-6:
                point_I_i.y = (worldPointA.y + worldPointB.y) / 2.0
                pointTail.y = point_I_i.y
            elif planeNormal.z > 1e-6:
                point_I_i.z = (worldPointA.z + worldPointB.z) / 2.0
                pointTail.z = point_I_i.z

        # Draw the yellow circular arrow in the case of 'Rev' point
        if CurrentJointType == 0 \
                and objectToDecorate.point_I_iName != "" \
                and objectToDecorate.pointTailName != "":
            pointDiam = minMidMax3(boxIntersection.XLength,
                                   boxIntersection.YLength,
                                   boxIntersection.ZLength,
                                   2)
            # Only draw if the diameter non-zero
            if pointDiam > 1e-6:
                # Draw the Left side
                # False == Left side
                Shape1 = DrawRotArrow(point_I_i, False, pointDiam)
                # Draw the Right side
                # True == Right side
                Shape2 = DrawRotArrow(pointTail, True, pointDiam)
                Shape = Part.makeCompound([Shape1, Shape2])
                objectToDecorate.Shape = Shape
                objectToDecorate.ViewObject.ShapeColor = (1.0, 1.0, 0.0)
                objectToDecorate.ViewObject.Transparency = 20

        # Draw a straight red arrow in the case of 'trans' point
        elif CurrentJointType == 1:
            Shape = DrawTransArrow(worldPointB, worldPointA, 15)
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.0, 0.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw the two circular arrows and a line in the case of 'Revolute-Revolute' point
        elif CurrentJointType == 2:
            point_I_iDiam = minMidMax3(solidBoxAList[ASolidIndex].XLength,
                                       solidBoxAList[ASolidIndex].YLength,
                                       solidBoxAList[ASolidIndex].ZLength,
                                       2)
            pointTailDiam = minMidMax3(solidBoxBList[BSolidIndex].XLength,
                                       solidBoxBList[BSolidIndex].YLength,
                                       solidBoxBList[BSolidIndex].ZLength,
                                       2)
            # Draw both left and right sides of the torus
            Shape1 = DrawRotArrow(point_I_i, True, point_I_iDiam / 2)
            Shape2 = DrawRotArrow(point_I_i, False, point_I_iDiam / 2)
            Shape3 = DrawRotArrow(pointTail, True, pointTailDiam / 2)
            Shape4 = DrawRotArrow(pointTail, False, pointTailDiam / 2)
            # Draw the arrow
            Shape5 = DrawTransArrow(point_I_i, pointTail, point_I_iDiam / 2)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3, Shape4, Shape5])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20
            objectToDecorate.lengthLink = (point_I_i - pointTail).Length

        # Draw a circular arrow and a line in the case of 'Translation-Revolute' point
        elif CurrentJointType == 3:
            Shape1 = DrawRotArrow(worldPointA, True, 3)
            Shape2 = DrawRotArrow(worldPointA, False, 3)
            Shape3 = DrawTransArrow(worldPointA, worldPointB, 15)
            Shape = Part.makeCompound([Shape1, Shape2, Shape3])
            objectToDecorate.Shape = Shape
            objectToDecorate.ViewObject.ShapeColor = (0.0, 1.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        # Draw a Bolt in the case of 'Fixed' point
        elif CurrentJointType == 7:
            point_I_ilen = minMidMax3(boxIntersection.XLength,
                                      boxIntersection.YLength,
                                      boxIntersection.ZLength,
                                      3)
            point_I_idiam = minMidMax3(boxIntersection.XLength,
                                       boxIntersection.YLength,
                                       boxIntersection.ZLength,
                                       2)
            objectToDecorate.Shape = DrawFixedBolt(worldPointA, point_I_idiam / 3, point_I_ilen / 3)
            objectToDecorate.ViewObject.ShapeColor = (1.0, 0.0, 0.0, 1.0)
            objectToDecorate.ViewObject.Transparency = 20

        else:
            # Add a null shape to the object for the other more fancy types
            # TODO : The appropriate shapes may be added at a later time
            objectToDecorate.Shape = Part.Shape() """


#  -------------------------------------------------------------------------
"""
def getAllSolidsLists():
    Run through the Solids and return lists of Names / Labels
    and a list of all the actual assembly objects
    allSolidsNames = []
    allSolidsLabels = []
    allSolidsObjects = []

    # Run through all the whole document's objects, looking for the Solids
    objects = CAD.ActiveDocument.Objects
    for obj in objects:
        if hasattr(obj, "Type") and obj.Type == 'Assembly':
            if Debug:
                Mess(obj.Name)
            SolidsObject = obj
            break
    else:
        Mess("No Assembly 4 object found")
        return allSolidsNames, allSolidsLabels, allSolidsObjects

    # Find all the parts
    # A part is searched for as something which is attached to something,
    # by means of something, and which has a shape
    for groupMember in SolidsObject.Group:
        if hasattr(groupMember, 'AttachedTo') and \
                hasattr(groupMember, 'AttachedBy') and \
                hasattr(groupMember, 'Shape'):
            if "^" + groupMember.Name + "$" in allSolidsNames:
                CAD.Console.PrintError("Duplicate Shape Name found: " + groupMember.Name + "\n")

            allSolidsNames.append(groupMember.Name)
            allSolidsLabels.append(groupMember.Label)
            allSolidsObjects.append(groupMember)

    return allSolidsNames, allSolidsLabels, allSolidsObjects
"""
#  -------------------------------------------------------------------------
"""def minMidMax3(x, y, z, minMidMax):
    Given three numbers, return the minimum, or the middle or the maximum one
        minimum: if minMidMax == 1
        middle:  if minMidMax == 2
        maximum: if minMidMax == 3
    
    x = abs(x)
    y = abs(y)
    z = abs(z)
    if x >= y:
        if y >= z:
            # x >= y >= z
            if minMidMax == 1:
                return z
            elif minMidMax == 2:
                return y
            else:
                return x
        elif x >= z:
            # x >= z > y
            if minMidMax == 1:
                return y
            elif minMidMax == 2:
                return z
            else:
                return x
        # z > x >= y
        elif minMidMax == 1:
            return y
        elif minMidMax == 2:
            return x
        else:
            return z
    # y > x
    elif x >= z:
        # y >= x >= z
        if minMidMax == 1:
            return z
        elif minMidMax == 2:
            return x
        else:
            return y
    # y > x and z > x
    elif y >= z:
        # y >= z > x
        if minMidMax == 1:
            return x
        elif minMidMax == 2:
            return z
        else:
            return y
    # z > y > x
    elif minMidMax == 1:
        return x
    elif minMidMax == 2:
        return y
    else:
        return z
    """
#  -------------------------------------------------------------------------
"""def minMidMaxVec(Vector, minMidMax):
    Given coordinates in a vector, return the minimum, or the middle or the maximum one
        minimum: if minMidMax == 1
        middle:  if minMidMax == 2
        maximum: if minMidMax == 3
    
    Vec = CAD.Vector(Vector)
    Vec.x = abs(Vec.x)
    Vec.y = abs(Vec.y)
    Vec.z = abs(Vec.z)
    if Vec.x >= Vec.y:
        if Vec.y >= Vec.z:
            # Vec.x >= Vec.y >= Vec.z
            if minMidMax == 1:
                return Vec.z
            elif minMidMax == 2:
                return Vec.y
            else:
                return Vec.x
        if Vec.x >= Vec.z:
            # Vec.x >= Vec.z > Vec.y
            if minMidMax == 1:
                return Vec.y
            elif minMidMax == 2:
                return Vec.z
            else:
                return Vec.x
        # Vec.z > Vec.x >= Vec.y
        if minMidMax == 1:
            return Vec.y
        elif minMidMax == 2:
            return Vec.x
        else:
            return Vec.z
    # y > x
    if Vec.x >= Vec.z:
        # Vec.y >= Vec.x >= Vec.z
        if minMidMax == 1:
            return Vec.z
        elif minMidMax == 2:
            return Vec.x
        else:
            return Vec.y
    # y > x and z > x
    if Vec.y >= Vec.z:
        # Vec.y >= Vec.z > Vec.x
        if minMidMax == 1:
            return Vec.x
        elif minMidMax == 2:
            return Vec.z
        else:
            return Vec.y
    # Vec.z > Vec.y > Vec.x
    if minMidMax == 1:
        return Vec.x
    elif minMidMax == 2:
        return Vec.y
    else:
        return Vec.z
    """
#  -------------------------------------------------------------------------
"""def getDictionaryOfBodyPoints():
    Run through the active document and return a dictionary of a dictionary of
    point names in each bodyObject 
    # i.e. {<body name> : {<point name> : <its index in the body's list of points>}}
    if Debug: Mess("SimToolsClass-getDictionaryOfBodyPoints")
    dictionaryOfBodyPoints = {}
    simGlobal = getsimGlobalObject()
    for groupMember in simGlobal.Group:
        if "SimBody" in groupMember.Name:
            PointDict = {}
            for index in range(len(groupMember.JointNameList)):
                PointDict[groupMember.JointNameList[index]] = index
            dictionaryOfBodyPoints[groupMember.Name] = PointDict.copy()

    return dictionaryOfBodyPoints
    """
#  -------------------------------------------------------------------------
