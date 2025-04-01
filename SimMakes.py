import FreeCAD as CAD
import FreeCADGui as CADGui
import SimTools as ST
import SimClasses as SC
import SimViewProvider as SV

Debug = False
# =============================================================================
def makeSimGlobal(name="SimGlobal"):
    """Create Sim simGlobal FreeCAD group object"""
    if Debug: ST.Mess("makeSimGlobal")

    simGlobalObject = CAD.ActiveDocument.addObject("App::DocumentObjectGroupPython", name)
    # Instantiate a SimGlobal object
    SC.SimGlobalClass(simGlobalObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimGlobalClass(simGlobalObject.ViewObject)
    return simGlobalObject
# =============================================================================
def makeSimSolver(name="SimSolver"):
    """Create a Sim Solver object"""
    if Debug: ST.Mess("makeSimSolver")

    solverObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimSolver object
    SC.SimSolverClass(solverObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimSolverClass(solverObject.ViewObject)
    return solverObject

# =============================================================================
def makeSimForce(name="SimForce"):
    # Create an empty Sim Force Object
    if Debug: ST.Mess("makeSimForce")

    forceObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimForce object
    SC.SimForceClass(forceObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimForceClass(forceObject.ViewObject)
    return forceObject
# =============================================================================
"""def makeSimBody(name="SimBody"):
    # Create an empty Sim Body object
    if Debug: ST.Mess("makeSimBody")
    
    bodyObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimBody object
    DC.SimBodyClass(bodyObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimBodyClass(bodyObject.ViewObject)
    return bodyObject
    """
# =============================================================================
""" def makeSimMaterial(name="SimMaterial"):
    if Debug: ST.Mess("makeSimMaterial\n")
    
   materialObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimMaterial object
    DC.SimMaterialClass(materialObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimMaterialClass(materialObject.ViewObject)
    return materialObject
    """
# =============================================================================
"""def makeSimJoint(name="SimJoint"):
    # Create an empty Sim Joint Object
    if Debug: ST.Mess("makeSimJoint")
    
    jointObject = CAD.ActiveDocument.addObject("Part::FeaturePython", name)
    # Instantiate a SimJoint object
    DC.SimJointClass(jointObject)
    # Instantiate the class to handle the Gui stuff
    SV.ViewProviderSimJointClass(jointObject.ViewObject)
    return jointObject
    """
# =============================================================================
