import FreeCAD as CAD
import FreeCADGui as CADGui

import SimTools as ST
import SimMakes as SM
import SimViewProvider as SV
import SimTaskPanels as SP

from PySide import QtCore

Debug = False
# =============================================================================
class CommandSimGlobalClass:
    """The Sim simGlobal command definition"""
    if Debug: ST.Mess("CommandSimGlobalClass-CLASS")
    #  -------------------------------------------------------------------------
    def GetResources(self):
        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
        Returns a dictionary defining the icon, the menu text and the tooltip"""
        if Debug: ST.Mess("CommandSimGlobalClass-GetResources")

        return {
            "Pixmap": ST.getSimModulePath("icons", "Icon2n.png"),
            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimGlobalAlias", "Add simGlobal"),
            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimGlobalAlias", "Creates a simGlobal for the Sim analysis data."),
        }
    #  -------------------------------------------------------------------------
    def IsActive(self):
        """Determine if the command/icon must be active or greyed out
        Only activate it if we have an Assembly model to use"""
        if Debug: ST.Mess("CommandSimGlobalClass-IsActive(query)")

        # Return True if we have an Assembly FreeCAD model document which is loaded and Active
        if CAD.ActiveDocument is None:
            CAD.Console.PrintErrorMessage("No active document is loaded into FreeCAD for MechSim to use")
            return False

        # Check that we at least have an Assembly Object
        Found = False
        for obj in CAD.ActiveDocument.Objects:
            if hasattr(obj, "Type") and obj.Type == 'Assembly':
                Found = True
                break
        if Found == False:
            CAD.Console.PrintErrorMessage("No Assembly Model found for MechSim to use")
            return False

        # Check that we don't already have a simGlobal object
        for obj in CAD.ActiveDocument.Objects:
            if hasattr(obj, "Name") and obj.Name == "SimGlobal":
                return False
        return True

    #  -------------------------------------------------------------------------
    def Activated(self):
        """Called when the create simGlobal command is run by either pressing
        the tool Icon, or running it from one of the available menus.
        We create the SimGlobal and set it to be Active"""
        if Debug: ST.Mess("CommandSimGlobalClass-Activated")

        # This is where we create a new empty Sim simGlobal
        SM.makeSimGlobal()

    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug: ST.Mess("TaskPanelSimGlobalClass-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug: ST.Mess("TaskPanelSimGlobalClass-__setstate__")
# =============================================================================
class CommandSimSolverClass:
    """The Sim Solver command definition"""
    if Debug: ST.Mess("CommandSimSolverClass-CLASS")
    #  -------------------------------------------------------------------------
    def GetResources(self):
        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
        Returns a dictionary defining the icon, the menu text and the tooltip"""
        if Debug: ST.Mess("CommandSimSolverClass-GetResources")

        return {
            "Pixmap": ST.getSimModulePath("icons", "Icon7n.png"),
            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimSolverAlias", "Run the analysis"),
            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimSolverAlias", "Run the analysis."),
        }
    #  -------------------------------------------------------------------------
    def IsActive(self):
        """Determine if the command/icon must be active or greyed out"""
        if Debug: ST.Mess("CommandSimSolverClass-IsActive(query)")

        return (ST.getsimGlobalObject() is not None)
    #  -------------------------------------------------------------------------
    def Activated(self):
        """Called when the Solver command is run"""
        if Debug: ST.Mess("CommandSimSolverClass-Activated")

        # Re-use any existing solver object
        for sObject in CAD.ActiveDocument.Objects:
            if "SimSolver" in sObject.Name:
                solverObject = sObject
                CADGui.ActiveDocument.setEdit(solverObject.Name)
                return

        # Make a new solver object
        SM.makeSimSolver("SimSolver")
        for sObject in CAD.ActiveDocument.Objects:
            if "SimSolver" in sObject.Name:
                solverObject = sObject
                CADGui.ActiveDocument.setEdit(solverObject.Name)
                return

        ST.Mess("Solver Object creation failed")

    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug: ST.Mess("TaskPanelSimSolverClass-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug: ST.Mess("TaskPanelSimSolverClass-__setstate__")
# =============================================================================
class CommandSimAnimationClass:
    """The Sim Animation command definition"""
    if Debug: ST.Mess("CommandSimAnimationClass-CLASS")

    #  -------------------------------------------------------------------------
    def GetResources(self):
        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
        Returns a dictionary defining the icon, the menu text and the tooltip"""
        if Debug: ST.Mess("CommandSimAnimationClass-GetResourcesC")

        return {
            "Pixmap": ST.getSimModulePath("icons", "Icon8n.png"),
            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimAnimationAlias", "Do Animation"),
            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimAnimationAlias", "Animates the motion of the bodies."),
        }

    #  -------------------------------------------------------------------------
    def IsActive(self):
        """Determine if there are already some results stored in the solver object
        i.e. Determine if the animate command/icon must be active or greyed out"""
        if Debug: ST.Mess("CommandSimAnimationClass-IsActive")

        return ST.getsimGlobalObject().SimResultsValid

    #  -------------------------------------------------------------------------
    def Activated(self):
        """Called when the Animation command is run"""
        if Debug: ST.Mess("CommandSimAnimationClass-Activated")

        # Get the identity of the Sim document (which is the active document on entry)
        self.SimDocument = CAD.ActiveDocument

        # Set an existing "Animation" document active or create it if it does not exist yet
        if "Animation" in CAD.listDocuments():
            CAD.setActiveDocument("Animation")
        else:
            CAD.newDocument("Animation")

        self.animationDocument = CAD.ActiveDocument

        # Add the ground object to the animation view (and forget about it)
        groundObj = self.SimDocument.findObjects(Name="^LinkGroup$")[0]
        animObj = self.animationDocument.addObject("Part::FeaturePython", "Ani_SimGround")
        animObj.Shape = groundObj.Shape
        SV.ViewProviderSimAnimateClass(animObj.ViewObject)

        # Generate the list of bodies to be animated and
        # create an object for each, with their shapes, in the animationDocument
        solverObj = self.SimDocument.findObjects(Name="^SimSolver$")[0]
        for bodyName in solverObj.BodyNames:
            bodyObj = self.SimDocument.findObjects(Name="^" + bodyName + "$")[0]

            animObj = self.animationDocument.addObject("Part::FeaturePython", ("Ani_" + bodyName))
            # Add the shape to the newly created object
            animObj.Shape = bodyObj.Shape
            # Instantiate the class to handle the Gui stuff
            SV.ViewProviderSimAnimateClass(animObj.ViewObject)

        # Request the animation window zoom to be set to fit the entire system
        CADGui.SendMsgToActiveView("ViewFit")

        # Edit the parameters by calling the task dialog
        taskd = SP.TaskPanelSimAnimateClass(
            self.SimDocument,
            self.animationDocument,
        )
        CADGui.Control.showDialog(taskd)

    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug: ST.Mess("CommandSimAnimClass-__getstate__")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug: ST.Mess("CommandSimAnimClass-__setstate__")
# ==============================================================================
#class CommandSimBodyClass:
#    """The Sim body command definition"""
#    if Debug:
#        ST.Mess("CommandSimBodyClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def GetResources(self):
#        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
#        Returns a dictionary defining the icon, the menu text and the tooltip"""
#        if Debug:
#            ST.Mess("CommandSimBodyClass-GetResources")
#        return {
#            "Pixmap": ST.getSimModulePath("icons", "Icon3n.png"),
#            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimBodyAlias", "Add Body"),
#            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimBodyAlias", "Creates and defines a body for the Sim analysis."), }
#    #  -------------------------------------------------------------------------
#    def IsActive(self):
#        """Determine if the command/icon must be active or greyed out
#        Only activate it when there is at least a simGlobal defined"""
#        if Debug:
#            ST.Mess("CommandSimBodyClass-IsActive(query)")
#        return False # ST.getsimGlobalObject() is not None
#    #  -------------------------------------------------------------------------
#    def Activated(self):
#        """Called when the Body Selection command is run"""
#        if Debug:
#            ST.Mess("CommandSimBodyClass-Activated")
#        # This is where we create a new empty Sim Body object
#        ST.getsimGlobalObject().addObject(SM.makeSimBody())
#        # Switch on the Sim Body Task Dialog
#        CADGui.ActiveDocument.setEdit(CAD.ActiveDocument.ActiveObject.Name)
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("CommandSimBody-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("CommandSimBodyClass-__setstate__")
# =============================================================================
#class CommandSimMaterialClass:
#    """The Sim Material command definition"""
#    if Debug:
#        ST.Mess("CommandSimMaterialClass-CLASS\n")
#    #  -------------------------------------------------------------------------
#    def GetResources(self):
#        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
#        Returns a dictionary defining the icon, the menu text and the tooltip"""
#        if Debug:
#            ST.Mess("CommandSimMaterialClass-GetResources\n")
#        return {
#            "Pixmap": ST.getSimModulePath("icons", "Icon5n.png"),
#            "MenuText": QtCore.QT_TRANSLATE_NOOP("Sim_Material_alias", "Add Material"),
#            "ToolTip": QtCore.QT_TRANSLATE_NOOP("Sim_Material_alias", "Define the material properties associated with each body part.")
#        }
#    #  -------------------------------------------------------------------------
#    def IsActive(self):
#        """Determine if the command/icon must be active or greyed out.
#        Only activate it when there is at least one body defined"""
#        if Debug:
#            ST.Mess("CommandSimMaterialClass-IsActive(query)\n")
#        return False # (len(ST.getDictionary("SimBody")) > 0) and (ST.getMaterialObject() is None)
#    #  -------------------------------------------------------------------------
#    def Activated(self):
#        """Called when the Material Selection command is run"""
#        if Debug:
#            ST.Mess("CommandSimMaterialClass-Activated\n")
#        # This is where we create a new empty Sim Material object
#        ST.getsimGlobalObject().addObject(SM.makeSimMaterial())
#        # Switch on the Sim Material Task Panel
#        CADGui.ActiveDocument.setEdit(CAD.ActiveDocument.ActiveObject.Name)
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-__getstate__\n")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-__setstate__\n")
# =============================================================================
#class CommandSimForceClass:
#    """The Sim Force command definition"""
#    if Debug:
#        ST.Mess("CommandSimForceClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def GetResources(self):
#        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
#        Returns a dictionary defining the icon, the menu text and the tooltip"""
#        if Debug:
#            ST.Mess("CommandSimForceClass-GetResources")
#        return {
#            "Pixmap": ST.getSimModulePath("icons", "Icon6n.png"),
#            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimForceAlias", "Add Force"),
#            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimForceAlias", "Creates and defines a force for the Sim analysis"),
#        }
#    #  -------------------------------------------------------------------------
#    def IsActive(self):
#        """Determine if the command/icon must be active or greyed out
#        Only activate it when there is at least one body defined"""
#        if Debug:
#            ST.Mess("CommandSimForceClass-IsActive(query)")
#        return False # len(ST.getDictionary("SimBody")) > 0
#    #  -------------------------------------------------------------------------
#    def Activated(self):
#        """Called when the Force Selection command is run"""
#        if Debug:
#            ST.Mess("CommandSimForceClass-Activated")
#        # This is where we create a new empty Sim Force object
#        ST.getsimGlobalObject().addObject(SM.makeSimForce())
#        # Switch on the Sim Force Task Dialog
#        CADGui.ActiveDocument.setEdit(CAD.ActiveDocument.ActiveObject.Name)
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-__setstate__")
# ==============================================================================
#class CommandSimJointClass:
#    """The Sim Joint command definition"""
#    if Debug:
#        ST.Mess("CommandSimJointClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def GetResources(self):
#        """Called by FreeCAD when 'CADGui.addCommand' is run in InitGui.py
#        Returns a dictionary defining the icon, the menu text and the tooltip"""
#        if Debug:
#            ST.Mess("CommandSimJointClass-GetResourcesC")
#        return {
#            "Pixmap": ST.getSimModulePath("icons", "Icon4n.png"),
#            "MenuText": QtCore.QT_TRANSLATE_NOOP("SimJointAlias", "Add Joint"),
#            "ToolTip": QtCore.QT_TRANSLATE_NOOP("SimJointAlias", "Creates and defines a joint for the Sim analysis."),
#        }
#    #  -------------------------------------------------------------------------
#    def IsActive(self):
#        """Determine if the command/icon must be active or greyed out.
#        Only activate it when there are at least two bodies defined"""
#        if Debug:
#            ST.Mess("CommandSimJointClass-IsActive(query)")
#        return False # len(ST.getDictionary("SimBody")) > 1
#    #  -------------------------------------------------------------------------
#    def Activated(self):
#        """Called when the Sim Joint command is run"""
#        if Debug:
#            ST.Mess("CommandSimJointClass-Activated")
#        # This is where we create a new empty Sim joint object
#        ST.getsimGlobalObject().addObject(SM.makeSimJoint())
#        # Switch on the Sim Joint Task Dialog
#        CADGui.ActiveDocument.setEdit(CAD.ActiveDocument.ActiveObject.Name)
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("CommandSimJointClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("CommandSimJointClass-__setstate__")
