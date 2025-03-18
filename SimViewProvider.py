import FreeCAD as CAD
import FreeCADGui as CADGui
import SimTools as ST
import SimTaskPanels as SP

from pivy import coin

Debug = False
# =============================================================================
class ViewProviderSimContainerClass:
    """A view provider for the SimContainer container object"""
    if Debug: ST.Mess("ViewProviderSimContainerClass-CLASS")
    #  -------------------------------------------------------------------------
    def __init__(self, containerViewObject):
        if Debug: ST.Mess("ViewProviderSimContainerClass-__init__")
        containerViewObject.Proxy = self
    #  -------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the container icon (Icon2n.png)"""
        if Debug: ST.Mess("ViewProviderSimContainer-getIcon")

        return ST.getSimModulePath("icons", "Icon2n.png")
    #  -------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    #  -------------------------------------------------------------------------
    def __load__(self):
        if Debug: ST.Mess("TaskPanelSimContainerClass-__load__")
        return self.Type
    #  -------------------------------------------------------------------------
    def __dump__(self, state):
        if Debug: ST.Mess("TaskPanelSimContainerClass-__dump__")
        if state: self.Type = state
# =============================================================================
class ViewProviderSimSolverClass:
    if Debug:
        ST.Mess("ViewProviderSimSolverClass-CLASS")
    #  -------------------------------------------------------------------------
    def __init__(self, solverViewObject):
        if Debug: ST.Mess("ViewProviderSimSolverClass-__init__")

        solverViewObject.Proxy = self
    #  -------------------------------------------------------------------------
    def doubleClicked(self, solverViewObject):
        """Open up the TaskPanel if it is not open"""
        if Debug: ST.Mess("ViewProviderSimSolverClass-doubleClicked")

        Document = CADGui.getDocument(solverViewObject.Object.Document)
        if not Document.getInEdit():
            Document.setEdit(solverViewObject.Object.Name)
        return True
    #  -------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the solver icon (Icon7n.png)"""
        if Debug: ST.Mess("ViewProviderSimSolverClass-getIcon")

        return ST.getSimModulePath("icons", "Icon7n.png")
    #  -------------------------------------------------------------------------
    def attach(self, solverViewObject):
        if Debug: ST.Mess("ViewProviderSimSolverClass-attach")

        self.solverObject = solverViewObject.Object
        solverViewObject.addDisplayMode(coin.SoGroup(), "Standard")
    #  -------------------------------------------------------------------------
    def getDisplayModes(self, obj):
        """Return an empty list of modes when requested"""
        if Debug: ST.Mess("ViewProviderSimSolverClass-getDisplayModes")

        return []
    #  -------------------------------------------------------------------------
    def getDefaultDisplayMode(self):
        if Debug: ST.Mess("ViewProviderSimSolverClass-getDefaultDisplayMode")

        return "Flat Lines"
    #  -------------------------------------------------------------------------
    def setDisplayMode(self, mode):
        if Debug: ST.Mess("ViewProviderSimSolverClass-setDisplayMode")

        return mode
    #  -------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    #  -------------------------------------------------------------------------
    def setEdit(self, solverViewObject, mode):
        """Edit the parameters by switching on the task dialog"""
        if Debug: ST.Mess("ViewProviderSimSolverClass-setEdit")

        CADGui.Control.showDialog(SP.TaskPanelSimSolverClass(self.solverObject))
        return True
    #  -------------------------------------------------------------------------
    def unsetEdit(self, viewobj, mode):
        """Shut down the task dialog"""
        if Debug: ST.Mess("ViewProviderSimSolverClass-unsetEdit")

        CADGui.Control.closeDialog()
    #  -------------------------------------------------------------------------
    def __load__(self):
        if Debug: ST.Mess("ViewProviderSimSolverClass-__load__")
        return self.Type
    #  -------------------------------------------------------------------------
    def __dump__(self, state):
        if Debug: ST.Mess("ViewProviderSimSolverClass-__dump__")
        if state: self.Type = state
# =============================================================================
class ViewProviderSimAnimateClass:
    """ A view provider for the SimAnimate container object """
    # -------------------------------------------------------------------------------------------------
    def __init__(self, animateViewObject):
        if Debug: ST.Mess("ViewProviderSimAnimateClass-__init__")

        animateViewObject.Proxy = self
    # -------------------------------------------------------------------------------------------------
    #def doubleClicked(self, animateViewObject):
    #    """Open up the TaskPanel if it is not open"""
    #    if Debug: ST.Mess("ViewProviderSimAnimateClass-doubleClicked")
    #
    #    if not ST.getActiveAnalysis() == self.animateObject:
    #        if CADGui.activeWorkbench().name() != 'SimWorkbench':
    #            CADGui.activateWorkbench("SimWorkbench")
    #        ST.setActiveAnalysis(self.animateObject)
    #        return True
    #    return True
    # -------------------------------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the animation icon (Icon8n.png)"""
        if Debug: ST.Mess("ViewProviderSimAnimateClass-getIcon")

        return ST.getSimModulePath("icons", "Icon8n.png")
    # -------------------------------------------------------------------------------------------------
    def attach(self, animateViewObject):
        if Debug: ST.Mess("ViewProviderSimAnimateClass-attach")

        self.animateObject = animateViewObject.Object
        animateViewObject.addDisplayMode(coin.SoGroup(), "Standard")
    # -------------------------------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    # -------------------------------------------------------------------------------------------------
    def __load__(self):
        return self.Type
    # -------------------------------------------------------------------------------------------------
    def __dump__(self, state):
        if state: self.Type = state
# ==============================================================================
#class ViewProviderSimBodyClass:
#    """A class which handles all the gui overheads"""
#    if Debug:
#        ST.Mess("ViewProviderSimBodyClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, bodyViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-__init__")
#        bodyViewObject.Proxy = self
#    #  -------------------------------------------------------------------------
#    def doubleClicked(self, bodyViewObject):
#        """Open up the TaskPanel if it is not open"""
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-doubleClicked")
#        Document = CADGui.getDocument(bodyViewObject.Object.Document)
#        if not Document.getInEdit():
#            Document.setEdit(bodyViewObject.Object.Name)
#        return True
#    #  -------------------------------------------------------------------------
#    def getIcon(self):
#        """Returns the full path to the body icon (Icon3n.png)"""
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-getIcon")
#        return ST.getSimModulePath("icons", "Icon3n.png")
#    #  -------------------------------------------------------------------------
#    def attach(self, bodyViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-attach")
#        self.bodyObject = bodyViewObject.Object
#        bodyViewObject.addDisplayMode(coin.SoGroup(), "Standard")
#    #  -------------------------------------------------------------------------
#    def getDisplayModes(self, bodyViewObject):
#        """Return an empty list of modes when requested"""
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-getDisplayModes")
#        return []
#    #  -------------------------------------------------------------------------
#    def getDefaultDisplayMode(self):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-getDefaultDisplayMode")
#        return "Flat Lines"
#    #  -------------------------------------------------------------------------
#    def setDisplayMode(self, mode):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-setDisplayMode")
#        return mode
#    #  -------------------------------------------------------------------------
#    def updateData(self, obj, prop):
#        return
#    #  -------------------------------------------------------------------------
#    def setEdit(self, bodyViewObject, mode):
#        """Edit the parameters by calling the task dialog"""
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-setEdit")
#        CADGui.Control.showDialog(SP.TaskPanelSimBodyClass(bodyViewObject.Object))
#        return True
#    #  -------------------------------------------------------------------------
#    def unsetEdit(self, bodyViewObject, mode):
#        """We have finished with the task dialog so close it"""
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-unsetEdit")
#        CADGui.Control.closeDialog()
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-__dump__")
#        if state:
#            self.Type = state
## =============================================================================
#class ViewProviderSimMaterialClass:
#    """Handle the screen interface stuff for the materials dialog"""
#    if Debug:
#        CAD.Console.PrintMessage("ViewProviderSimMaterialClass-CLASS\n")
#    #  -------------------------------------------------------------------------
#    def __init__(self, materialViewObject):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-__init__\n")
#        materialViewObject.Proxy = self
#    #  -------------------------------------------------------------------------
#    def doubleClicked(self, materialViewObject):
#        """Open up the TaskPanel if it is not open"""
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-doubleClicked\n")
#        Document = CADGui.getDocument(materialViewObject.Object.Document)
#        if not Document.getInEdit():
#            Document.setEdit(materialViewObject.Object.Name)
#        return True
#    #  -------------------------------------------------------------------------
#    def getIcon(self):
#        """Returns the full path to the material icon (Icon5n.png)"""
#        if Debug:
#            ST.Mess("ViewProviderSimMaterialClass-getIcon")
#        return ST.getSimModulePath("icons", "Icon5n.png")
#    #  -------------------------------------------------------------------------
#    def attach(self, materialViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimMaterialClass-attach")
#        self.materialObject = materialViewObject.Object
#        materialViewObject.addDisplayMode(coin.SoGroup(), "Standard")
#    #  -------------------------------------------------------------------------
#    def getDisplayModes(self, materialObject):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-getDisplayModes\n")
#        return []
#    #  -------------------------------------------------------------------------
#    def getDefaultDisplayMode(self):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-getDefaultDisplayMode\n")
#        return "Flat Lines"
#    #  -------------------------------------------------------------------------
#    def setDisplayMode(self, mode):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-setDisplayMode\n")
#        return mode
#    #  -------------------------------------------------------------------------
#    def updateData(self, obj, prop):
#        return
#    #  -------------------------------------------------------------------------
#    def setEdit(self, materialViewObject, mode):
#        """Edit the parameters by calling the task dialog"""
#        if Debug:
#            ST.Mess("ViewProviderSimMaterialClass-setEdit")
#        CADGui.Control.showDialog(SP.TaskPanelSimMaterialClass(self.materialObject))
#        return True
#    #  -------------------------------------------------------------------------
#    def unsetEdit(self, materialViewObject, mode):
#        """Close the task dialog when we have finished using it"""
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-unsetEdit\n")
#        CADGui.Control.closeDialog()
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-__load__\n")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            CAD.Console.PrintMessage("ViewProviderSimMaterialClass-__dump__\n")
#        if state:
#            self.Type = state
## =============================================================================
#class ViewProviderSimForceClass:
#    if Debug:
#        ST.Mess("ViewProviderSimForceClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, forceViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-__init__")
#        forceViewObject.Proxy = self
#    #  -------------------------------------------------------------------------
#    def doubleClicked(self, forceViewObject):
#        """Open up the TaskPanel if it is not open"""
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-doubleClicked")
#        Document = CADGui.getDocument(forceViewObject.Object.Document)
#        if not Document.getInEdit():
#            Document.setEdit(forceViewObject.Object.Name)
#        return True
#    #  -------------------------------------------------------------------------
#    def getIcon(self):
#        """Returns the full path to the force icon (Icon6n.png)"""
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-getIcon")
#        return ST.getSimModulePath("icons", "Icon6n.png")
#    #  -------------------------------------------------------------------------
#    def attach(self, forceViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-attach")
#        self.forceObject = forceViewObject.Object
#        forceViewObject.addDisplayMode(coin.SoGroup(), "Standard")
#    #  -------------------------------------------------------------------------
#    def getDisplayModes(self, forceViewObject):
#        """Return an empty list of modes when requested"""
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-getDisplayModes")
#        return []
#    #  -------------------------------------------------------------------------
#    def getDefaultDisplayMode(self):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-getDefaultDisplayMode")
#        return "Flat Lines"
#    #  -------------------------------------------------------------------------
#    def setDisplayMode(self, mode):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-setDisplayMode")
#        return mode
#    #  -------------------------------------------------------------------------
#    def updateData(self, obj, prop):
#        return
#    #  -------------------------------------------------------------------------
#    def setEdit(self, forceViewObject, mode):
#        """Edit the parameters by calling the task dialog"""
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-setEdit")
#        CADGui.Control.showDialog(SP.TaskPanelSimForceClass(self.forceObject))
#        return True
#    #  -------------------------------------------------------------------------
#    def unsetEdit(self, forceViewObject, mode):
#        """Terminate the editing via the task dialog"""
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-unsetEdit")
#        CADGui.Control.closeDialog()
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-__dump__")
#        if state:
#            self.Type = state
##  =============================================================================
#class ViewProviderSimJointClass:
#    if Debug:
#        ST.Mess("ViewProviderSimJointClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, jointViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-__init__")
#        jointViewObject.Proxy = self
#        self.jointObject = jointViewObject.Object
#    #  -------------------------------------------------------------------------
#    def doubleClicked(self, jointViewObject):
#        """Open up the TaskPanel if it is not open"""
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-doubleClicked")
#        Document = CADGui.getDocument(jointViewObject.Object.Document)
#        if not Document.getInEdit():
#            Document.setEdit(jointViewObject.Object.Name)
#        return True
#    #  -------------------------------------------------------------------------
#    def getIcon(self):
#        """Returns the full path to the joint icon (Icon4n.png)"""
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-getIcon")
#        return ST.getSimModulePath("icons", "Icon4n.png")
#    #  -------------------------------------------------------------------------
#    def attach(self, jointViewObject):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-attach")
#        self.jointObject = jointViewObject.Object
#        jointViewObject.addDisplayMode(coin.SoGroup(), "Standard")
#    #  -------------------------------------------------------------------------
#    def getDisplayModes(self, jointViewObject):
#        """Return an empty list of modes when requested"""
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-getDisplayModes")
#        return []
#    #  -------------------------------------------------------------------------
#    def getDefaultDisplayMode(self):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-getDefaultDisplayMode")
#        return "Flat Lines"
#    #  -------------------------------------------------------------------------
#    def setDisplayMode(self, mode):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-setDisplayMode")
#        return mode
#    #  -------------------------------------------------------------------------
#    def updateData(self, obj, prop):
#        return
#    #  -------------------------------------------------------------------------
#    def setEdit(self, jointViewObject, mode):
#        """Edit the parameters by calling the task dialog"""
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-setEdit")
#        CADGui.Control.showDialog(SP.TaskPanelSimJointClass(self.jointObject))
#        return True
#    #  -------------------------------------------------------------------------
#    def unsetEdit(self, jointViewObject, mode):
#        """We have finished with the task dialog so close it"""
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-unsetEdit")
#        CADGui.Control.closeDialog()
#    #  -------------------------------------------------------------------------
#    def __load__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-__load__")
#        return self.Type
#    #  -------------------------------------------------------------------------
#    def __dump__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimJoint-__dump__")
#        if state:
#            self.Type = state
## =============================================================================
