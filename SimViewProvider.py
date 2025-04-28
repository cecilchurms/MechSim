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
import SimTaskPanels as SP

from pivy import coin

# =============================================================================
class ViewProviderSimGlobalClass:
    """A view provider for the SimGlobal object"""
    #  -------------------------------------------------------------------------
    def __init__(self, simGlobalViewObject):
        simGlobalViewObject.Proxy = self
    #  -------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the simGlobal icon (Icon2n.png)"""

        return ST.getSimModulePath("icons", "Icon2n.png")
    #  -------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
    # =============================================================================
class ViewProviderSimSolverClass:
    #  -------------------------------------------------------------------------
    def __init__(self, solverViewObject):
        solverViewObject.Proxy = self
    #  -------------------------------------------------------------------------
    def doubleClicked(self, solverViewObject):
        """Open up the TaskPanel if it is not open"""

        Document = CADGui.getDocument(solverViewObject.Object.Document)
        if not Document.getInEdit():
            Document.setEdit(solverViewObject.Object.Name)
        return True
    #  -------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the solver icon (Icon7n.png)"""

        return ST.getSimModulePath("icons", "Icon7n.png")
    #  -------------------------------------------------------------------------
    def attach(self, solverViewObject):

        self.solverObject = solverViewObject.Object
        solverViewObject.addDisplayMode(coin.SoGroup(), "Standard")
    #  -------------------------------------------------------------------------
    def getDisplayModes(self, obj):
        """Return an empty list of modes when requested"""

        return []
    #  -------------------------------------------------------------------------
    def getDefaultDisplayMode(self):

        return "Flat Lines"
    #  -------------------------------------------------------------------------
    def setDisplayMode(self, mode):

        return mode
    #  -------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    #  -------------------------------------------------------------------------
    def setEdit(self, solverViewObject, mode):
        """Edit the parameters by switching on the task dialog"""

        CADGui.Control.showDialog(SP.TaskPanelSimSolverClass(self.solverObject))
        return True
    #  -------------------------------------------------------------------------
    def unsetEdit(self, viewobj, mode):
        """Shut down the task dialog"""
        CADGui.Control.closeDialog()
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
    # =============================================================================
class ViewProviderSimAnimateClass:
    """ A view provider for the SimAnimate simGlobal object """
    # -------------------------------------------------------------------------------------------------
    def __init__(self, animateViewObject):

        animateViewObject.Proxy = self
    # -------------------------------------------------------------------------------------------------
    #def doubleClicked(self, animateViewObject):
    #    """Open up the TaskPanel if it is not open"""
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

        return ST.getSimModulePath("icons", "Icon8n.png")
    # -------------------------------------------------------------------------------------------------
    def attach(self, animateViewObject):

        self.animateObject = animateViewObject.Object
        animateViewObject.addDisplayMode(coin.SoGroup(), "Standard")
    # -------------------------------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    # -------------------------------------------------------------------------------------------------
    def __getstate__(self):
        pass
    # -------------------------------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
# ==============================================================================
class ViewProviderSimMaterialClass:
    """Handle the screen interface stuff for the materials dialog"""
    #  -------------------------------------------------------------------------
    def __init__(self, materialViewObject):
        materialViewObject.Proxy = self
    #  -------------------------------------------------------------------------
    def doubleClicked(self, materialViewObject):
        """Open up the TaskPanel if it is not open"""
        Document = CADGui.getDocument(materialViewObject.Object.Document)
        if not Document.getInEdit():
            Document.setEdit(materialViewObject.Object.Name)
        return True
    #  -------------------------------------------------------------------------
    def getIcon(self):
        """Returns the full path to the material icon (Icon5n.png)"""
        return ST.getSimModulePath("icons", "Icon5n.png")
    #  -------------------------------------------------------------------------
    def attach(self, materialViewObject):
        self.materialObject = materialViewObject.Object
        materialViewObject.addDisplayMode(coin.SoGroup(), "Standard")
    #  -------------------------------------------------------------------------
    def getDisplayModes(self, materialObject):
        return []
    #  -------------------------------------------------------------------------
    def getDefaultDisplayMode(self):
        return "Flat Lines"
    #  -------------------------------------------------------------------------
    def setDisplayMode(self, mode):
        return mode
    #  -------------------------------------------------------------------------
    def updateData(self, obj, prop):
        return
    #  -------------------------------------------------------------------------
    def setEdit(self, materialViewObject, mode):
        """Edit the parameters by calling the task dialog"""
        CADGui.Control.showDialog(SP.TaskPanelSimMaterialClass(self.materialObject))
        return True
    #  -------------------------------------------------------------------------
    def unsetEdit(self, materialViewObject, mode):
        """Close the task dialog when we have finished using it"""
        CADGui.Control.closeDialog()
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
# =============================================================================
#class ViewProviderSimBodyClass:
#    """A class which handles all the gui overheads"""
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
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimBodyClass-__setstate__")
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
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimForceClass-__setstate__")
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
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("ViewProviderSimJointClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("ViewProviderSimJoint-__setstate__")
## =============================================================================
