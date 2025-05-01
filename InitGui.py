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
# *          Nikra-DAP thus underwent a major re-write and was enlarged          *
# *                   into the Mechanical Simulator: "MechSim"                   *
# *                                                                              *
# *              The initial stages of  Nikra-DAP  were supported by:            *
# *                 Engineering X, an international collaboration                *
# *                founded by the Royal Academy of Engineering and               *
# *                        Lloyd's Register Foundation.                          *
# *                                                                              *
# *                 An early version of Nikra-DAP was written by:                *
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
import FreeCAD
import FreeCADGui

global Debug

# =============================================================================
# InitGui.py
# 1. Connects the workbench to FreeCAD and FreeCADGui
# 2. Builds the graphical interface between the workbench and FreeCAD
# 3. Couples the graphical interface of FreeCAD with the functions of the workbench
# =============================================================================
class MechSim(Workbench):
    """This class encompasses the whole MechSim workbench"""
    #  -------------------------------------------------------------------------
    def __init__(self):
        """Called on startup of FreeCAD"""

        import SimTools

        # Set up the text for the Sim workbench option, the MechSim icon, and the tooltip
        self.__class__.Icon = SimTools.getSimModulePath("icons", "Icon1n.png")
        self.__class__.MenuText = "MechSim"
        self.__class__.ToolTip = "Mechanical objects simulator workbench based on Prof. Nikravesh's DAP solver"
    #  -------------------------------------------------------------------------
    def Initialize(self):
        """Called on the first selection of the MechSim Workbench
        and couples the main MechSim functions to the FreeCAD interface"""

        import SimCommands

        # Add the commands to FreeCAD's list of functions
        FreeCADGui.addCommand("SimGlobalAlias", SimCommands.CommandSimGlobalClass())
        FreeCADGui.addCommand("SimMaterialAlias", SimCommands.CommandSimMaterialClass())
        FreeCADGui.addCommand("SimSolverAlias", SimCommands.CommandSimSolverClass())
        FreeCADGui.addCommand("SimAnimationAlias", SimCommands.CommandSimAnimationClass())
        #FreeCADGui.addCommand("SimBodyAlias", SimCommands.CommandSimBodyClass())
        #FreeCADGui.addCommand("SimJointAlias", SimCommands.CommandSimJointClass())
        #FreeCADGui.addCommand("SimForceAlias", SimCommands.CommandSimForceClass())

        # Create a toolbar with the Sim commands (icons)
        self.appendToolbar("MechSim Commands", self.MakeCommandList())

        # Create a drop-down menu item for the menu bar
        self.appendMenu("MechSim", self.MakeCommandList())
    #  -------------------------------------------------------------------------
    def ContextMenu(self, recipient):
        """This is executed whenever the user right-clicks on screen
        'recipient'=='view' when mouse is in the VIEW window
        'recipient'=='tree' when mouse is in the TREE window
        We currently do no use either flag"""

        # Append the Sim commands to the existing context menu
        self.appendContextMenu("MechSim Commands", self.MakeCommandList())
    #  -------------------------------------------------------------------------
    def MakeCommandList(self):
        """Define a list of our aliases for all the Sim main functions"""

        return [
            "SimGlobalAlias",
            "SimMaterialAlias",
            "Separator",
            "SimSolverAlias",
            "Separator",
            "SimAnimationAlias"
        ]
    # "Separator",
    # "SimBodyAlias",
    # "SimForceAlias",
    # "SimJointAlias",
    # "Separator",
    #  -------------------------------------------------------------------------
    def Activated(self):
        """Called when the MechSim workbench is run"""
    #  -------------------------------------------------------------------------
    def Deactivated(self):
        """This function is executed each time the MechSim workbench is stopped"""
    #  -------------------------------------------------------------------------
    def GetClassName(self):
        """This function is mandatory if this is a full FreeCAD workbench
        The returned string should be exactly 'Gui::PythonWorkbench'
        This enables FreeCAD to ensure that the stuff in its 'Mod' folder
        is a valid workbench and not just rubbish"""

        return "Gui::PythonWorkbench"
    # --------------------------------------------------------------------------
    def __str__(self):
        return str(self.__dict__)
# =============================================================================
# Run when FreeCAD detects a workbench folder in its 'Mod' folder
# Add the workbench to the list of workbenches and initialize it
# =============================================================================
FreeCADGui.addWorkbench(MechSim())
