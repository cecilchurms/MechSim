import FreeCAD
import FreeCADGui

global Debug
Debug = False
# =============================================================================
# InitGui.py
# 1. Connects the workbench to FreeCAD and FreeCADGui
# 2. Builds the graphical interface between the workbench and FreeCAD
# 3. Couples the graphical interface of FreeCAD with the functions of the workbench
# =============================================================================
class MechSim(Workbench):
    """This class encompasses the whole MechSim workbench"""
    if Debug: FreeCAD.Console.PrintMessage("MechSim WorkbenchClass-CLASS\n")
    #  -------------------------------------------------------------------------
    def __init__(self):
        """Called on startup of FreeCAD"""
        if Debug: FreeCAD.Console.PrintMessage("MechSim WorkbenchClass-__init__\n")

        import SimTools

        # Set up the text for the Sim workbench option, the MechSim icon, and the tooltip
        self.__class__.Icon = SimTools.getSimModulePath("icons", "Icon1n.png")
        self.__class__.MenuText = "MechSim"
        self.__class__.ToolTip = "Mechanical objects simulator workbench based on Prof. Nikravesh's DAP solver"
    #  -------------------------------------------------------------------------
    def Initialize(self):
        """Called on the first selection of the MechSim Workbench
        and couples the main MechSim functions to the FreeCAD interface"""
        if Debug: FreeCAD.Console.PrintMessage("MechSim WorkbenchClass-Initialize\n")

        import SimCommands

        # Add the commands to FreeCAD's list of functions
        FreeCADGui.addCommand("SimGlobalAlias", SimCommands.CommandSimGlobalClass())
        FreeCADGui.addCommand("SimSolverAlias", SimCommands.CommandSimSolverClass())
        FreeCADGui.addCommand("SimAnimationAlias", SimCommands.CommandSimAnimationClass())
        #FreeCADGui.addCommand("SimBodyAlias", SimCommands.CommandSimBodyClass())
        #FreeCADGui.addCommand("SimMaterialAlias", SimCommands.CommandSimMaterialClass())
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
        if Debug: FreeCAD.Console.PrintMessage("SimWorkbenchClass-ContextMenu\n")

        # Append the Sim commands to the existing context menu
        self.appendContextMenu("MechSim Commands", self.MakeCommandList())
    #  -------------------------------------------------------------------------
    def MakeCommandList(self):
        """Define a list of our aliases for all the Sim main functions"""
        if Debug: FreeCAD.Console.PrintMessage("SimWorkbenchClass-MakeCommandList\n")

        return [
            "SimGlobalAlias",
            "SimSolverAlias",
            "SimAnimationAlias"
        ]
    # "Separator",
    # "Separator",
    # "SimBodyAlias",
    # "SimMaterialAlias",
    # "SimForceAlias",
    # "SimJointAlias",
    # "Separator",
    #  -------------------------------------------------------------------------
    def Activated(self):
        """Called when the MechSim workbench is run"""
        if Debug: FreeCAD.Console.PrintMessage("SimWorkbenchClass-Activated\n")
    #  -------------------------------------------------------------------------
    def Deactivated(self):
        """This function is executed each time the MechSim workbench is stopped"""
        if Debug: FreeCAD.Console.PrintMessage("SimWorkbenchClass-Deactivated\n")
    #  -------------------------------------------------------------------------
    def GetClassName(self):
        """This function is mandatory if this is a full FreeCAD workbench
        The returned string should be exactly 'Gui::PythonWorkbench'
        This enables FreeCAD to ensure that the stuff in its 'Mod' folder
        is a valid workbench and not just rubbish"""
        if Debug: FreeCAD.Console.PrintMessage("SimWorkbenchClass-GetClassName\n")

        return "Gui::PythonWorkbench"
    # --------------------------------------------------------------------------
    def __str__(self):
        return str(self.__dict__)
# =============================================================================
# Run when FreeCAD detects a workbench folder in its 'Mod' folder
# Add the workbench to the list of workbenches and initialize it
# =============================================================================
FreeCADGui.addWorkbench(MechSim())
