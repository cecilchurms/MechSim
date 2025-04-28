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
import SimMain

from PySide import QtGui, QtCore
import Part
import math
import numpy as np
from os import path, getcwd

Debug = False
# =============================================================================
class TaskPanelSimSolverClass:
    """Taskpanel for Executing Sim Solver User Interface"""
    #  -------------------------------------------------------------------------
    def __init__(self, solverTaskObject):
        """Run on first instantiation of a TaskPanelSimSolver class"""

        self.solverTaskObject = solverTaskObject
        solverTaskObject.Proxy = self

        # Get the directory name to store results in
        if solverTaskObject.Directory == "":
            solverTaskObject.Directory = getcwd()

        # Load the taskDialog form information
        ui_path = path.join(path.dirname(__file__), "TaskPanelSimSolver.ui")
        self.form = CADGui.PySideUic.loadUi(ui_path)

        # Set up actions on the solver button
        self.form.solveButton.clicked.connect(self.solveButtonClicked_Callback)

        # Set the time in the form
        self.form.endTime.setValue(self.solverTaskObject.TimeLength)
        self.form.reportingTime.setValue(self.solverTaskObject.DeltaTime)

        # Set the file name and directory
        self.form.checkRK45.stateChanged.connect(self.solverType_Callback())
        self.form.checkRK23.stateChanged.connect(self.solverType_Callback())
        self.form.checkDOP853.stateChanged.connect(self.solverType_Callback())
        self.form.outputDirectory.setText(self.solverTaskObject.Directory)
        self.form.outputFileName.setText(self.solverTaskObject.FileName)
        self.form.browseFileDirectory.clicked.connect(self.getFolderDirectory_Callback)

        # Set the accuracy in the form
        self.form.Accuracy.setValue(self.solverTaskObject.Accuracy)

        # Default is to output only the animation data
        self.form.outputAnimOnly.toggled.connect(self.outputAnimOnlyCheckboxChanged_Callback)
        self.form.outputAnimOnly.setEnabled(True)
        self.form.outputAnimOnly.setChecked(True)

    #  -------------------------------------------------------------------------
    def accept(self):
        """Run when we press the OK button"""

        self.solverTaskObject.DeltaTime = self.form.reportingTime.value()
        self.solverTaskObject.TimeLength = self.form.endTime.value()
        self.solverTaskObject.FileName = self.form.outputFileName.text()
        self.solverTaskObject.Accuracy = self.form.Accuracy.value()
        self.solverType_Callback()

        # Close the dialog
        Document = CADGui.getDocument(self.solverTaskObject.Document)
        Document.resetEdit()

    #  -------------------------------------------------------------------------
    def outputAnimOnlyCheckboxChanged_Callback(self):
        if self.form.outputAnimOnly.isChecked():
            self.form.outputFileLabel.setDisabled(True)
            self.form.outputFileName.setDisabled(True)
            self.form.browseFileDirectory.setDisabled(True)
            self.form.outputDirectoryLabel.setDisabled(True)
            self.form.outputDirectory.setDisabled(True)
        else:
            self.form.outputFileLabel.setEnabled(True)
            self.form.outputFileName.setEnabled(True)
            self.form.browseFileDirectory.setEnabled(True)
            self.form.outputDirectoryLabel.setEnabled(True)
            self.form.outputDirectory.setEnabled(True)
            #self.solverTaskObject.Accuracy = 9
            self.form.Accuracy.setValue(self.solverTaskObject.Accuracy)
    #  -------------------------------------------------------------------------
    def solveButtonClicked_Callback(self):
        """Call the MainSolve() method in the SimMainC class"""

        self.solverType_Callback()
        # Change the solve button to red with 'Solving' on it
        self.form.solveButton.setDisabled(True)
        self.form.solveButton.setText("Solving")

        # Do some arithmetic to allow the repaint to happen
        # before the frame becomes unresponsive due to the big maths
        self.form.solveButton.repaint()
        self.form.solveButton.update()

        # Do a Delay to give the colour time
        t = 0.0
        for f in range(1000000):
            t += f/10.0

        self.form.solveButton.repaint()
        self.form.solveButton.update()

        # Pass the parameters in the form to the Main Solver
        self.solverTaskObject.Directory = self.form.outputDirectory.text()
        if self.form.outputAnimOnly.isChecked():
            self.solverTaskObject.FileName = "-"
        else:
            self.solverTaskObject.FileName = self.form.outputFileName.text()

        self.solverTaskObject.TimeLength = self.form.endTime.value()
        self.solverTaskObject.DeltaTime = self.form.reportingTime.value()
        self.solverTaskObject.Accuracy = self.form.Accuracy.value()

        # Instantiate the SimMainC class and run the solver
        self.SimMainC_Instance = SimMain.SimMainC(self.solverTaskObject.TimeLength,
                                                     self.solverTaskObject.DeltaTime,
                                                     self.solverTaskObject.Accuracy,
                                                     self.form.correctInitial.isChecked())
        if self.SimMainC_Instance.initialised is True:
            self.SimMainC_Instance.MainSolve()

        # Return the solve button to green with 'Solve' on it
        self.form.solveButton.setText("Solve")
        self.form.solveButton.setEnabled(True)

        # Close the dialog
        Document = CADGui.getDocument(self.solverTaskObject.Document)
        Document.resetEdit()

    #  -------------------------------------------------------------------------
    def getFolderDirectory_Callback(self):
        """Request the directory where the .csv result files will be written"""

        self.solverTaskObject.Directory = QtGui.QFileDialog.getExistingDirectory()
        self.form.outputDirectory.setText(self.solverTaskObject.Directory)
    #  -------------------------------------------------------------------------
    def solverType_Callback(self):
        """Change the LAPACK solver type"""

        if self.form.checkRK45.isChecked():
            self.solverTaskObject.SolverType = "RK45"
        elif self.form.checkRK23.isChecked():
            self.solverTaskObject.SolverType = "RK23"
        elif self.form.checkDOP853.isChecked():
            self.solverTaskObject.SolverType = "DOP853"

    #  -------------------------------------------------------------------------
    def solverTypeRK23_Callback(self):
        """Change the LAPACK solver type"""

        if self.form.checkRK23.isChecked():
            self.form.checkRK45.setChecked(False)
            self.form.checkRK23.setChecked(True)
            self.form.checkDOP853.setChecked(False)
            self.solverTaskObject.SolverType = "RK23"

    #  -------------------------------------------------------------------------
    def solverTypeDOP853_Callback(self):
        """Change the LAPACK solver type"""

        if self.form.checkDOP853.isChecked():
            self.form.checkRK45.setChecked(False)
            self.form.checkRK23.setChecked(False)
            self.form.checkDOP853.setChecked(True)
            self.solverTaskObject.SolverType = "DOP853"

    #  -------------------------------------------------------------------------
    def getStandardButtons(self):
        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""

        return QtGui.QDialogButtonBox.Close
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
# ==============================================================================
class TaskPanelSimAnimateClass:
    """Taskpanel for Running an animation"""
    #  -------------------------------------------------------------------------
    def __init__(self, SimDocument, animationDocument):
        """Run on first instantiation of a TaskPanelSimAnimate class"""

        # Transfer the called parameters to the instance variables
        for solver in SimDocument.Objects:
            if hasattr(solver, "Name") and solver.Name == "SimSolver":
                self.solverObj = solver
                break

        self.SimDocument = SimDocument
        self.animationDocument = animationDocument

        # Here we get the list of objects from the animation document
        self.animationBodyObjects = CAD.ActiveDocument.Objects

        # Set play back period to mid-range
        self.playBackPeriod = 100  # msec

        # Load the Sim Animate ui form
        ui_Path = path.join(path.dirname(__file__), "TaskPanelSimAnimate.ui")
        self.form = CADGui.PySideUic.loadUi(ui_Path)

        # Define callback functions when changes are made in the dialog
        self.form.horizontalSlider.valueChanged.connect(self.moveObjects_Callback)
        self.form.startButton.clicked.connect(self.playStart_Callback)
        self.form.stopButton.clicked.connect(self.stopStop_Callback)
        self.form.playSpeed.valueChanged.connect(self.changePlaySpeed_Callback)

        # Fetch the animation object for all the bodies and place in a list
        self.animationBodyObj = []
        for animationBodyName in self.solverObj.BodyNames:
            self.animationBodyObj.append(self.animationDocument.findObjects(Name="^Ani_"+animationBodyName+"$")[0])

        # Load the calculated values of positions/angles from the results file
        self.Positions = np.loadtxt(path.join(self.solverObj.Directory, "SimAnimation.csv"))
        self.nTimeSteps = len(self.Positions.T[0])
        if Debug:
            ST.Mess("Positions")
            ST.PrintNp2D(self.Positions)

        # Positions matrix is:
        # timeValue : body1X body1Y body1phi : body2X body2Y body2phi : ...
        # next time tick

        # Shift all the values relative to the starting position of each body
        startTick = self.Positions[0, :]
        self.startX = []
        self.startY = []
        self.startPhi = []
        for aniBody in range(1, len(self.solverObj.BodyNames)):
            self.startX.append(startTick[aniBody * 3 - 2])
            self.startY.append(startTick[aniBody * 3 - 1])
            self.startPhi.append(startTick[aniBody * 3])
        for tick in range(self.nTimeSteps):
            thisTick = self.Positions[tick, :]
            for aniBody in range(1, len(self.solverObj.BodyNames)):
                thisTick[aniBody * 3 - 2 ] -= self.startX[aniBody-1]
                thisTick[aniBody * 3 - 1 ] -= self.startY[aniBody-1]
                thisTick[aniBody * 3 ] -= self.startPhi[aniBody-1]
            self.Positions[tick, :] = thisTick
        ST.Mess("Diff Positions")
        ST.PrintNp2D(self.Positions)

        # Set up the timer parameters
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.playBackPeriod)
        self.timer.timeout.connect(self.onTimerTimeout_Callback)  # callback function after each tick

        # Set up the values displayed on the dialog
        self.form.horizontalSlider.setRange(0, self.nTimeSteps - 1)
        self.form.timeStepLabel.setText("0.000s of {0:5.3f}s".format(self.solverObj.TimeLength))
    #  -------------------------------------------------------------------------
    def reject(self):
        """Run when we press the Close button
        Closes document and sets the active document
        back to the solver document"""

        CADGui.Control.closeDialog()
        CAD.setActiveDocument(self.SimDocument.Name)
        CAD.closeDocument(self.animationDocument.Name)
    #  -------------------------------------------------------------------------
    def playStart_Callback(self):
        """Start the Qt timer when the play button is pressed"""

        self.timer.start()
    #  -------------------------------------------------------------------------
    def stopStop_Callback(self):
        """Stop the Qt timer when the stop button is pressed"""

        self.timer.stop()
    #  -------------------------------------------------------------------------
    def onTimerTimeout_Callback(self):
        """Increment the tick position in the player, looping, if requested"""

        tickPosition = self.form.horizontalSlider.value()
        tickPosition += 1
        if tickPosition >= self.nTimeSteps:
            if self.form.loopCheckBox.isChecked():
                tickPosition: int = 0
            else:
                self.timer.stop()

        # Update the slider in the dialog
        self.form.horizontalSlider.setValue(tickPosition)
    #  -------------------------------------------------------------------------
    def changePlaySpeed_Callback(self, newSpeed):
        """Alter the playback period by a factor of 1/newSpeed"""

        self.timer.setInterval(self.playBackPeriod * (1.0 / newSpeed))
    #  -------------------------------------------------------------------------
    def moveObjects_Callback(self, tick):
        """Move all the bodies to their pose at this clock tick"""

        self.form.timeStepLabel.setText(
            "{0:5.3f}s of {1:5.3f}s".format(
                tick * self.solverObj.DeltaTime,
                self.solverObj.TimeLength
            )
        )

        thisTick = self.Positions[tick, :]
        for aniBody in range(1, len(self.solverObj.BodyNames)):
            X = thisTick[aniBody * 3 - 2]
            Y = thisTick[aniBody * 3 - 1]
            Phi = thisTick[aniBody * 3 - 0]
            if Debug:
                ST.Mess("X: "+str(X)+" Y: "+str(Y)+" Phi: "+str(Phi))

            self.animationBodyObj[aniBody].Placement = \
                CAD.Placement(CAD.Vector(X, Y, 0.0),
                CAD.Rotation(CAD.Vector(0.0, 0.0, 1.0),Phi/math.pi*180.0),
                CAD.Vector(self.startX[aniBody-1], self.startY[aniBody-1], 0.0))
    #  -------------------------------------------------------------------------
    def getStandardButtons(self):
        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""
        return QtGui.QDialogButtonBox.Close
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        pass
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        pass
# ==============================================================================
class TaskPanelSimMaterialClass:
    """Task panel for adding a Material for each solid Part"""
    if Debug:
        CAD.Console.PrintMessage("TaskPanelSimMaterialClass-CLASS\n")
    #  -------------------------------------------------------------------------
    def __init__(self, materialTaskObject):
        """Run on first instantiation of a TaskPanelSimMaterial class"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimMaterialClass-__init__\n")

        self.materialTaskObject = materialTaskObject
        materialTaskObject.Proxy = self

        # Get the materials data from the materials library
        cardID2cardData, cardID2cardName, DummyDict = cardutils.import_materials()

        # Set up a record for the default density value at the beginning of the density dictionary
        self.densityDict = {'Default': 1000.0}

        # Add all the cards to the densityDict except for specialised steels
        for materialID in sorted(cardID2cardData.keys()):
            # If the density option is 'None' set the density to a very small but non-zero value
            if cardID2cardName[materialID] == "None":
                self.densityDict[cardID2cardName[materialID]] = 0.000000001
            # We want to ignore all the gazillion types of steel - except the generic one
            elif 'Steel' not in cardID2cardName[materialID] or cardID2cardName[materialID] == 'Steel-Generic':
                # The density values are in various number formats on the cards, so
                # Get the density string from the card, and filter out the non-numeric characters
                # This is fancy python - don't alter at all if you don't understand it
                # Python Syntax:  f(x) if condition else g(x) for x in sequence
                densityStr = cardID2cardData[materialID]['Density'][0:-3]
                density = ''.join(x for x in densityStr if x.isdigit() or x in ['.', '-', ','])
                densityNoComma = ''.join(x if x.isdigit() or x in ['.', '-'] else '.' for x in density)
                self.densityDict[cardID2cardName[materialID]] = float(str(densityNoComma))

        # Last thing, add a custom density card at the end of the list
        self.densityDict['Custom'] = 1000

        # Get a list of all the names (and labels) of all the Solid parts
        self.modelSolidsNamesList, self.modelSolidsLabelsList, DummyList = ST.getAllSolidsLists()
        self.modelMaterialsNamesList = []
        self.modelMaterialsDensitiesList = []

        # Create a density entry for all the solid names in the model
        for index in range(len(self.modelSolidsNamesList)):
            # Search for the material for this solid in any pre-existing list of solids
            for preIndex in range(len(materialTaskObject.solidsNameList)):
                if self.modelSolidsNamesList[index] == materialTaskObject.solidsNameList[preIndex]:
                    self.modelMaterialsNamesList.append(materialTaskObject.materialsNameList[preIndex])
                    self.modelMaterialsDensitiesList.append(materialTaskObject.materialsDensityList[preIndex])
                    break
            else:
                # If we don't find it, then create a default material at this index
                self.modelMaterialsNamesList.append("Default")
                self.modelMaterialsDensitiesList.append(1000.0)

        # Set up the task dialog
        ui_path = path.join(path.dirname(__file__), "TaskPanelSimMaterials.ui")
        self.form = CADGui.PySideUic.loadUi(ui_path)

        # Set up kg/m3 according to the material object value
        if materialTaskObject.kgm3ORgcm3 is True:
            self.form.kgm3.setChecked(True)
        else:
            self.form.gcm3.setChecked(True)

        # Set up the table for the densities in the task dialog
        self.form.tableWidget.clearContents()
        self.form.tableWidget.setRowCount(0)
        self.form.tableWidget.horizontalHeader().setResizeMode(QtGui.QHeaderView.ResizeToContents)

        # Display a density table of all the solid parts in the dialog form
        for tableIndex in range(len(self.modelSolidsNamesList)):
            self.form.tableWidget.insertRow(tableIndex)
            # --------------------------------------
            # Column 0 -- Name of solid in the model
            # Add the solid LABEL to the form, not the solid NAME
            partName = QtGui.QTableWidgetItem(self.modelSolidsLabelsList[tableIndex])
            partName.setFlags(QtCore.Qt.ItemIsEnabled)
            self.form.tableWidget.setItem(tableIndex, 0, partName)

            # ----------------------------------
            # Column 1 - Material type selection
            # Create a separate new combobox of all material types in column 1 of each solid Part
            combo = QtGui.QComboBox()
            # If we don't find an entry, then the material will be default (item 0 in list)
            dictIndex = 0
            comboIndex = 0
            for materName in self.densityDict:
                # First add the material name to the list in the combo box
                combo.addItem(materName)
                # Now check if this one is the currently selected density
                if materName == self.modelMaterialsNamesList[tableIndex]:
                    comboIndex = dictIndex
                dictIndex += 1
            # Save the combo in the table cell and set the current index
            self.form.tableWidget.setCellWidget(tableIndex, 1, combo)
            combo.setCurrentIndex(comboIndex)

            # Set the connect function for the density index changed
            combo.currentIndexChanged.connect(self.MaterialComboChanged_Callback)

            # -------------------------
            # Column 2 - Density values
            # Insert the appropriate density into column 2
            if materialTaskObject.kgm3ORgcm3 is True:
                density = str(float(self.modelMaterialsDensitiesList[tableIndex]))
            else:
                density = str(float(self.modelMaterialsDensitiesList[tableIndex])/1000.0)

            self.form.tableWidget.setItem(
                tableIndex,
                2,
                QtGui.QTableWidgetItem(density))

            self.form.tableWidget.resizeColumnsToContents()
        # End of populating the table in the task dialog

        # Connect up changes in the form to the appropriate handler
        self.form.tableWidget.cellClicked.connect(self.showSelectionInGui_Callback)
        self.form.tableWidget.cellChanged.connect(self.manualDensityEntered_Callback)
        self.form.kgm3.toggled.connect(self.densityUnits_Callback)
        self.form.gcm3.toggled.connect(self.densityUnits_Callback)
    #  -------------------------------------------------------------------------
    def accept(self):
        """Run when we press the OK button"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimMaterialClass-accept\n")

        # Transfer the lists we have been working on, into the material Object
        self.materialTaskObject.solidsNameList = self.modelSolidsNamesList
        self.materialTaskObject.materialsNameList = self.modelMaterialsNamesList
        self.materialTaskObject.materialsDensityList = self.modelMaterialsDensitiesList

        # Get the body dictionary and calculate the new CoG and MoI
        # As any change in material will affect their CoG and MoI
        bodyObjDict = ST.getDictionary("SimBody")
        for bodyName in bodyObjDict:
            bodyObj = bodyObjDict[bodyName]
            ST.computeCoGAndMomentInertia(bodyObj)

        # Update all the stuff by asking for a re-compute
        self.materialTaskObject.recompute()
        CADGui.getDocument(self.materialTaskObject.Document).resetEdit()
    #  -------------------------------------------------------------------------
    def densityUnits_Callback(self):
        """Fix up the densities if we change between kg/m3 and g/cm3"""
        if self.form.kgm3.isChecked():
            self.materialTaskObject.kgm3ORgcm3 = True
        else:
            self.materialTaskObject.kgm3ORgcm3 = False

        for tableIndex in range(len(self.modelMaterialsDensitiesList)):
            if self.materialTaskObject.kgm3ORgcm3 is True:
                density = str(float(self.modelMaterialsDensitiesList[tableIndex]))
            else:
                density = str(float(self.modelMaterialsDensitiesList[tableIndex]) / 1000.0)
            self.form.tableWidget.setItem(
                tableIndex,
                2,
                QtGui.QTableWidgetItem(density))
    #  -------------------------------------------------------------------------
    def manualDensityEntered_Callback(self):
        """We have entered a density value manually"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimMaterialClass-manualDensityEntered_Callback\n")

        currentRow = self.form.tableWidget.currentRow()
        currentColumn = self.form.tableWidget.currentColumn()

        # Update if a Custom density has been entered in column 2
        if currentColumn == 2:
            densityStr = self.form.tableWidget.item(currentRow, 2).text()
            if len(densityStr) > 0:
                if 'e' in densityStr:
                    densityNoComma = densityStr
                else:
                    # The next few rows are fancy python
                    # Don't change them at all, unless you know what they mean
                    # Filter out any non-numerics
                    density = ''.join(x for x in densityStr if x.isdigit() or x in ['.', '-', ','])
                    # Replace any comma with a full-stop [period]
                    densityNoComma = ''.join(x if x.isdigit() or x in ['.', '-'] else '.' for x in density)
            else:
                densityNoComma = "1e-9"

            # Update the entry with the new custom value
            self.modelMaterialsNamesList[currentRow] = 'Custom'
            if self.materialTaskObject.kgm3ORgcm3 is True:
                self.modelMaterialsDensitiesList[currentRow] = float(str(densityNoComma))
            else:
                self.modelMaterialsDensitiesList[currentRow] = float(str(densityNoComma)) * 1000.0

            # Update the density to being a Custom one in the table
            # (the last density option in the list)
            combo = self.form.tableWidget.cellWidget(currentRow, 1)
            combo.setCurrentIndex(len(self.densityDict) - 1)
            self.form.tableWidget.setCellWidget(currentRow, 1, combo)
            # Write the new density value which was entered, back into column 2 of current Row
            # It was entered with the current units, so it should stay the same size as was entered
            self.form.tableWidget.item(currentRow, 2).setText(str(densityNoComma))
    #  -------------------------------------------------------------------------
    def MaterialComboChanged_Callback(self):
        """We have changed the type of material for this body"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimMaterialClass-MaterialComboChanged_Callback\n")

        # Find out where in the table our change occurred
        currentRow = self.form.tableWidget.currentRow()
        currentColumn = self.form.tableWidget.currentColumn()

        # Ignore false alarm calls to this callback function
        if currentRow < 0 and currentColumn < 0:
            return

        # Find the new material name we have selected
        MaterialClassombo = self.form.tableWidget.cellWidget(currentRow, 1)
        materialName = MaterialClassombo.currentText()

        # Update the entry with new material and density
        self.modelMaterialsNamesList[currentRow] = materialName
        self.modelMaterialsDensitiesList[currentRow] = float(self.densityDict[materialName])

        # Display the newly selected density in the table
        # NOTE: Density is stored internally as kg / m^3
        if self.materialTaskObject.kgm3ORgcm3 is True:
            self.form.tableWidget.setItem(currentRow, 2, QtGui.QTableWidgetItem(str(self.densityDict[materialName])))
        else:
            self.form.tableWidget.setItem(currentRow, 2, QtGui.QTableWidgetItem(str(self.densityDict[materialName] / 1000.0)))

        self.form.tableWidget.resizeColumnsToContents()
    #  -------------------------------------------------------------------------
    def showSelectionInGui_Callback(self, row, column):
        """Show the object in the Gui when we click on its name in the table"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimMaterialClass-showSelectionInGui_Callback\n")

        #  Select the object to make it highlighted
        if column == 0:
            # Find the object matching the solid item we have clicked on
            selectionObjectName = self.form.tableWidget.item(row, column).text()
            selection_object = CAD.ActiveDocument.findObjects(Label="^"+selectionObjectName+"$")[0]
            # Clear other possible visible selections and make this solid show in the "selected" colour
            CADGui.Selection.clearSelection()
            CADGui.Selection.addSelection(selection_object)
    #  -------------------------------------------------------------------------
    def getStandardButtons(self):
        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""
        if Debug:
            CAD.Console.PrintMessage("TaskPanelSimAnimateClass-getStandardButtons\n")
        return QtGui.QDialogButtonBox.Ok
    #  -------------------------------------------------------------------------
    def __getstate__(self):
        if Debug:
            CAD.Console.PrintMessage("SimMaterialClass-__getstate__\n")
    #  -------------------------------------------------------------------------
    def __setstate__(self, state):
        if Debug:
            CAD.Console.PrintMessage("SimMaterialClass-__setstate__\n")
# =============================================================================
#class TaskPanelSimBodyClass:
#    """Task panel for adding and editing Sim Bodies"""
#    if Debug:
#        ST.Mess("TaskPanelSimBodyClass-CLASS")
#
#
#  ----------------------------------------------------------
#
#    def __init__(self, bodyTaskObject):
#        """Run on first instantiation of a TaskPanelSimBody class
#        or when the body is re-built on loading of saved model etc.
#        [Called explicitly by FreeCAD]"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-__init__")
#
#        # Remember stuff to refer to later
#        self.bodyTaskObject = bodyTaskObject
#        self.taskDocName = CAD.getDocument(self.bodyTaskObject.Document.Name)
#        self.bodyTaskObject.Proxy = self
#
#        # Set up the form used to create the dialog box
#        ui_path = path.join(path.dirname(__file__), "TaskPanelSimBodies.ui")
#        self.form = CADGui.PySideUic.loadUi(ui_path)
#
#        # Signal that we only allow non-moving(ground) body for the first body
#        Prefix = '<html><head/><body><p align="center"><span style="font-weight:600;">'
#        Suffix = '</span></p></body></html>'
#        if bodyTaskObject.Name == "SimBody":
#            self.form.movingStationary.setText(Prefix + "Stationary" + Suffix)
#            self.form.velocityGroup.setDisabled(True)
#        else:
#            self.form.movingStationary.setText(Prefix + "Moving" + Suffix)
#            self.form.velocityGroup.setEnabled(True)
#
#        # Give the body a nice transparent blue colour
#        self.bodyTaskObject.ViewObject.Transparency = 20
#        self.bodyTaskObject.ViewObject.ShapeColor = (0.5, 0.5, 1.0, 1.0)
#        CADGui.Selection.addObserver(self.bodyTaskObject)
#
#        # --------------------------------------------------------
#        # Set up the movement plane normal stuff in the dialog box
#        # --------------------------------------------------------
#        # Fetch the movementPlaneNormal vector from the simGlobal and
#        # normalize the LARGEST coordinate to 1
#        # (i.e. it is easier to visualise [0, 1, 1] instead of [0, 0.707, 0.707]
#        # or [1, 1, 1] instead of [0.577, 0.577, 0.577])
#        self.movementPlaneNormal = ST.getsimGlobalObject().movementPlaneNormal
#        maxCoordinate = 1
#        if self.movementPlaneNormal.Length == 0:
#            CAD.Console.PrintError("The plane normal vector is the null vector - this should never occur\n")
#        else:
#            if abs(self.movementPlaneNormal.x) > abs(self.movementPlaneNormal.y):
#                if abs(self.movementPlaneNormal.x) > abs(self.movementPlaneNormal.z):
#                    maxCoordinate = abs(self.movementPlaneNormal.x)
#                else:
#                    maxCoordinate = abs(self.movementPlaneNormal.z)
#            else:
#                if abs(self.movementPlaneNormal.y) > abs(self.movementPlaneNormal.z):
#                    maxCoordinate = abs(self.movementPlaneNormal.y)
#                else:
#                    maxCoordinate = abs(self.movementPlaneNormal.z)
#        self.movementPlaneNormal /= maxCoordinate
#
#        # Tick the checkboxes where the Plane Normal has a non-zero value
#        self.form.planeX.setChecked(abs(self.movementPlaneNormal.x) > 1e-6)
#        self.form.planeY.setChecked(abs(self.movementPlaneNormal.y) > 1e-6)
#        self.form.planeZ.setChecked(abs(self.movementPlaneNormal.z) > 1e-6)
#
#        # Transfer the X/Y/Z plane normal coordinates to the form
#        self.form.planeXdeci.setValue(self.movementPlaneNormal.x)
#        self.form.planeYdeci.setValue(self.movementPlaneNormal.y)
#        self.form.planeZdeci.setValue(self.movementPlaneNormal.z)
#
#        # Temporarily disable changing to an alternative movement plane - see note in PlaneNormal_Callback
#        # Set the Define Plane tick box as un-ticked
#        self.form.definePlaneNormal.setChecked(False)
#        self.form.definePlaneNormal.setEnabled(False)
#        self.form.definePlaneNormalLabel.setEnabled(False)
#        self.form.planeX.setEnabled(False)
#        self.form.planeY.setEnabled(False)
#        self.form.planeZ.setEnabled(False)
#        self.form.planeXdeci.setEnabled(False)
#        self.form.planeYdeci.setEnabled(False)
#        self.form.planeZdeci.setEnabled(False)
#
#        # Clean things up to reflect what we have changed
#        self.PlaneNormal_Callback()
#
#        # -----------------------------
#        # Set up the assembly 4 objects
#        # -----------------------------
#        # Get any existing list of ass4Solids in this body
#        self.ass4SolidsNames = self.bodyTaskObject.ass4SolidsNames
#        self.ass4SolidsLabels = self.bodyTaskObject.ass4SolidsLabels
#
#        # Get the list of ALL possible ass4Solids in the entire assembly
#        self.modelAss4SolidsNames, self.modelAss4SolidsLabels, self.modelAss4SolidObjectsList = ST.getAllSolidsLists()
#
#        # Set up the model ass4Solids list in the combo selection box
#        self.form.partLabel.clear()
#        self.form.partLabel.addItems(self.modelAss4SolidsLabels)
#        self.form.partLabel.setCurrentIndex(0)
#        self.selectedAss4SolidsToFormF()
#
#        # Populate the form with velocities
#        self.velocitiesToFormX_Callback()
#        self.velocitiesToFormY_Callback()
#        self.velocitiesToFormZ_Callback()
#        self.angularVelToFormVal_Callback()
#
#        # Set the Radians and m/s radio buttons as the default
#        self.form.radians.setChecked(True)
#        self.form.mms.setChecked(True)
#
#        # --------------------------------------------------------
#        # Set up the callback functions for various things changed
#        # --------------------------------------------------------
#        self.form.buttonRemovePart.clicked.connect(self.buttonRemovePart_Callback)
#        self.form.buttonAddPart.clicked.connect(self.buttonAddPart_Callback)
#        self.form.partsList.currentRowChanged.connect(self.partsListRowChanged_Callback)
#        self.form.planeX.toggled.connect(self.PlaneNormal_Callback)
#        self.form.planeY.toggled.connect(self.PlaneNormal_Callback)
#        self.form.planeZ.toggled.connect(self.PlaneNormal_Callback)
#        self.form.planeXdeci.valueChanged.connect(self.PlaneNormal_Callback)
#        self.form.planeYdeci.valueChanged.connect(self.PlaneNormal_Callback)
#        self.form.planeZdeci.valueChanged.connect(self.PlaneNormal_Callback)
#        self.form.definePlaneNormal.toggled.connect(self.PlaneNormal_Callback)
#
#        self.form.velocityX.valueChanged.connect(self.velocitiesFromFormX_Callback)
#        self.form.velocityY.valueChanged.connect(self.velocitiesFromFormY_Callback)
#        self.form.velocityZ.valueChanged.connect(self.velocitiesFromFormZ_Callback)
#        self.form.angularVelocity.valueChanged.connect(self.angularVelFromFormVal_Callback)
#
#        self.form.mms.toggled.connect(self.velocitiesToFormX_Callback)
#        self.form.mms.toggled.connect(self.velocitiesToFormY_Callback)
#        self.form.mms.toggled.connect(self.velocitiesToFormZ_Callback)
#        self.form.ms.toggled.connect(self.velocitiesToFormX_Callback)
#        self.form.ms.toggled.connect(self.velocitiesToFormY_Callback)
#        self.form.ms.toggled.connect(self.velocitiesToFormZ_Callback)
#        self.form.degrees.toggled.connect(self.angularVelToFormVal_Callback)
#        self.form.radians.toggled.connect(self.angularVelToFormVal_Callback)
#
#    #  -------------------------------------------------------------------------
#    def accept(self):
#        """Run when we press the OK button - we have finished all the hard work
#           now transfer it into the Sim body object"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-accept")
#
#        # Refuse to 'OK' if no part references have been defined
#        if len(self.ass4SolidsLabels) == 0:
#            CAD.Console.PrintError("No Parts have been added to this body\n")
#            CAD.Console.PrintError("First add at least one Part to this body\n")
#            CAD.Console.PrintError("        or alternatively:\n")
#            CAD.Console.PrintError("add any part to it, 'OK' the body,\n")
#            CAD.Console.PrintError("and then delete it from the SimGlobal tree\n\n")
#            return
#
#        # Store the velocities into the bodyTaskObject
#        self.velocitiesFromFormX_Callback()
#        self.velocitiesFromFormY_Callback()
#        self.velocitiesFromFormZ_Callback()
#        self.angularVelFromFormVal_Callback()
#
#        # Store the normalised plane Normal into the simGlobal object
#        # If it is still undefined (zero vector) then set plane normal to z
#        if self.movementPlaneNormal == CAD.Vector(0, 0, 0):
#            self.movementPlaneNormal.z = 1.0
#        self.movementPlaneNormal /= self.movementPlaneNormal.Length
#        ST.getsimGlobalObject().movementPlaneNormal = self.movementPlaneNormal
#
#        # Run through the sub-parts and add all Shapes into a ShapeList
#        ShapeList = []
#        for ass4Solids in self.ass4SolidsNames:
#            solidObject = self.taskDocName.findObjects(Name="^" + ass4Solids + "$")[0]
#            # Put all the referenced shapes into a list
#            ShapeList.append(solidObject.Shape)
#
#        # Start off with an empty shape
#        self.bodyTaskObject.Shape = Part.Shape()
#        # Replace empty shape with a new compound one
#        if len(ShapeList) > 0:
#            # Make a Part.Compound shape out of all the Referenced shapes in the list
#            CompoundShape = Part.makeCompound(ShapeList)
#            if CompoundShape is not None:
#                # Store this compound shape into the body object
#                self.bodyTaskObject.Shape = CompoundShape
#            else:
#                # Otherwise flag that no object has a shape
#                CAD.Console.PrintError("Compound Body has no shape - this should not occur\n")
#
#        # Transfer the names and labels to the bodyTaskObject
#        self.bodyTaskObject.ass4SolidsNames = self.ass4SolidsNames
#        self.bodyTaskObject.ass4SolidsLabels = self.ass4SolidsLabels
#
#        # Save the information to the point lists
#        jointNameList = []
#        pointLabels = []
#        pointLocals = []
#
#        # Get the info for the main (first) Assembly-4 Solid in the SimBody
#        mainSolidObject = self.taskDocName.findObjects(Name="^" + self.ass4SolidsNames[0] + "$")[0]
#
#        # Save this world body PLACEMENT in the body object -
#        # POA_O is P-lacement from O-rigin to body A LCS in world (O-rigin) coordinates
#        POA_O = mainSolidObject.Placement
#        self.bodyTaskObject.world = POA_O
#
#        # Process all POINTS belonging to the main Solid (Solid A)
#        # VAa_A is the local V-ector from solid A to point 'a' in solid A local coordinates
#        for mainPoint in mainSolidObject.Group:
#            if hasattr(mainPoint, 'MapMode') and not \
#                    ('Wire' in str(mainPoint.Shape)) and not \
#                    ('Sketch' in str(mainPoint.Name)):
#                jointNameList.append(
#                    mainSolidObject.Name + "-{" + mainPoint.Name + "}")  # the name of the associated point
#                pointLabels.append(
#                    mainSolidObject.Label + "-{" + mainPoint.Label + "}")  # the label of the associated point
#                VAa_A = CAD.Vector(mainPoint.Placement.Base)
#                pointLocals.append(VAa_A)
#
#                if Debug:
#                    ST.MessNoLF("Local vector from " + mainSolidObject.Label +
#                                " to point " + mainPoint.Label +
#                                " in " + mainSolidObject.Label + " coordinates: ")
#                    ST.PrintVec(VAa_A)
#
#        # Now convert all other solids (i.e. from 1 onward) and their points into points relative to the solid A LCS
#        if len(self.ass4SolidsNames) > 1:
#            for assIndex in range(1, len(self.ass4SolidsNames)):
#                subAss4SolidsObject = self.taskDocName.findObjects(Name="^" + self.ass4SolidsNames[assIndex] + "$")[0]
#                # Find the relationship between the subAss4SolidsPlacement and the mainSolidObject.Placement
#                # i.e. from LCS of solid A to the LCS of solid B (in terms of the local coordinates of A)
#                jointNameList.append(subAss4SolidsObject.Name + "-{" + self.ass4SolidsNames[assIndex] + "}")
#                pointLabels.append(subAss4SolidsObject.Label + "-{" + self.ass4SolidsLabels[assIndex] + "}")
#                POB_O = subAss4SolidsObject.Placement
#                VAB_A = POA_O.toMatrix().inverse().multVec(POB_O.Base)
#                pointLocals.append(VAB_A)
#
#                if Debug:
#                    ST.MessNoLF("Vector from origin to " +
#                                subAss4SolidsObject.Label + " in world coordinates: ")
#                    ST.PrintVec(POB_O.Base)
#                    ST.MessNoLF("Local vector from " + mainSolidObject.Label +
#                                " to " + subAss4SolidsObject.Label +
#                                " in " + mainSolidObject.Label + " coordinates: ")
#                    ST.PrintVec(VAB_A)
#
#                # Now handle all the points which are inside Solid B
#                for sub_member in subAss4SolidsObject.Group:
#                    if hasattr(sub_member, 'MapMode'):
#                        if not ('Wire' in str(sub_member.Shape)):
#                            if not ('Sketch' in str(sub_member.Label)):
#                                jointNameList.append(subAss4SolidsObject.Name + "-{" + sub_member.Name + "}")
#                                pointLabels.append(subAss4SolidsObject.Label + "-{" + sub_member.Label + "}")
#                                VBb_B = sub_member.Placement.Base  # VBb_B: the local vector from the LCS of solid B to the point b
#                                VOb_O = POB_O.toMatrix().multVec(
#                                    VBb_B)  # VOb_O: the vector from the origin to the point b in solid B
#                                VAb_A = POA_O.toMatrix().inverse().multVec(
#                                    VOb_O)  # VAb_A: the local vector from solid A LCS to the point b in solid B
#                                pointLocals.append(VAb_A)
#
#                                if Debug:
#                                    ST.Mess(" ")
#                                    ST.MessNoLF("Vector from origin to " + subAss4SolidsObject.Label +
#                                                " in world coordinates: ")
#                                    ST.PrintVec(POB_O.Base)
#                                    ST.MessNoLF("Relationship between the " + subAss4SolidsObject.Label +
#                                                " Vector and the " + mainSolidObject.Label +
#                                                " Vector in " + mainSolidObject.Label + " coordinates: ")
#                                    ST.PrintVec(VAB_A)
#                                    ST.MessNoLF("Local vector from " + subAss4SolidsObject.Label +
#                                                " to the point " + sub_member.Label + ": ")
#                                    ST.PrintVec(VBb_B)
#                                    ST.MessNoLF("Vector from the origin to the point " + sub_member.Label +
#                                                " in body " + subAss4SolidsObject.Label + ": ")
#                                    ST.PrintVec(VOb_O)
#                                    ST.MessNoLF("Local vector from " + mainSolidObject.Label +
#                                                " to the point " + sub_member.Label +
#                                                " in body " + subAss4SolidsObject.Label + ": ")
#                                    ST.PrintVec(VAb_A)
#        if Debug:
#            ST.Mess("Names: ")
#            ST.Mess(jointNameList)
#            ST.Mess("Labels: ")
#            ST.Mess(pointLabels)
#            ST.Mess("Locals: ")
#            for vec in range(len(pointLocals)):
#                ST.MessNoLF(pointLabels[vec])
#                ST.PrintVec(pointLocals[vec])
#
#        # Condense all the duplicate points into one
#        # And save them in the bodyTaskObject
#        ST.condensePoints(jointNameList, pointLabels, pointLocals)
#        self.bodyTaskObject.jointNameList = jointNameList
#        self.bodyTaskObject.pointLabels = pointLabels
#        self.bodyTaskObject.pointLocals = pointLocals
#
#        # Recompute document to update view provider based on the shapes
#        self.bodyTaskObject.recompute()
#
#        # Switch off the Task panel
#        GuiDocument = CADGui.getDocument(self.bodyTaskObject.Document)
#        GuiDocument.resetEdit()
#
#    #  -------------------------------------------------------------------------
#    def selectedAss4SolidsToFormF(self):
#        """The ass4Solids list is the list of all the parts which make up this body.
#        Rebuild the ass4Solids list in the task panel dialog form from our copy of it"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-selectedAss4SolidsToFormF")
#        self.form.partsList.clear()
#        for subBody in self.ass4SolidsLabels:
#            self.form.partsList.addItem(subBody)
#
#    #  -------------------------------------------------------------------------
#    def velocitiesToFormX_Callback(self):
#        """Rebuild the velocities in the form when we have changed the X component"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesToFormF")
#        # If we have checked m/s units then convert to meters per second
#        if self.form.ms.isChecked():
#            self.form.velocityX.setValue(self.bodyTaskObject.worldDot.x / 1000.0)
#        else:
#            self.form.velocityX.setValue(self.bodyTaskObject.worldDot.x)
#
#    #  -------------------------------------------------------------------------
#    def velocitiesToFormY_Callback(self):
#        """Rebuild the velocities in the form when we have changed the Y component"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesToFormF")
#        # If we have checked m/s units then convert to meters per second
#        if self.form.ms.isChecked():
#            self.form.velocityY.setValue(self.bodyTaskObject.worldDot.y / 1000.0)
#        else:
#            self.form.velocityY.setValue(self.bodyTaskObject.worldDot.y)
#
#    #  -------------------------------------------------------------------------
#    def velocitiesToFormZ_Callback(self):
#        """Rebuild the velocities in the form when we have changed the Z component"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesToFormF")
#        # If we have checked m/s units then convert to meters per second
#        if self.form.ms.isChecked():
#            self.form.velocityZ.setValue(self.bodyTaskObject.worldDot.z / 1000.0)
#        else:
#            self.form.velocityZ.setValue(self.bodyTaskObject.worldDot.z)
#
#    #  -------------------------------------------------------------------------
#    def angularVelToFormVal_Callback(self):
#        """Rebuild the velocities in the form when we have changed the angular velocity"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesToFormF")
#        # If we have checked degrees units then convert to deg/s from rad/s
#        if self.form.degrees.isChecked():
#            self.form.angularVelocity.setValue(self.bodyTaskObject.phiDot * 180.0 / math.pi)
#        else:
#            self.form.angularVelocity.setValue(self.bodyTaskObject.phiDot)
#
#    #  -------------------------------------------------------------------------
#    def velocitiesFromFormX_Callback(self):
#        """Rebuild when we have changed something"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesFromFormF")
#        if self.bodyTaskObject.Name == "SimBody":
#            self.bodyTaskObject.worldDot.x = 0.0
#            return
#        # If we have checked m/s units then convert from meters per second
#        if self.form.ms.isChecked():
#            self.bodyTaskObject.worldDot.x = self.form.velocityX.value() * 1000.0
#        else:
#            self.bodyTaskObject.worldDot.x = self.form.velocityX.value()
#
#    #  -------------------------------------------------------------------------
#    def velocitiesFromFormY_Callback(self):
#        """Rebuild when we have changed something"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesFromFormF")
#        if self.bodyTaskObject.Name == "SimBody":
#            self.bodyTaskObject.worldDot.y = 0.0
#            return
#        # If we have checked m/s units then convert from meters per second
#        if self.form.ms.isChecked():
#            self.bodyTaskObject.worldDot.y = self.form.velocityY.value() * 1000.0
#        else:
#            self.bodyTaskObject.worldDot.y = self.form.velocityY.value()
#
#    #  -------------------------------------------------------------------------
#    def velocitiesFromFormZ_Callback(self):
#        """Rebuild when we have changed something"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesFromFormF")
#        if self.bodyTaskObject.Name == "SimBody":
#            self.bodyTaskObject.worldDot.z = 0.0
#            return
#
#        # If we have checked m/s units then convert from meters per second
#        if self.form.ms.isChecked():
#            self.bodyTaskObject.worldDot.z = self.form.velocityZ.value() * 1000.0
#        else:
#            self.bodyTaskObject.worldDot.z = self.form.velocityZ.value()
#
#    #  -------------------------------------------------------------------------
#    def angularVelFromFormVal_Callback(self):
#        """Rebuild when we have changed something"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-velocitiesFromFormF")
#        if self.bodyTaskObject.Name == "SimBody":
#            self.bodyTaskObject.phiDot = 0.0
#            return
#
#        # If we have checked degrees units then convert to rad/s from deg/s
#        if self.form.degrees.isChecked():
#            self.bodyTaskObject.phiDot = self.form.angularVelocity.value() * math.pi / 180.0
#        else:
#            self.bodyTaskObject.phiDot = self.form.angularVelocity.value()
#
#    #  -------------------------------------------------------------------------
#    def PlaneNormal_Callback(self):
#        """Rebuild when we have changed something to do with the plane normal
#        The plane normal transforms are built in, but might still need debugging
#        It is therefore temporarily commented out to avoid complaints
#        from users at this stage"""
#        if Debug:
#            ST.Mess("PlaneNormal_Callback")
#
#        if not self.form.definePlaneNormal.isChecked():
#            # Hide the movementPlaneNormal tick boxes if the 'define' tickbox is unchecked
#            # self.form.planeX.setEnabled(False)
#            # self.form.planeY.setEnabled(False)
#            # self.form.planeZ.setEnabled(False)
#            # self.form.planeXdeci.setEnabled(False)
#            # self.form.planeYdeci.setEnabled(False)
#            # self.form.planeZdeci.setEnabled(False)
#            self.form.planeGroup.setHidden(True)
#
#            self.taskDocName.recompute()
#            return
#
#        # Show the tick boxes
#        self.form.planeX.setEnabled(True)
#        self.form.planeY.setEnabled(True)
#        self.form.planeZ.setEnabled(True)
#        self.form.planeXdeci.setEnabled(True)
#        self.form.planeYdeci.setEnabled(True)
#        self.form.planeZdeci.setEnabled(True)
#
#        # All the following paraphanalia is to handle various methods of defining the plane vector
#        if self.form.planeX.isChecked():
#            self.movementPlaneNormal.x = self.form.planeXdeci.value()
#            if self.movementPlaneNormal.x == 0:
#                self.movementPlaneNormal.x = 1.0
#                self.form.planeXdeci.setValue(1.0)
#        else:
#            self.movementPlaneNormal.x = 0.0
#            self.form.planeXdeci.setValue(0.0)
#
#        if self.form.planeY.isChecked():
#            self.movementPlaneNormal.y = self.form.planeYdeci.value()
#            if self.movementPlaneNormal.y == 0:
#                self.movementPlaneNormal.y = 1.0
#                self.form.planeYdeci.setValue(1.0)
#        else:
#            self.movementPlaneNormal.y = 0.0
#            self.form.planeYdeci.setValue(0.0)
#
#        if self.form.planeZ.isChecked():
#            self.movementPlaneNormal.z = self.form.planeZdeci.value()
#            if self.movementPlaneNormal.z == 0:
#                self.movementPlaneNormal.z = 1.0
#                self.form.planeZdeci.setValue(1.0)
#        else:
#            self.movementPlaneNormal.z = 0.0
#            self.form.planeZdeci.setValue(0.0)
#
#        if self.movementPlaneNormal == CAD.Vector(0, 0, 0):
#            self.movementPlaneNormal.z = 1.0
#
#        # Make a temporary plane and normal vector in the object view box to show the movement plane
#        cylinder = Part.makeCylinder(5, 300, CAD.Vector(0, 0, 0), self.movementPlaneNormal)
#        plane = Part.makeCylinder(500, 1, CAD.Vector(0, 0, 0), self.movementPlaneNormal)
#        cone = Part.makeCone(10, 0, 20, CAD.Vector(self.movementPlaneNormal).multiply(300), self.movementPlaneNormal)
#        planeNormal = Part.makeCompound([cylinder, plane, cone])
#
#        self.bodyTaskObject.Shape = planeNormal
#        self.bodyTaskObject.ViewObject.Transparency = 20
#        self.bodyTaskObject.ViewObject.ShapeColor = (0.5, 1.0, 0.5, 1.0)
#
#        self.taskDocName.recompute()
#
#    #  -------------------------------------------------------------------------
#    def buttonAddPart_Callback(self):
#        """Run when we click the add part button"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-buttonAddPart_Callback")
#
#        # Find the object for the part name we have selected
#        partIndex = self.form.partLabel.currentIndex()
#        addPartObject = self.modelAss4SolidObjectsList[partIndex]
#        # Add it to the list of ass4SolidsLabels if it is not already there
#        if addPartObject.Name not in self.ass4SolidsNames:
#            self.ass4SolidsNames.append(self.modelAss4SolidsNames[partIndex])
#            self.ass4SolidsLabels.append(self.modelAss4SolidsLabels[partIndex])
#
#        # Highlight the current item
#        CADGui.Selection.clearSelection()
#        CADGui.Selection.addSelection(addPartObject)
#
#        # Rebuild the subBody's List in the form
#        self.selectedAss4SolidsToFormF()
#
#    #  -------------------------------------------------------------------------
#    def buttonRemovePart_Callback(self):
#        """Run when we remove a body already added"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-buttonRemovePart_Callback")
#
#        # Remove the current row
#        if len(self.ass4SolidsNames) > 0:
#            row = self.form.partsList.currentRow()
#            self.ass4SolidsNames.pop(row)
#            self.ass4SolidsLabels.pop(row)
#
#        # Rebuild the subBodies in the form
#        self.selectedAss4SolidsToFormF()
#
#    #  -------------------------------------------------------------------------
#    def partsListRowChanged_Callback(self, row):
#        """Actively select the part in the requested row,
#           to make it visible when viewing parts already in list"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-partsListRowChanged_Callback")
#
#        if len(self.ass4SolidsNames) > 0:
#            # Clear the highlight on the previous item selected
#            CADGui.Selection.clearSelection()
#            # Highlight the current item
#            selection_object = self.taskDocName.findObjects(Name="^" + self.ass4SolidsNames[row] + "$")[0]
#            CADGui.Selection.addSelection(selection_object)
#
#    #  -------------------------------------------------------------------------
#    def getStandardButtons(self):
#        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-getStandardButtons")
#        return QtGui.QDialogButtonBox.Ok
#
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-__getstate__")
#
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("TaskPanelSimBodyClass-__setstate__")
## =============================================================================
#class TaskPanelSimForceClass:
#    """Taskpanel for adding a MechSim Force"""
#    if Debug:
#        ST.Mess("TaskPanelSimForceClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, forceTaskObject):
#        """Run on first instantiation of a TaskPanelSimForce class"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-__init__")
#
#        self.forceTaskObject = forceTaskObject
#        forceTaskObject.Proxy = self
#
#        # Define variables for later use
#        # (class variables should preferably be used first in the __init__ block)
#        self.pointNameListOneOne = []
#        self.pointNameListOneTwo = []
#        self.pointNameListTwoOne = []
#        self.pointNameListTwoTwo = []
#        self.pointLabelListOneOne = []
#        self.pointLabelListOneTwo = []
#        self.pointLabelListTwoOne = []
#        self.pointLabelListTwoTwo = []
#
#        # Load up the task panel layout definition
#        ui_Path = path.join(path.dirname(__file__), "TaskPanelSimForces.ui")
#        self.form = CADGui.PySideUic.loadUi(ui_Path)
#
#        # Populate the body object dictionary with body {names : objects}
#        self.bodyObjDict = ST.getDictionary("SimBody")
#
#        # Populate the bodyLabels and bodyNames lists
#        self.bodyLabels = []
#        self.bodyNames = []
#        self.bodyObjects = []
#        for bodyName in self.bodyObjDict:
#            bodyObj = self.bodyObjDict[bodyName]
#            self.bodyNames.append(bodyObj.Name)
#            self.bodyLabels.append(bodyObj.Name)
#            self.bodyObjects.append(self.bodyObjDict[bodyObj.Name])
#
#        # Populate the form from the forceTaskObject
#        self.form.linSpringLength.setValue(self.forceTaskObject.LengthAngle0)
#        self.form.linSpringStiffness.setValue(self.forceTaskObject.Stiffness)
#        self.form.rotSpringAngle.setValue(self.forceTaskObject.LengthAngle0)
#        self.form.rotSpringStiffness.setValue(self.forceTaskObject.Stiffness)
#        self.form.linSpringDamp.setValue(self.forceTaskObject.DampingCoeff)
#        self.form.linSpringDampLength.setValue(self.forceTaskObject.LengthAngle0)
#        self.form.linSpringDampStiffness.setValue(self.forceTaskObject.Stiffness)
#        self.form.rotSpringDamp.setValue(self.forceTaskObject.DampingCoeff)
#        self.form.rotSpringDampAngle.setValue(self.forceTaskObject.LengthAngle0)
#        self.form.rotSpringDampStiffness.setValue(self.forceTaskObject.Stiffness)
#
#        # Connect changes in the body choices to the respective Callback functions
#        self.form.actuatorCombo.currentIndexChanged.connect(self.actuator_Changed_Callback)
#        self.form.body_1B1P.currentIndexChanged.connect(self.body_1B1P_Changed_Callback)
#        self.form.body_1B2P.currentIndexChanged.connect(self.body_1B2P_Changed_Callback)
#        self.form.body1_2B1P.currentIndexChanged.connect(self.body1_2B1P_Changed_Callback)
#        self.form.body2_2B1P.currentIndexChanged.connect(self.body2_2B1P_Changed_Callback)
#        self.form.body1_2B2P.currentIndexChanged.connect(self.body1_2B2P_Changed_Callback)
#        self.form.body2_2B2P.currentIndexChanged.connect(self.body2_2B2P_Changed_Callback)
#
#        # Connect changes in the point choices to the respective Callback functions
#        self.form.point_1B1P.currentIndexChanged.connect(self.point_1B1P_Changed_Callback)
#        self.form.point1_1B2P.currentIndexChanged.connect(self.point1_1B2P_Changed_Callback)
#        self.form.point2_1B2P.currentIndexChanged.connect(self.point2_1B2P_Changed_Callback)
#        self.form.point_2B1P.currentIndexChanged.connect(self.point_2B1P_Changed_Callback)
#        self.form.point1_2B2P.currentIndexChanged.connect(self.point1_2B2P_Changed_Callback)
#        self.form.point2_2B2P.currentIndexChanged.connect(self.point2_2B2P_Changed_Callback)
#
#        # Set up the body/actuator combo boxes
#        self.form.actuatorCombo.clear()
#        self.form.actuatorCombo.addItems(ST.FORCE_TYPE)
#        self.form.actuatorCombo.setCurrentIndex(forceTaskObject.forceType)
#
#        self.form.body_1B1P.clear()
#        self.form.body_1B1P.addItems(self.bodyLabels)
#        self.form.body_1B1P.setCurrentIndex(forceTaskObject.Bi)
#
#        self.form.body_1B2P.clear()
#        self.form.body_1B2P.addItems(self.bodyLabels)
#        self.form.body_1B2P.setCurrentIndex(forceTaskObject.Bi)
#
#        self.form.body1_2B1P.clear()
#        self.form.body1_2B1P.addItems(self.bodyLabels)
#        self.form.body1_2B1P.setCurrentIndex(forceTaskObject.Bi)
#
#        self.form.body2_2B1P.clear()
#        self.form.body2_2B1P.addItems(self.bodyLabels)
#        self.form.body2_2B1P.setCurrentIndex(forceTaskObject.Bj)
#
#        self.form.body1_2B2P.clear()
#        self.form.body1_2B2P.addItems(self.bodyLabels)
#        self.form.body1_2B2P.setCurrentIndex(forceTaskObject.Bi)
#
#        self.form.body2_2B2P.clear()
#        self.form.body2_2B2P.addItems(self.bodyLabels)
#        self.form.body1_2B2P.setCurrentIndex(forceTaskObject.Bj)
#
#        # Copy the state of the gravity vector to the form
#        simGlobalObject = ST.getsimGlobalObject()
#        if simGlobalObject.gravityVector.x != 0.0 and simGlobalObject.gravityValid is True:
#            self.form.gravityX.setChecked(True)
#        else:
#            self.form.gravityX.setChecked(False)
#        if simGlobalObject.gravityVector.y != 0.0 and simGlobalObject.gravityValid is True:
#            self.form.gravityY.setChecked(True)
#        else:
#            self.form.gravityY.setChecked(False)
#        if simGlobalObject.gravityVector.z != 0.0 and simGlobalObject.gravityValid is True:
#            self.form.gravityZ.setChecked(True)
#        else:
#            self.form.gravityZ.setChecked(False)
#
#        # Connect changes in the gravity direction to the respective Callback function
#        self.form.gravityX.toggled.connect(self.gravityX_Changed_Callback)
#        self.form.gravityY.toggled.connect(self.gravityY_Changed_Callback)
#        self.form.gravityZ.toggled.connect(self.gravityZ_Changed_Callback)
#    #  -------------------------------------------------------------------------
#    def accept(self):
#        """Run when we press the OK button - we have finished all the hard work"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-accept")
#
#        # Transfer spring values to the form
#        # Actuator type
#        # 0-Gravity
#        # 1=Linear Spring 2-Rotational Spring
#        # 3-Linear Spring Damper 4-Rotational Spring Damper 5-Unilateral Spring Damper
#        # 6-Constant Force Local to Body 7-Constant Global Force 8-Constant Torque about a Point
#        # 9-Contact Friction 10-Motor 11-Motor with Air Friction
#        if self.forceTaskObject.forceType == 0:
#            pass
#        elif self.forceTaskObject.forceType == 1:
#            self.forceTaskObject.LengthAngle0 = self.form.linSpringLength.value()
#            self.forceTaskObject.Stiffness = self.form.linSpringStiffness.value()
#        elif self.forceTaskObject.forceType == 2:
#            self.forceTaskObject.LengthAngle0 = self.form.rotSpringAngle.value()
#            self.forceTaskObject.Stiffness = self.form.rotSpringStiffness.value()
#        elif self.forceTaskObject.forceType == 3:
#            self.forceTaskObject.DampingCoeff = self.form.linSpringDamp.value()
#            self.forceTaskObject.LengthAngle0 = self.form.linSpringDampLength.value()
#            self.forceTaskObject.Stiffness = self.form.linSpringDampStiffness.value()
#        elif self.forceTaskObject.forceType == 4:
#            self.forceTaskObject.DampingCoeff = self.form.rotSpringDamp.value()
#            self.forceTaskObject.LengthAngle0 = self.form.rotSpringDampAngle.value()
#            self.forceTaskObject.Stiffness = self.form.rotSpringDampStiffness.value()
#        elif self.forceTaskObject.forceType == 7:
#            self.forceTaskObject.constWorldForce = self.form.globalForceMag.value() * CAD.Vector(self.form.globalForceX.value(), self.form.globalForceY.value(), self.form.globalForceZ.value())
#        else:
#            CAD.Console.PrintError("Code for the selected force is still in development")
#
#        # Switch off the Task panel
#        GuiDocument = CADGui.getDocument(self.forceTaskObject.Document)
#        GuiDocument.resetEdit()
#
#        # Validate and Clean up Gravity entries if this entry is Gravity
#        if self.form.actuatorCombo.currentIndex() == 0:
#            simGlobalObject = ST.getsimGlobalObject()
#            # Remove the other gravity entry if another one already altered the vector
#            if (self.forceTaskObject.newForce is True) and (simGlobalObject.gravityValid is True):
#                # We now have two gravity forces
#                # Find the first gravity force and remove it
#                # the new one will always be after it in the list
#                self.forceTaskObject.newForce = False
#                forceList = CAD.getDocument(self.forceTaskObject.Document.Name).findObjects(Name="SimForce")
#                for forceObj in forceList:
#                    if forceObj.forceType == 0 and len(forceList) > 1:
#                        CAD.ActiveDocument.removeObject(forceObj.Name)
#                        break
#            # Remove this gravity entry if it is null
#            if simGlobalObject.gravityVector == CAD.Vector(0.0, 0.0, 0.0):
#                forceList = CAD.getDocument(self.forceTaskObject.Document.Name).findObjects(Name="SimForce")
#                if len(forceList) > 0:
#                    for forceObj in forceList:
#                        if forceObj.forceType == 0:
#                            CAD.ActiveDocument.removeObject(forceObj.Name)
#                            simGlobalObject.gravityValid = False
#                            break
#            else:
#                # All is OK, so validate the gravityVector in the simGlobal
#                self.forceTaskObject.newForce = False
#                simGlobalObject.gravityValid = True
#    #  -------------------------------------------------------------------------
#    def updateToolTipF(self, ComboName, LabelList):
#        bodyString = ""
#        for point in LabelList:
#            bodyString = bodyString + ST.parsePoint(point) + "\n========\n"
#        ComboName.setToolTip(str(bodyString[:-10]))
#    #  -------------------------------------------------------------------------
#    def body_1B1P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body_1B1P_Changed_Callback")
#
#        # Update the body in the forceTaskObject
#        bodyIndex = self.form.body_1B1P.currentIndex()
#        self.forceTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bi = bodyIndex
#
#        # Load the new set of point labels, which are in the specified body, into the form
#        self.pointNameListOneOne, self.pointLabelListOneOne = ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        self.updateToolTipF(self.form.body_1B1P, self.pointLabelListOneOne)
#
#        self.updateToolTipF(self.form.point_1B1P, [self.pointLabelListOneOne[0]])
#        self.form.point_1B1P.clear()
#        self.form.point_1B1P.addItems(self.pointLabelListOneOne)
#        self.form.point_1B1P.setCurrentIndex(0)
#    #  -------------------------------------------------------------------------
#    def body_1B2P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body_1B2P_Changed_Callback")
#
#        # Update the body in the forceTaskObject
#        bodyIndex = self.form.body_1B2P.currentIndex()
#        self.forceTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bi = bodyIndex
#
#        # Load the new set of point labels, which are in the specified body, into the form
#        self.pointNameListOneOne, self.pointLabelListOneOne = ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        self.updateToolTipF(self.form.body_1B2P, self.pointLabelListOneOne)
#
#        self.updateToolTipF(self.form.point1_1B2P, [self.pointLabelListOneOne[0]])
#        self.form.point1_1B2P.clear()
#        self.form.point1_1B2P.addItems(self.pointLabelListOneOne)
#        self.form.point1_1B2P.setCurrentIndex(0)
#
#        self.pointNameListOneTwo = self.pointNameListOneOne[:]
#        self.pointLabelListOneTwo = self.pointLabelListOneOne[:]
#
#        self.updateToolTipF(self.form.point2_1B2P, [self.pointLabelListOneTwo[0]])
#        self.form.point2_1B2P.clear()
#        self.form.point2_1B2P.addItems(self.pointLabelListOneTwo)
#        self.form.point2_1B2P.setCurrentIndex(0)
#    #  -------------------------------------------------------------------------
#    def body1_2B1P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body1_2B1P_Changed_Callback")
#
#        # Update the body in the forceTaskObject
#        bodyIndex = self.form.body1_2B1P.currentIndex()
#        self.forceTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bi = bodyIndex
#
#        # Load the new set of point labels, which are in the specified body, into the form
#        bodyIndex = self.form.body1_2B1P.currentIndex()
#        self.pointNameListOneOne, self.pointLabelListOneOne = ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        self.updateToolTipF(self.form.body1_2B1P, self.pointLabelListOneOne)
#
#        self.updateToolTipF(self.form.point_2B1P, [self.pointLabelListOneOne[0]])
#        self.form.point_2B1P.clear()
#        self.form.point_2B1P.addItems(self.pointLabelListOneOne)
#        self.form.point_2B1P.setCurrentIndex(0)
#    #  -------------------------------------------------------------------------
#    def body2_2B1P_Changed_Callback(self):
#        """Only update the body in forceTaskObject - there are no points associated with it"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body2_2B1P_Changed_Callback")
#
#        # Update the body tooltip
#        self.form.body2_2B1P.setToolTip("")
#
#        # Update the forceTaskObject
#        bodyIndex = self.form.body2_2B1P.currentIndex()
#        self.forceTaskObject.body_J_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_J_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bj = bodyIndex
#    #  -------------------------------------------------------------------------
#    def body1_2B2P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body1_2B2P_Changed_Callback")
#
#        # Update the body in the forceTaskObject
#        bodyIndex = self.form.body1_2B2P.currentIndex()
#        self.forceTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bi = bodyIndex
#
#        # Load the new set of point labels, which are contained in the specified body, into the form
#        self.pointNameListOneOne, self.pointLabelListOneOne = ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        self.updateToolTipF(self.form.body1_2B2P, self.pointLabelListOneOne)
#
#        self.updateToolTipF(self.form.point1_2B2P, [self.pointLabelListOneOne[0]])
#        self.form.point1_2B2P.clear()
#        self.form.point1_2B2P.addItems(self.pointLabelListOneOne)
#        self.form.point1_2B2P.setCurrentIndex(0)
#    #  -------------------------------------------------------------------------
#    def body2_2B2P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-body2_2B2P_Changed_Callback")
#
#        # Update the body in the forceTaskObject
#        bodyIndex = self.form.body2_2B2P.currentIndex()
#        self.forceTaskObject.body_J_Name = self.bodyNames[bodyIndex]
#        self.forceTaskObject.body_J_Label = self.bodyLabels[bodyIndex]
#        self.forceTaskObject.Bj = bodyIndex
#
#        # Load the new set of point labels, which are in the specified body, into the form
#        self.pointNameListTwoTwo, self.pointLabelListTwoTwo = ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        self.updateToolTipF(self.form.body2_2B2P, self.pointLabelListTwoTwo)
#
#        self.updateToolTipF(self.form.point2_2B2P, [self.pointLabelListTwoTwo[0]])
#        self.form.point2_2B2P.clear()
#        self.form.point2_2B2P.addItems(self.pointLabelListTwoTwo)
#        self.form.point2_2B2P.setCurrentIndex(0)
#    #  -------------------------------------------------------------------------
#    def point_1B1P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point_1B1P_Changed_Callback")
#
#        pointIndex = self.form.point_1B1P.currentIndex()
#
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point_1B1P.setToolTip(str(ST.parsePoint(self.pointLabelListOneOne[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_i_Name = self.pointNameListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Label = self.pointLabelListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def point1_1B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point1_1B2P_Changed_Callback")
#
#        pointIndex = self.form.point1_1B2P.currentIndex()
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point1_1B2P.setToolTip(str(ST.parsePoint(self.pointLabelListOneOne[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_i_Name = self.pointNameListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Label = self.pointLabelListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def point2_1B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point2_1B2P_Changed_Callback")
#
#        pointIndex = self.form.point2_1B2P.currentIndex()
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point2_1B2P.setToolTip(str(ST.parsePoint(self.pointLabelListOneTwo[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_i_Name = self.pointNameListOneTwo[pointIndex]
#            self.forceTaskObject.point_i_Label = self.pointLabelListOneTwo[pointIndex]
#            self.forceTaskObject.point_i_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def point_2B1P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point_2B1P_Changed_Callback")
#
#        pointIndex = self.form.point_2B1P.currentIndex()
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point_2B1P.setToolTip(str(ST.parsePoint(self.pointLabelListOneOne[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_i_Name = self.pointNameListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Label = self.pointLabelListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def point1_2B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point1_2B2P_Changed_Callback")
#
#        pointIndex = self.form.point1_2B2P.currentIndex()
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point1_2B2P.setToolTip(str(ST.parsePoint(self.pointLabelListOneOne[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_i_Name = self.pointNameListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Label = self.pointLabelListOneOne[pointIndex]
#            self.forceTaskObject.point_i_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def point2_2B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-point2_2B2P_Changed_Callback")
#
#        pointIndex = self.form.point2_2B2P.currentIndex()
#        if pointIndex != -1:
#            # Update the tooltip
#            self.form.point2_2B2P.setToolTip(str(ST.parsePoint(self.pointLabelListTwoTwo[pointIndex])))
#
#            # Update the point in the forceTaskObject
#            self.forceTaskObject.point_j_Name = self.pointNameListTwoTwo[pointIndex]
#            self.forceTaskObject.point_j_Label = self.pointLabelListTwoTwo[pointIndex]
#            self.forceTaskObject.point_j_Index = pointIndex
#    #  -------------------------------------------------------------------------
#    def gravityX_Changed_Callback(self):
#        """The X gravity check box has gone from either checked to unchecked
#        or unchecked to checked"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-gravityX_Changed_Callback")
#        simGlobalObject = ST.getsimGlobalObject()
#        if self.form.gravityX.isChecked():
#            # X has been checked, uncheck Y and Z
#            self.form.gravityY.setChecked(False)
#            self.form.gravityZ.setChecked(False)
#            # Set the appropriate gravity vector
#            simGlobalObject.gravityVector = CAD.Vector(-9810.0, 0.0, 0.0)
#        else:
#            simGlobalObject.gravityVector.x = 0.0
#        if Debug:
#            ST.Mess("Gravity Vector")
#            ST.Mess(simGlobalObject.gravityVector)
#    #  -------------------------------------------------------------------------
#    def gravityY_Changed_Callback(self):
#        """The Y gravity check box has gone from either checked to unchecked
#        or unchecked to checked"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-gravityY_Changed_Callback")
#        simGlobalObject = ST.getsimGlobalObject()
#        if self.form.gravityY.isChecked():
#            # Y has been checked, uncheck X and Z
#            self.form.gravityX.setChecked(False)
#            self.form.gravityZ.setChecked(False)
#            # Set the appropriate gravity vector
#            simGlobalObject.gravityVector = CAD.Vector(0.0, -9810.0, 0.0)
#        else:
#            simGlobalObject.gravityVector.y = 0.0
#        if Debug:
#            ST.Mess("Gravity Vector")
#            ST.Mess(simGlobalObject.gravityVector)
#    #  -------------------------------------------------------------------------
#    def gravityZ_Changed_Callback(self):
#        """The Z gravity check box has gone from either checked to unchecked
#        or unchecked to checked"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-gravityZ_Changed_Callback")
#        simGlobalObject = ST.getsimGlobalObject()
#        if self.form.gravityZ.isChecked():
#            # Z has been checked, uncheck X and Y
#            self.form.gravityX.setChecked(False)
#            self.form.gravityY.setChecked(False)
#            # Set the appropriate gravity vector
#            simGlobalObject.gravityVector = CAD.Vector(0.0, 0.0, -9810.0)
#        else:
#            simGlobalObject.gravityVector.z = 0.0
#        if Debug:
#            ST.Mess("Gravity Vector")
#            ST.Mess(simGlobalObject.gravityVector)
#    #  -------------------------------------------------------------------------
#    def actuator_Changed_Callback(self):
#        """Selects which of the forceData and bodyPointData pages are active,
#        and shown/hidden based on the actuator type"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-actuator_Changed_Callback")
#
#        # Actuator type
#        # 0-Gravity
#        # 1=Linear Spring 2-Rotational Spring
#        # 3-Linear Spring Damper 4-Rotational Spring Damper 5-Unilateral Spring Damper
#        # 6-Constant Force Local to Body 7-Constant Global Force 8-Constant Torque about a Point
#        # 9-Contact Friction 10-Motor 11-Motor with Air Friction
#
#        # Body vs Point page numbers in the Forces form:
#        OneBodyTwoPoints = 0
#        TwoBodiesOnePoint = 1
#        TwoBodiesTwoPoints = 2
#        OneBodyOnePoint = 3
#
#        # Set up the combos in the form, depending on the actuator type
#        forceType = self.form.actuatorCombo.currentIndex()
#        self.forceTaskObject.forceType = forceType
#
#        # The Gravity actuator option - forceData page number is 11
#        if forceType == 0:
#            # Gravity has no bodies and points
#            # And it is on page 12 of 12 (forceData page 11)
#            self.form.forceData.setCurrentIndex(11)
#            self.form.bodyPointData.setHidden(True)
#            return
#
#        # After gravity, the forceData page numbers are all one less than forceType
#        self.form.forceData.setCurrentIndex(forceType - 1)
#
#        # Two bodies, two points
#        if (forceType == 1) or (forceType == 3) or (forceType == 5):
#            self.form.bodyPointData.setCurrentIndex(TwoBodiesTwoPoints)
#            self.form.bodyPointData.setHidden(False)
#
#        # One body, two points
#        elif forceType == 2 or forceType == 4 or forceType == 6:
#            self.form.bodyPointData.setCurrentIndex(OneBodyTwoPoints)
#            self.form.bodyPointData.setHidden(False)
#
#        # No bodies or points
#        elif forceType == 7:
#            self.form.bodyPointData.setHidden(True)
#
#        # One body, one point
#        elif forceType == 8:
#            self.form.bodyPointData.setCurrentIndex(OneBodyOnePoint)
#            self.form.bodyPointData.setHidden(False)
#
#        # No bodies or points
#        elif forceType == 9 or forceType == 10 or forceType == 11:
#            self.form.bodyPointData.setHidden(True)
#    #  -------------------------------------------------------------------------
#    def getStandardButtons(self):
#        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-getStandardButtons")
#        return int(QtGui.QDialogButtonBox.Ok)
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("TaskPanelSimForceClass-__setstate__")
## ==============================================================================
#class TaskPanelSimJointClass:
#    """Task panel for editing Sim Joints"""
#    if Debug:
#        ST.Mess("TaskPanelSimJointClass-CLASS")
#    #  -------------------------------------------------------------------------
#    def __init__(self, jointTaskObject):
#        """Run on first instantiation of a TaskPanelJoint class"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-__init__")
#
#        self.jointTaskObject = jointTaskObject
#        jointTaskObject.Proxy = self
#
#        # Initialise some class instances which we will use later
#        self.pointNameListFirstBody = []
#        self.pointLabelListFirstBody = []
#        self.pointNameListSecondBody = []
#        self.pointLabelListSecondBody = []
#
#        # Load up the Task panel dialog definition file
#        ui_path = path.join(path.dirname(__file__), "TaskPanelSimJoints.ui")
#        self.form = CADGui.PySideUic.loadUi(ui_path)
#
#        # Populate the body object dictionary with body {names : objects}
#        self.bodyObjDict = ST.getDictionary("SimBody")
#
#        # Make up the lists of possible body names and labels and their objects to ensure ordering
#        self.bodyNames = []
#        self.bodyLabels = []
#        self.bodyObjects = []
#        for bodyName in self.bodyObjDict:
#            bodyObj = self.bodyObjDict[bodyName]
#            self.bodyNames.append(bodyObj.Name)
#            self.bodyLabels.append(bodyObj.Label)
#            self.bodyObjects.append(bodyObj)
#
#        if self.jointTaskObject.SimJoint == "Revolute":
#            # Switch on rotation "driver function select" and switch off translation "driver function select"
#            self.form.withRotationDriver.setVisible(True)
#            self.form.withTranslationDriver.setVisible(False)
#        elif self.jointTaskObject.SimJoint == "Translation":
#            # Switch on translation "driver function select" and switch off rotation "driver function select"
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withTranslationDriver.setVisible(True)
#        else:
#            # Switch off both "driver function selects"
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withTranslationDriver.setVisible(False)
#
#        if self.jointTaskObject.FunctType == -1:
#            # We hide all equations
#            self.hideAllEquationsF()
#            # Switch off all Radio Buttons
#            # self.form.radioButtonA.setVisible(False)
#            # self.form.radioButtonB.setVisible(False)
#            # self.form.radioButtonC.setVisible(False)
#            # self.form.radioButtonD.setVisible(False)
#            # self.form.radioButtonE.setVisible(False)
#            # self.form.radioButtonF.setVisible(False)
#            self.form.radioButtonA.setChecked(False)
#            self.form.radioButtonB.setChecked(False)
#            self.form.radioButtonC.setChecked(False)
#            self.form.radioButtonD.setChecked(False)
#            self.form.radioButtonE.setChecked(False)
#            self.form.radioButtonF.setChecked(False)
#        else:
#            self.form.withRotationDriver.setChecked(True)
#            # We show all equations
#            self.showAllEquationsF()
#            # Switch on the relevant Radio Buttons
#            if jointTaskObject.FunctType == 0:
#                self.form.radioButtonA.setChecked(True)
#            elif jointTaskObject.FunctType == 1:
#                self.form.radioButtonB.setChecked(True)
#            elif jointTaskObject.FunctType == 2:
#                self.form.radioButtonC.setChecked(True)
#            elif jointTaskObject.FunctType == 3:
#                self.form.radioButtonD.setChecked(True)
#            elif jointTaskObject.FunctType == 4:
#                self.form.radioButtonE.setChecked(True)
#            elif jointTaskObject.FunctType == 5:
#                self.form.radioButtonF.setChecked(True)
#            # if jointTaskObject.FunctType == 0:
#            #     self.form.radioButtonA.setVisible(True)
#            # elif jointTaskObject.FunctType == 1:
#            #     self.form.radioButtonB.setVisible(True)
#            # elif jointTaskObject.FunctType == 2:
#            #     self.form.radioButtonC.setVisible(True)
#            # elif jointTaskObject.FunctType == 3:
#            #     self.form.radioButtonD.setVisible(True)
#            # elif jointTaskObject.FunctType == 4:
#            #     self.form.radioButtonE.setVisible(True)
#            # elif jointTaskObject.FunctType == 5:
#            #     self.form.radioButtonF.setVisible(True)
#
#        # Connect changes to the respective Callback functions
#        self.form.jointType.currentIndexChanged.connect(self.jointType_Changed_Callback)
#
#        self.form.body_1B1P.currentIndexChanged.connect(self.body_1B1P_Changed_Callback)
#        self.form.body_1B2P.currentIndexChanged.connect(self.body_1B2P_Changed_Callback)
#        self.form.body1_2B2P.currentIndexChanged.connect(self.body1_2B2P_Changed_Callback)
#        self.form.body2_2B2P.currentIndexChanged.connect(self.body2_2B2P_Changed_Callback)
#        self.form.body1_2B3P.currentIndexChanged.connect(self.body1_2B3P_Changed_Callback)
#        self.form.body2_2B3P.currentIndexChanged.connect(self.body2_2B3P_Changed_Callback)
#        self.form.body1_2B4P.currentIndexChanged.connect(self.body1_2B4P_Changed_Callback)
#        self.form.body2_2B4P.currentIndexChanged.connect(self.body2_2B4P_Changed_Callback)
#        self.form.bodyDisc.currentIndexChanged.connect(self.bodyDisc_Changed_Callback)
#
#        self.form.point_1B1P.currentIndexChanged.connect(self.point_1B1P_Changed_Callback)
#        self.form.vectorHead_1B2P.currentIndexChanged.connect(self.vectorHead_1B2P_Changed_Callback)
#        self.form.vectorTail_1B2P.currentIndexChanged.connect(self.vectorTail_1B2P_Changed_Callback)
#        self.form.point1_2B2P.currentIndexChanged.connect(self.point1_2B2P_Changed_Callback)
#        self.form.point2_2B2P.currentIndexChanged.connect(self.point2_2B2P_Changed_Callback)
#        self.form.vectorHead_2B3P.currentIndexChanged.connect(self.vectorHead_2B3P_Changed_Callback)
#        self.form.vectorTail_2B3P.currentIndexChanged.connect(self.vectorTail_2B3P_Changed_Callback)
#        self.form.point_2B3P.currentIndexChanged.connect(self.point_2B3P_Changed_Callback)
#        self.form.vector1Head_2B4P.currentIndexChanged.connect(self.vector1Head_2B4P_Changed_Callback)
#        self.form.vector1Tail_2B4P.currentIndexChanged.connect(self.vector1Tail_2B4P_Changed_Callback)
#        self.form.vector2Head_2B4P.currentIndexChanged.connect(self.vector2Head_2B4P_Changed_Callback)
#        self.form.vector2Tail_2B4P.currentIndexChanged.connect(self.vector2Tail_2B4P_Changed_Callback)
#
#        self.form.pointDiscCentre.currentIndexChanged.connect(self.pointDiscCentre_Changed_Callback)
#        self.form.pointDiscRim.currentIndexChanged.connect(self.pointDiscRim_Changed_Callback)
#
#        self.form.withRotationDriver.toggled.connect(self.withRotationDriver_Changed_Callback)
#        self.form.withTranslationDriver.toggled.connect(self.withTranslationDriver_Changed_Callback)
#
#        self.form.radioButtonA.toggled.connect(self.radioA_Changed_Callback)
#        self.form.radioButtonB.toggled.connect(self.radioB_Changed_Callback)
#        self.form.radioButtonC.toggled.connect(self.radioC_Changed_Callback)
#        self.form.radioButtonD.toggled.connect(self.radioD_Changed_Callback)
#        self.form.radioButtonE.toggled.connect(self.radioE_Changed_Callback)
#        self.form.radioButtonF.toggled.connect(self.radioF_Changed_Callback)
#
#        # Set the joint type combo box up according to the jointObject
#        jointTypeComboList = ['Undefined'] + ST.JOINT_TYPE
#        self.form.jointType.addItems(jointTypeComboList)
#
#        self.form.jointType.setCurrentIndex(jointTaskObject.JointType + 1)
#
#        # self.form.body1_2B2P.Index() = jointTaskObject.Bi + 1
#        # self.form.body2_2B2P.currentIndex(jointTaskObject.Bj + 1)
#
#        # Copy over the current driver function parameters to the form in case we need them
#        self.parmsToFormF()
#
#        # Make the jointTaskObject "Observed" when the cursor is on it / it is selected
#        CADGui.Selection.addObserver(jointTaskObject)
#    #  -------------------------------------------------------------------------
#    def accept(self):
#        """Run when we press the OK button"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-accept")
#
#        formJointType = self.form.jointType.currentIndex() - 1
#        if formJointType == ST.JOINT_TYPE_DICTIONARY["Revolute"] or \
#                formJointType == ST.JOINT_TYPE_DICTIONARY["Revolute-Revolute"] or \
#                formJointType == ST.JOINT_TYPE_DICTIONARY["Fixed"]:
#            self.jointTaskObject.pointHeadUnitName = ""
#            self.jointTaskObject.pointHeadUnitLabel = ""
#            self.jointTaskObject.pointHeadUnitIndex = -1
#            self.jointTaskObject.pointTailUnitName = ""
#            self.jointTaskObject.pointTailUnitLabel = ""
#            self.jointTaskObject.pointTailUnitIndex = -1
#            # pass
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Translation"]:
#            pass
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Translation-Revolute"]:
#            self.jointTaskObject.pointTailUnitName = ""
#            self.jointTaskObject.pointTailUnitLabel = ""
#            self.jointTaskObject.pointTailUnitIndex = -1
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Disc"]:
#            self.jointTaskObject.pointTailName = ""
#            self.jointTaskObject.pointTailLabel = ""
#            self.jointTaskObject.Pj = -1
#            self.jointTaskObject.pointTailUnitName = ""
#            self.jointTaskObject.pointTailUnitLabel = ""
#            self.jointTaskObject.pointTailUnitIndex = -1
#
#        # Switch off the Task panel
#        GuiDocument = CADGui.getDocument(self.jointTaskObject.Document)
#        GuiDocument.resetEdit()
#
#        # Transfer parms from form to jointTaskObject
#        if self.jointTaskObject.FunctType == 0:
#            self.jointTaskObject.Coeff0 = self.form.FuncACoeff0.value()
#            self.jointTaskObject.Coeff1 = self.form.FuncACoeff1.value()
#            self.jointTaskObject.Coeff2 = self.form.FuncACoeff2.value()
#            self.jointTaskObject.startTimeDriveFunc = self.form.FuncAstartTime.value()
#            self.jointTaskObject.endTimeDriveFunc = self.form.FuncAendTime.value()
#        elif self.jointTaskObject.FunctType == 1:
#            self.jointTaskObject.startTimeDriveFunc = self.form.FuncBstartTime.value()
#            self.jointTaskObject.startValueDriveFunc = self.form.FuncBstartValue.value()
#            self.jointTaskObject.endTimeDriveFunc = self.form.FuncBendTime.value()
#            self.jointTaskObject.endValueDriveFunc = self.form.FuncBendValue.value()
#
#    #  -------------------------------------------------------------------------
#    def hideAllEquationsF(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-hideAllEquations")
#
#        self.form.funcCoeff.setHidden(True)
#        self.form.radioButtonA.setVisible(False)
#        self.form.radioButtonB.setVisible(False)
#        self.form.radioButtonC.setVisible(False)
#        self.form.radioButtonD.setVisible(False)
#        self.form.radioButtonE.setVisible(False)
#        self.form.radioButtonF.setVisible(False)
#
#        # The html image of the equations does not grey out when we set them hidden
#        # So we have two copies of each, the one normal and the one grey
#        # and we switch between the two when we enable/disable the functions
#        self.form.funcAequationGrey.setVisible(True)
#        self.form.funcAequation.setHidden(True)
#        self.form.funcBequationGrey.setVisible(True)
#        self.form.funcBequation.setHidden(True)
#        self.form.funcCequationGrey.setVisible(True)
#        self.form.funcCequation.setHidden(True)
#        self.form.funcDequationGrey.setVisible(True)
#        self.form.funcDequation.setHidden(True)
#        self.form.funcEequationGrey.setVisible(True)
#        self.form.funcEequation.setHidden(True)
#        self.form.funcFequationGrey.setVisible(True)
#        self.form.funcFequation.setHidden(True)
#    #  -------------------------------------------------------------------------
#    def showAllEquationsF(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-showAllEquationsF")
#
#        # We show the options for a revolute / translation driver
#        # if self.form.jointType.currentIndex() == ST.JOINT_TYPE_DICTIONARY["Revolute"] - 1:
#        #     self.form.withRotationDriver.setVisible(True)
#        #     self.form.withRotationDriver.setEnabled(True)
#        # elif self.form.jointType.currentIndex() == ST.JOINT_TYPE_DICTIONARY["Translation"] - 1:
#        #     self.form.withTranslationDriver.setVisible(True)
#        #     self.form.withTranslationDriver.setEnabled(True)
#
#        self.form.funcCoeff.setEnabled(True)
#        self.form.funcCoeff.setVisible(True)
#        self.form.radioButtonA.setVisible(True)
#        self.form.radioButtonB.setVisible(True)
#        self.form.radioButtonC.setVisible(True)
#        self.form.radioButtonD.setVisible(True)
#        self.form.radioButtonE.setVisible(True)
#        self.form.radioButtonF.setVisible(True)
#
#        # The html image of the equations does not grey out when we set them hidden
#        # So we have two copies of each, the one normal and the one grey
#        # and we switch between the two when we enable/disable the functions
#        self.form.funcAequationGrey.setHidden(True)
#        self.form.funcAequation.setVisible(True)
#        self.form.funcBequationGrey.setHidden(True)
#        self.form.funcBequation.setVisible(True)
#        self.form.funcCequationGrey.setHidden(True)
#        self.form.funcCequation.setVisible(True)
#        self.form.funcDequationGrey.setHidden(True)
#        self.form.funcDequation.setVisible(True)
#        self.form.funcEequationGrey.setHidden(True)
#        self.form.funcEequation.setVisible(True)
#        self.form.funcFequationGrey.setHidden(True)
#        self.form.funcFequation.setVisible(True)
#    #  -------------------------------------------------------------------------
#    def parmsToFormF(self):
#        """Transfer the applicable parameters from the joint object to the applicable page in the form"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-parmsToFormF")
#
#        # Function type a
#        self.form.FuncACoeff0.setValue(self.jointTaskObject.Coeff0)
#        self.form.FuncACoeff1.setValue(self.jointTaskObject.Coeff1)
#        self.form.FuncACoeff2.setValue(self.jointTaskObject.Coeff2)
#        self.form.FuncAendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#
#        # Function type b
#        self.form.FuncBstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#        self.form.FuncBendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#        self.form.FuncBstartValue.setValue(self.jointTaskObject.startValueDriveFunc)
#        self.form.FuncBendValue.setValue(self.jointTaskObject.endValueDriveFunc)
#
#        # Function type c
#        self.form.FuncCstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#        self.form.FuncCendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#        self.form.FuncCstartValue.setValue(self.jointTaskObject.startValueDriveFunc)
#        self.form.FuncCendDeriv.setValue(self.jointTaskObject.endDerivativeDriveFunc)
#
#        # Function type d
#        self.form.FuncDCoeff0.setValue(self.jointTaskObject.Coeff0)
#        self.form.FuncDCoeff1.setValue(self.jointTaskObject.Coeff1)
#        self.form.FuncDCoeff2.setValue(self.jointTaskObject.Coeff2)
#        self.form.FuncDCoeff3.setValue(self.jointTaskObject.Coeff3)
#        self.form.FuncDCoeff4.setValue(self.jointTaskObject.Coeff4)
#        self.form.FuncDstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#        self.form.FuncDendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#
#        # Function type e
#        self.form.FuncFCoeff0.setValue(self.jointTaskObject.Coeff0)
#        self.form.FuncFCoeff1.setValue(self.jointTaskObject.Coeff1)
#        self.form.FuncFCoeff2.setValue(self.jointTaskObject.Coeff2)
#        self.form.FuncFCoeff3.setValue(self.jointTaskObject.Coeff3)
#        self.form.FuncFCoeff4.setValue(self.jointTaskObject.Coeff4)
#        self.form.FuncFCoeff5.setValue(self.jointTaskObject.Coeff5)
#        self.form.FuncFstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#        self.form.FuncFendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#
#        # Function type f
#        self.form.FuncFCoeff0.setValue(self.jointTaskObject.Coeff0)
#        self.form.FuncFCoeff1.setValue(self.jointTaskObject.Coeff1)
#        self.form.FuncFCoeff2.setValue(self.jointTaskObject.Coeff2)
#        self.form.FuncFCoeff3.setValue(self.jointTaskObject.Coeff3)
#        self.form.FuncFCoeff4.setValue(self.jointTaskObject.Coeff4)
#        self.form.FuncFCoeff5.setValue(self.jointTaskObject.Coeff5)
#        self.form.FuncFstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#        self.form.FuncFendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#    #  -------------------------------------------------------------------------
#    def withRotationDriver_Changed_Callback(self):
#        if self.form.withRotationDriver.isChecked() == False:
#            self.jointTaskObject.FunctType = -1
#            self.hideAllEquationsF()
#        else:
#            self.showAllEquationsF()
#            # if self.form.radioButtonA.isChecked():
#            #     self.jointTaskObject.FunctType = 0
#
#            # elif self.form.radioButtonB.isChecked():
#            #     self.jointTaskObject.FunctType = 1
#            # elif self.form.radioButtonC.isChecked():
#            #     self.jointTaskObject.FunctType = 2
#            # elif self.form.radioButtonD.isChecked():
#            #     self.jointTaskObject.FunctType = 3
#            # elif self.form.radioButtonE.isChecked():
#            #     self.jointTaskObject.FunctType = 4
#            # elif self.form.radioButtonF.isChecked():
#            #     self.jointTaskObject.FunctType = 5
#    #  -------------------------------------------------------------------------
#    def withTranslationDriver_Changed_Callback(self):
#        if self.form.withTranslationDriver.isChecked() == False:
#            self.jointTaskObject.FunctType = -1
#            self.hideAllEquationsF()
#        else:
#            self.showAllEquationsF()
#            if self.form.radioButtonA.isChecked():
#                self.jointTaskObject.FunctType = 0
#            elif self.form.radioButtonB.isChecked():
#                self.jointTaskObject.FunctType = 1
#            elif self.form.radioButtonC.isChecked():
#                self.jointTaskObject.FunctType = 2
#            elif self.form.radioButtonD.isChecked():
#                self.jointTaskObject.FunctType = 3
#            elif self.form.radioButtonE.isChecked():
#                self.jointTaskObject.FunctType = 4
#            elif self.form.radioButtonF.isChecked():
#                self.jointTaskObject.FunctType = 5
#    #  -------------------------------------------------------------------------
#    def driveFunc_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-driveFunc_Changed_Callback")
#
#        # Set the current page of the funcCoeff stacked widget
#        self.form.funcCoeff.setEnabled(True)
#        self.showAllEquationsF()
#    #  -------------------------------------------------------------------------
#    def jointType_Changed_Callback(self):
#        """When we have _Changed the joint movement type"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-jointType_Changed_Callback")
#
#        formJointType = self.form.jointType.currentIndex() - 1
#        if self.form.jointType.currentIndex() > 0:
#            self.jointTaskObject.JointType = formJointType
#
#        # Set up which page of body definition we must see
#        # Pages in the joint dialog
#        # 0 - 2Bodies 2Points [rev rev-rev Fixed]
#        # 1 - 2Bodies 4Points [trans]
#        # 2 - 2Bodies 3Points [trans-rev]
#        # 3 - 1Body 1Point [driven-rev]
#        # 4 - 1Body 2Points [driven-trans]
#        # 5 - Body Disc [disc]
#        # 6 - expansion
#        #   And whether the driven function stuff is available
#        #   And populate the body combo boxes in the applicable page
#        if formJointType == ST.JOINT_TYPE_DICTIONARY["Revolute"]:
#            # ST.initComboInFormF(self.form.body1_2B2P, self.bodyLabels, -1)
#            # ST.initComboInFormF(self.form.body2_2B2P, self.bodyLabels, -1)
#            ST.initComboInFormF(self.form.body1_2B2P, self.bodyLabels, self.jointTaskObject.Bi)
#            ST.initComboInFormF(self.form.body2_2B2P, self.bodyLabels, self.jointTaskObject.Bj)
#            self.form.definitionWidget.setCurrentIndex(0)
#            self.form.withRotationDriver.setVisible(True)
#            self.form.withRotationDriver.setEnabled(True)
#            self.form.withTranslationDriver.setVisible(False)
#            self.form.withTranslationDriver.setEnabled(False)
#            # self.hideAllEquationsF()
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Revolute-Revolute"] or \
#                formJointType == ST.JOINT_TYPE_DICTIONARY["Fixed"]:
#            # ST.initComboInFormF(self.form.body1_2B2P, self.bodyLabels, -1)
#            # ST.initComboInFormF(self.form.body2_2B2P, self.bodyLabels, -1)
#            ST.initComboInFormF(self.form.body1_2B2P, self.bodyLabels, self.jointTaskObject.Bi)
#            ST.initComboInFormF(self.form.body2_2B2P, self.bodyLabels, self.jointTaskObject.Bj)
#            self.form.definitionWidget.setCurrentIndex(0)
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withTranslationDriver.setVisible(False)
#            self.hideAllEquationsF()
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Translation"]:
#            # ST.initComboInFormF(self.form.body1_2B4P, self.bodyLabels, -1)
#            # ST.initComboInFormF(self.form.body2_2B4P, self.bodyLabels, -1)
#            ST.initComboInFormF(self.form.body1_2B4P, self.bodyLabels, self.jointTaskObject.Bi)
#            ST.initComboInFormF(self.form.body2_2B4P, self.bodyLabels, self.jointTaskObject.Bj)
#            self.form.definitionWidget.setCurrentIndex(1)
#            self.form.withTranslationDriver.setVisible(True)
#            self.form.withTranslationDriver.setEnabled(True)
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withRotationDriver.setEnabled(False)
#            self.hideAllEquationsF()
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Translation-Revolute"]:
#            # ST.initComboInFormF(self.form.body1_2B3P, self.bodyLabels, -1)
#            # ST.initComboInFormF(self.form.body2_2B3P, self.bodyLabels, -1)
#            ST.initComboInFormF(self.form.body1_2B3P, self.bodyLabels, self.jointTaskObject.Bi)
#            ST.initComboInFormF(self.form.body2_2B3P, self.bodyLabels, self.jointTaskObject.Bj)
#            self.form.definitionWidget.setCurrentIndex(2)
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withTranslationDriver.setVisible(False)
#            self.hideAllEquationsF()
#        elif formJointType == ST.JOINT_TYPE_DICTIONARY["Disc"]:
#            ST.initComboInFormF(self.form.bodyDisc, self.bodyLabels, -1)
#            self.form.withRotationDriver.setVisible(False)
#            self.form.withTranslationDriver.setVisible(False)
#            self.form.definitionWidget.setCurrentIndex(5)
#            self.hideAllEquationsF()
#    #  -------------------------------------------------------------------------
#    def body_1B1P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body_1B1P_Changed_Callback")
#
#        bodyIndex = self.form.body_1B1P.currentIndex() - 1
#        self.jointTaskObject.Bi = bodyIndex
#        if bodyIndex == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            updateToolTipF(self.form.body_1B1P, ['Undefined'])
#            ST.initComboInFormF(self.form.point_1B1P, ['Undefined'], -1)
#            updateToolTipF(self.form.point_1B1P, ['Undefined'])
#        else:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#            updateToolTipF(self.form.body_1B1P, self.pointLabelListFirstBody)
#            ST.initComboInFormF(self.form.point_1B1P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.point_1B1P, self.pointLabelListFirstBody)
#    #  -------------------------------------------------------------------------
#    def point_1B1P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-point_1B1P_Changed_Callback")
#
#        pointIndex = self.form.point_1B1P.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.point_1B1P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.point_1B1P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def body_1B2P_Changed_Callback(self):
#        """In the case of unit vectors, the joint point is the tail of the vector
#        and the other point, the head"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body_1B2P_Changed_Callback")
#
#        bodyIndex = self.form.body_1B2P.currentIndex() - 1
#        self.jointTaskObject.Bi = bodyIndex
#        if bodyIndex == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            updateToolTipF(self.form.body_1B2P, ['Undefined'])
#            ST.initComboInFormF(self.form.vectorHead_1B2P, ['Undefined'], -1)
#            updateToolTipF(self.form.vectorHead_1B2P, ['Undefined'])
#            ST.initComboInFormF(self.form.vectorTail_1B2P, ['Undefined'], -1)
#            updateToolTipF(self.form.vectorTail_1B2P, ['Undefined'])
#        else:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#            updateToolTipF(self.form.body_1B2P, self.pointLabelListFirstBody)
#            ST.initComboInFormF(self.form.vectorHead_1B2P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vectorHead_1B2P, self.pointLabelListFirstBody)
#            ST.initComboInFormF(self.form.vectorTail_1B2P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vectorTail_1B2P, self.pointLabelListFirstBody)
#    #  ------------------------------------------------------------------------
#    def vectorTail_1B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vectorHead_1B2P_Changed_Callback")
#
#        pointIndex = self.form.vectorTail_1B2P.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.vectorTail_1B2P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vectorTail_1B2P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def vectorHead_1B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vectorTail_1B2P_Changed_Callback")
#
#        pointIndex = self.form.vectorHead_1B2P.currentIndex() - 1
#        self.jointTaskObject.pointHeadUnitIndex = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadUnitName = ""
#            self.jointTaskObject.pointHeadUnitLabel = ""
#            updateToolTipF(self.form.vectorHead_1B2P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadUnitName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadUnitLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vectorHead_1B2P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def body1_2B2P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimjointClass-body1_2B2P_Changed_Callback")
#
#        newBodyIndex = self.form.body1_2B2P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bi == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            # self.jointTaskObject.Pi = -1
#            updateToolTipF(self.form.body1_2B2P, ['Undefined'])
#            # ST.initComboInFormF(self.form.point1_2B2P, ['Undefined'], -1)
#            # updateToolTipF(self.form.point1_2B2P, ['Undefined'])
#            ST.initComboInFormF(self.form.point1_2B2P, self.pointLabelListFirstBody, -1)
#        # elif newBodyIndex > -1:
#        if newBodyIndex > -1:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body1_2B2P, self.pointLabelListFirstBody)
#            if newBodyIndex == self.jointTaskObject.Bi:
#                ST.initComboInFormF(self.form.point1_2B2P, self.pointLabelListFirstBody, self.jointTaskObject.Pi)
#            else:
#                ST.initComboInFormF(self.form.point1_2B2P, self.pointLabelListFirstBody, -1)
#                # self.jointTaskObject.Pi = -1
#            updateToolTipF(self.form.point1_2B2P, self.pointLabelListFirstBody)
#            self.jointTaskObject.Bi = newBodyIndex
#        # else:
#        if self.jointTaskObject.Bi > -1:
#            self.form.body1_2B2P.setCurrentIndex(self.jointTaskObject.Bi + 1)
#
#    # --------------------------------------------------------------------------
#    def body2_2B2P_Changed_Callback(self):
#        """Populate form with a new list of point labels when a body is Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimjointClass-body2_2B2P_Changed_Callback")
#
#        newBodyIndex = self.form.body2_2B2P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bj == -1:
#            self.jointTaskObject.body_J_Name = ""
#            self.jointTaskObject.body_J_Label = ""
#            updateToolTipF(self.form.body2_2B2P, ['Undefined'])
#            # ST.initComboInFormF(self.form.point2_2B2P, ['Undefined'], -1)
#            updateToolTipF(self.form.point2_2B2P, ['Undefined'])
#            ST.initComboInFormF(self.form.point2_2B2P, self.pointLabelListSecondBody, -1)
#            # self.jointTaskObject.Pj = -1
#        # elif newBodyIndex > -1:
#        if newBodyIndex > -1:
#            self.pointNameListSecondBody, self.pointLabelListSecondBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_J_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_J_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body2_2B2P, self.pointLabelListSecondBody)
#            if newBodyIndex == self.jointTaskObject.Bj:
#                ST.initComboInFormF(self.form.point2_2B2P, self.pointLabelListSecondBody, self.jointTaskObject.Pj)
#            else:
#                ST.initComboInFormF(self.form.point2_2B2P, self.pointLabelListSecondBody, -1)
#                # self.jointTaskObject.Pj = -1
#            updateToolTipF(self.form.point2_2B2P, self.pointLabelListSecondBody)
#            self.jointTaskObject.Bj = newBodyIndex
#        # else:
#        if self.jointTaskObject.Bj > -1:
#            self.form.body2_2B2P.setCurrentIndex(self.jointTaskObject.Bj + 1)
#
#    #  -------------------------------------------------------------------------
#    def point1_2B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-point1_2B2P_Changed_Callback")
#
#        pointIndex = self.form.point1_2B2P.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.point1_2B2P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.point1_2B2P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def point2_2B2P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-point2_2B2P_Changed_Callback")
#
#        pointIndex = self.form.point2_2B2P.currentIndex() - 1
#        self.jointTaskObject.Pj = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointTailName = ""
#            self.jointTaskObject.pointTailLabel = ""
#            updateToolTipF(self.form.point2_2B2P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointTailName = self.pointNameListSecondBody[pointIndex]
#            self.jointTaskObject.pointTailLabel = self.pointLabelListSecondBody[pointIndex]
#            updateToolTipF(self.form.point2_2B2P, [self.pointLabelListSecondBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def body1_2B3P_Changed_Callback(self):
#        """The Body2 combo box in the Translation joint page current index has _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body1_2B3P_Changed_Callback")
#
#        # bodyIndex = self.form.body1_2B3P.currentIndex() - 1
#        # self.jointTaskObject.Bi = bodyIndex
#        # if bodyIndex == -1:
#        #     self.jointTaskObject.body_I_Name = ""
#        #     self.jointTaskObject.body_I_Label = ""
#        #     updateToolTipF(self.form.body1_2B3P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vectorHead_2B3P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vectorHead_2B3P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vectorTail_2B3P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vectorTail_2B3P, ['Undefined'])
#
#        newBodyIndex = self.form.body1_2B3P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bi == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            updateToolTipF(self.form.body1_2B3P, ['Undefined'])
#            ST.initComboInFormF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vectorTail_2B3P, ['Undefined'])
#            ST.initComboInFormF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vectorHead_2B3P, ['Undefined'])
#
#        # else:
#        #     self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#        #         ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        #     self.jointTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        #     self.jointTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        #     updateToolTipF(self.form.body1_2B3P, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody)
#
#        if newBodyIndex > -1:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body1_2B3P, self.pointLabelListFirstBody)
#            if newBodyIndex == self.jointTaskObject.Bi:
#                ST.initComboInFormF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody, self.jointTaskObject.Pi)
#                ST.initComboInFormF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody, self.jointTaskObject.pointHeadUnitIndex)
#            else:
#                ST.initComboInFormF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody, -1)
#                ST.initComboInFormF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vectorTail_2B3P, self.pointLabelListFirstBody)
#            updateToolTipF(self.form.vectorHead_2B3P, self.pointLabelListFirstBody)
#            self.jointTaskObject.Bi = newBodyIndex
#
#        if self.jointTaskObject.Bi > -1:
#            self.form.body1_2B3P.setCurrentIndex(self.jointTaskObject.Bi + 1)
#
#    #  -------------------------------------------------------------------------
#    def body2_2B3P_Changed_Callback(self):
#        """The Body II Revolute combo box current index has _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body2_2B3P_Changed_Callback")
#
#        # bodyIndex = self.form.body2_2B3P.currentIndex() - 1
#        # self.jointTaskObject.Bj = bodyIndex
#        # if bodyIndex == -1:
#        #     self.jointTaskObject.body_J_Name = ""
#        #     self.jointTaskObject.body_J_Label = ""
#        #     updateToolTipF(self.form.body2_2B3P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.point_2B3P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.point_2B3P, ['Undefined'])
#
#        newBodyIndex = self.form.body2_2B3P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bj == -1:
#            self.jointTaskObject.body_J_Name = ""
#            self.jointTaskObject.body_J_Label = ""
#            updateToolTipF(self.form.body2_2B3P, ['Undefined'])
#            ST.initComboInFormF(self.form.point_2B3P, self.pointLabelListSecondBody, -1)
#            updateToolTipF(self.form.point_2B3P, ['Undefined'])
#
#        # else:
#        #     self.pointNameListSecondBody, self.pointLabelListSecondBody = \
#        #         ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        #     self.jointTaskObject.body_J_Name = self.bodyNames[bodyIndex]
#        #     self.jointTaskObject.body_J_Label = self.bodyLabels[bodyIndex]
#        #     updateToolTipF(self.form.body2_2B3P, self.pointLabelListSecondBody)
#        #     ST.initComboInFormF(self.form.point_2B3P, self.pointLabelListSecondBody, -1)
#        #     updateToolTipF(self.form.point_2B3P, self.pointLabelListSecondBody)
#
#        if newBodyIndex > -1:
#            self.pointNameListSecondBody, self.pointLabelListSecondBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_J_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_J_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body2_2B3P, self.pointLabelListSecondBody)
#            if newBodyIndex == self.jointTaskObject.Bj:
#                ST.initComboInFormF(self.form.point_2B3P, self.pointLabelListSecondBody, self.jointTaskObject.Pj)
#            else:
#                ST.initComboInFormF(self.form.point_2B3P, self.pointLabelListSecondBody, -1)
#            updateToolTipF(self.form.point_2B3P, self.pointLabelListSecondBody)
#            self.jointTaskObject.Bj = newBodyIndex
#
#        if self.jointTaskObject.Bj > -1:
#            self.form.body2_2B3P.setCurrentIndex(self.jointTaskObject.Bj + 1)
#
#
#    #  -------------------------------------------------------------------------
#    def vectorTail_2B3P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vectorHead_2B3P_Changed_Callback")
#
#        pointIndex = self.form.vectorTail_2B3P.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.vectorTail_2B3P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vectorTail_2B3P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def vectorHead_2B3P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vectorTail_2B3P_Changed_Callback")
#
#        pointIndex = self.form.vectorHead_2B3P.currentIndex() - 1
#        self.jointTaskObject.pointHeadUnitIndex = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadUnitName = ""
#            self.jointTaskObject.pointHeadUnitLabel = ""
#            updateToolTipF(self.form.vectorHead_2B3P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadUnitName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadUnitLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vectorHead_2B3P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def point_2B3P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-point_2B3P_Changed_Callback")
#
#        pointIndex = self.form.point_2B3P.currentIndex() - 1
#        self.jointTaskObject.Pj = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointTailName = ""
#            self.jointTaskObject.pointTailLabel = ""
#            updateToolTipF(self.form.point_2B3P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointTailName = self.pointNameListSecondBody[pointIndex]
#            self.jointTaskObject.pointTailLabel = self.pointLabelListSecondBody[pointIndex]
#            updateToolTipF(self.form.point_2B3P, [self.pointLabelListSecondBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def body1_2B4P_Changed_Callback(self):
#        """The Body1 Translation combo box in the Translation joint page current index has _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body1_2B4P_Changed_Callback")
#
#        # newBodyIndex = self.form.body1_2B4P.currentIndex() - 1
#        # self.jointTaskObject.Bi = newBodyIndex
#        # if newBodyIndex == -1:
#        #     self.jointTaskObject.body_I_Name = ""
#        #     self.jointTaskObject.body_I_Label = ""
#        #     updateToolTipF(self.form.body1_2B4P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vector1Head_2B4P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vector1Head_2B4P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vector1Tail_2B4P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vector1Tail_2B4P, ['Undefined'])
#
#        newBodyIndex = self.form.body1_2B4P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bi == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            updateToolTipF(self.form.body1_2B4P, ['Undefined'])
#            ST.initComboInFormF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vector1Tail_2B4P, ['Undefined'])
#            ST.initComboInFormF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vector1Head_2B4P, ['Undefined'])
#
#        # else:
#        #     self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#        #         ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#        #     self.jointTaskObject.body_I_Name = self.bodyNames[newBodyIndex]
#        #     self.jointTaskObject.body_I_Label = self.bodyLabels[newBodyIndex]
#        #     updateToolTipF(self.form.body1_2B4P, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody)
#
#        if newBodyIndex > -1:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body1_2B4P, self.pointLabelListFirstBody)
#            if newBodyIndex == self.jointTaskObject.Bi:
#                ST.initComboInFormF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody, self.jointTaskObject.Pi)
#                ST.initComboInFormF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody, self.jointTaskObject.pointHeadUnitIndex)
#            else:
#                ST.initComboInFormF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody, -1)
#                ST.initComboInFormF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.vector1Tail_2B4P, self.pointLabelListFirstBody)
#            updateToolTipF(self.form.vector1Head_2B4P, self.pointLabelListFirstBody)
#            self.jointTaskObject.Bi = newBodyIndex
#
#        if self.jointTaskObject.Bi > -1:
#            self.form.body1_2B4P.setCurrentIndex(self.jointTaskObject.Bi + 1)
#
#    #  -------------------------------------------------------------------------
#    def body2_2B4P_Changed_Callback(self):
#        """The Body2 combo box in the Translation joint page current index has _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-body2_2B4P_Changed_Callback")
#
#        # newBodyIndex = self.form.body2_2B4P.currentIndex() - 1
#        # self.jointTaskObject.Bj = newBodyIndex
#        # if newBodyIndex == -1:
#        #     self.jointTaskObject.body_J_Name = ""
#        #     self.jointTaskObject.body_J_Label = ""
#        #     updateToolTipF(self.form.body2_2B4P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vector2Tail_2B4P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vector2Tail_2B4P, ['Undefined'])
#        #     ST.initComboInFormF(self.form.vector2Head_2B4P, ['Undefined'], -1)
#        #     updateToolTipF(self.form.vector2Head_2B4P, ['Undefined'])
#
#        newBodyIndex = self.form.body2_2B4P.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bj == -1:
#            self.jointTaskObject.body_J_Name = ""
#            self.jointTaskObject.body_J_Label = ""
#            updateToolTipF(self.form.body2_2B4P, ['Undefined'])
#            ST.initComboInFormF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody, -1)
#            updateToolTipF(self.form.vector2Tail_2B4P, ['Undefined'])
#            ST.initComboInFormF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody, -1)
#            updateToolTipF(self.form.vector2Head_2B4P, ['Undefined'])
#
#        # else:
#        #     self.pointNameListSecondBody, self.pointLabelListSecondBody = \
#        #         ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#        #     self.jointTaskObject.body_J_Name = self.bodyNames[newBodyIndex]
#        #     self.jointTaskObject.body_J_Label = self.bodyLabels[newBodyIndex]
#        #     updateToolTipF(self.form.body2_2B4P, self.pointLabelListSecondBody)
#        #     ST.initComboInFormF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody, -1)
#        #     updateToolTipF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody)
#        #     ST.initComboInFormF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody, -1)
#        #     updateToolTipF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody)
#
#        if newBodyIndex > -1:
#            self.pointNameListSecondBody, self.pointLabelListSecondBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_J_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_J_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.body2_2B4P, self.pointLabelListSecondBody)
#            if newBodyIndex == self.jointTaskObject.Bj:
#                ST.initComboInFormF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody, self.jointTaskObject.Pj)
#                ST.initComboInFormF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody, self.jointTaskObject.pointTailUnitIndex)
#            else:
#                ST.initComboInFormF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody, -1)
#                ST.initComboInFormF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody, -1)
#            updateToolTipF(self.form.vector2Tail_2B4P, self.pointLabelListSecondBody)
#            updateToolTipF(self.form.vector2Head_2B4P, self.pointLabelListSecondBody)
#            self.jointTaskObject.Bj = newBodyIndex
#
#        if self.jointTaskObject.Bj > -1:
#            self.form.body2_2B4P.setCurrentIndex(self.jointTaskObject.Bj + 1)
#
#    #  -------------------------------------------------------------------------
#    def vector1Tail_2B4P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vector1Head_2B4P_Changed_Callback")
#
#        pointIndex = self.form.vector1Tail_2B4P.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.vector1Tail_2B4P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vector1Tail_2B4P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def vector1Head_2B4P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vector1Tail_2B4P_Changed_Callback")
#
#        pointIndex = self.form.vector1Head_2B4P.currentIndex() - 1
#        self.jointTaskObject.pointHeadUnitIndex = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadUnitName = ""
#            self.jointTaskObject.pointHeadUnitLabel = ""
#            updateToolTipF(self.form.vector1Head_2B4P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadUnitName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadUnitLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.vector1Head_2B4P, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def vector2Tail_2B4P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vector2Head_2B4P_Changed_Callback")
#
#        pointIndex = self.form.vector2Tail_2B4P.currentIndex() - 1
#        self.jointTaskObject.Pj = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointTailName = ""
#            self.jointTaskObject.pointTailLabel = ""
#            updateToolTipF(self.form.vector2Tail_2B4P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointTailName = self.pointNameListSecondBody[pointIndex]
#            self.jointTaskObject.pointTailLabel = self.pointLabelListSecondBody[pointIndex]
#            updateToolTipF(self.form.vector2Tail_2B4P, [self.pointLabelListSecondBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def vector2Head_2B4P_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-vector2Tail_2B4P_Changed_Callback")
#
#        pointIndex = self.form.vector2Head_2B4P.currentIndex() - 1
#        self.jointTaskObject.pointTailUnitIndex = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointTailUnitName = ""
#            self.jointTaskObject.pointTailUnitLabel = ""
#            updateToolTipF(self.form.vector2Head_2B4P, ['Undefined'])
#        else:
#            self.jointTaskObject.pointTailUnitName = self.pointNameListSecondBody[pointIndex]
#            self.jointTaskObject.pointTailUnitLabel = self.pointLabelListSecondBody[pointIndex]
#            updateToolTipF(self.form.vector2Head_2B4P, [self.pointLabelListSecondBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def bodyDisc_Changed_Callback(self):
#        """The Body Disc combo box in the Disc joint page current index has _Changed"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-bodyDisc_Changed_Callback")
#
#        # bodyIndex = self.form.bodyDisc.currentIndex() - 1
#        # self.jointTaskObject.Bi = bodyIndex
#        # if bodyIndex == -1:
#        #     self.jointTaskObject.body_I_Name = ""
#        #     self.jointTaskObject.body_I_Label = ""
#        #     updateToolTipF(self.form.bodyDisc, ['Undefined'])
#        #     ST.initComboInFormF(self.form.pointDiscCentre, ['Undefined'], -1)
#        #     updateToolTipF(self.form.pointDiscCentre, ['Undefined'])
#        #     ST.initComboInFormF(self.form.pointDiscRim, ['Undefined'], -1)
#        #     updateToolTipF(self.form.pointDiscRim, ['Undefined'])
#
#        newBodyIndex = self.form.bodyDisc.currentIndex() - 1
#        if newBodyIndex == -1 and self.jointTaskObject.Bi == -1:
#            self.jointTaskObject.body_I_Name = ""
#            self.jointTaskObject.body_I_Label = ""
#            updateToolTipF(self.form.bodyDisc, ['Undefined'])
#            ST.initComboInFormF(self.form.pointDiscCentre, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.pointDiscCentre, ['Undefined'])
#            ST.initComboInFormF(self.form.pointDiscRim, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.pointDiscRim, ['Undefined'])
#
#        # else:
#        #     self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#        #         ST.getPointsFromBodyName(self.bodyNames[bodyIndex], self.bodyObjDict)
#        #     self.jointTaskObject.body_I_Name = self.bodyNames[bodyIndex]
#        #     self.jointTaskObject.body_I_Label = self.bodyLabels[bodyIndex]
#        #     updateToolTipF(self.form.bodyDisc, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.pointDiscCentre, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.pointDiscCentre, self.pointLabelListFirstBody)
#        #     ST.initComboInFormF(self.form.pointDiscRim, self.pointLabelListFirstBody, -1)
#        #     updateToolTipF(self.form.pointDiscRim, self.pointLabelListFirstBody)
#
#        if newBodyIndex > -1:
#            self.pointNameListFirstBody, self.pointLabelListFirstBody = \
#                ST.getPointsFromBodyName(self.bodyNames[newBodyIndex], self.bodyObjDict)
#            self.jointTaskObject.body_I_Name = self.bodyNames[newBodyIndex]
#            self.jointTaskObject.body_I_Label = self.bodyLabels[newBodyIndex]
#            updateToolTipF(self.form.bodyDisc, self.pointLabelListFirstBody)
#            if newBodyIndex == self.jointTaskObject.Bi:
#                ST.initComboInFormF(self.form.pointDiscCentre, self.pointLabelListFirstBody, self.jointTaskObject.Pi)
#                ST.initComboInFormF(self.form.pointDiscRim, self.pointLabelListFirstBody, self.jointTaskObject.pointHeadUnitIndex)
#            else:
#                ST.initComboInFormF(self.form.pointDiscCentre, self.pointLabelListFirstBody, -1)
#                ST.initComboInFormF(self.form.pointDiscRim, self.pointLabelListFirstBody, -1)
#            updateToolTipF(self.form.pointDiscCentre, self.pointLabelListFirstBody)
#            updateToolTipF(self.form.pointDiscRim, self.pointLabelListFirstBody)
#            self.jointTaskObject.Bi = newBodyIndex
#        # else:
#        if self.jointTaskObject.Bi > -1:
#            self.form.bodyDisc.setCurrentIndex(self.jointTaskObject.Bi + 1)
#
#    #  -------------------------------------------------------------------------
#    def pointDiscCentre_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-pointDiscCentre_Changed_Callback")
#
#        pointIndex = self.form.pointDiscCentre.currentIndex() - 1
#        self.jointTaskObject.Pi = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadName = ""
#            self.jointTaskObject.pointHeadLabel = ""
#            updateToolTipF(self.form.pointDiscCentre, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.pointDiscCentre, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def pointDiscRim_Changed_Callback(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-pointDiscRim_Changed_Callback")
#
#        pointIndex = self.form.pointDiscRim.currentIndex() - 1
#        self.jointTaskObject.pointHeadUnitIndex = pointIndex
#        if pointIndex < 0:
#            self.jointTaskObject.pointHeadUnitName = ""
#            self.jointTaskObject.pointHeadUnitLabel = ""
#            updateToolTipF(self.form.pointDiscRim, ['Undefined'])
#        else:
#            self.jointTaskObject.pointHeadUnitName = self.pointNameListFirstBody[pointIndex]
#            self.jointTaskObject.pointHeadUnitLabel = self.pointLabelListFirstBody[pointIndex]
#            updateToolTipF(self.form.pointDiscRim, [self.pointLabelListFirstBody[pointIndex]])
#    #  -------------------------------------------------------------------------
#    def radioA_Changed_Callback(self):
#        if self.form.radioButtonA.isChecked():
#            self.form.funcCoeff.setCurrentIndex(0)
#
#            if self.jointTaskObject.FunctType == 0:
#                self.form.FuncACoeff0.setValue(self.jointTaskObject.Coeff0)
#                self.form.FuncACoeff1.setValue(self.jointTaskObject.Coeff1)
#                self.form.FuncACoeff2.setValue(self.jointTaskObject.Coeff2)
#                self.form.FuncAstartTime.setValue(self.jointTaskObject.startTimeDriveFunc)
#                self.form.FuncAendTime.setValue(self.jointTaskObject.endTimeDriveFunc)
#            else:
#                # Transfer zero parms into form
#                self.form.FuncACoeff0.setValue(0)
#                self.form.FuncACoeff1.setValue(0)
#                self.form.FuncACoeff2.setValue(0)
#                self.form.FuncAstartTime.setValue(0)
#                self.form.FuncAendTime.setValue(0)
#
#                self.jointTaskObject.FunctType = 0
#
#    #  -------------------------------------------------------------------------
#    def radioB_Changed_Callback(self):
#        if self.form.radioButtonB.isChecked():
#            self.form.funcCoeff.setCurrentIndex(1)
#            self.jointTaskObject.FunctType = 1
#    #  -------------------------------------------------------------------------
#    def radioC_Changed_Callback(self):
#        if self.form.radioButtonC.isChecked():
#            self.form.funcCoeff.setCurrentIndex(2)
#            self.jointTaskObject.FunctType = 2
#    #  -------------------------------------------------------------------------
#    def radioD_Changed_Callback(self):
#        if self.form.radioButtonD.isChecked():
#            self.form.funcCoeff.setCurrentIndex(3)
#            self.jointTaskObject.FunctType = 3
#    #  -------------------------------------------------------------------------
#    def radioE_Changed_Callback(self):
#        if self.form.radioButtonE.isChecked():
#            self.form.funcCoeff.setCurrentIndex(4)
#            self.jointTaskObject.FunctType = 4
#    #  -------------------------------------------------------------------------
#    def radioF_Changed_Callback(self):
#        if self.form.radioButtonF.isChecked():
#            self.form.funcCoeff.setCurrentIndex(5)
#            self.jointTaskObject.FunctType = 5
#    #  -------------------------------------------------------------------------
#    def getStandardButtons(self):
#        """ Set which button will appear at the top of the TaskDialog [Called from FreeCAD]"""
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-getStandardButtons")
#        return QtGui.QDialogButtonBox.Ok
#    #  -------------------------------------------------------------------------
#    def __getstate__(self):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-__getstate__")
#    #  -------------------------------------------------------------------------
#    def __setstate__(self, state):
#        if Debug:
#            ST.Mess("TaskPanelSimJointClass-__setstate__")
