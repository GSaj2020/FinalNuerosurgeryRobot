import os
import unittest
import slicer, vtk, qt, ctk, random
import sitkUtils as su
import SimpleITK as sitk
from slicer.ScriptedLoadableModule import *
import logging
import numpy
import math


#
# PathPlanner
#

class PathPlanner(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "Path Planner"
        self.parent.categories = ["Path Planner"]
        self.parent.dependencies = []
        self.parent.contributors = ["Ezzat Aiman, King's College London"]
        self.parent.helpText = """ This is an algorithm that performs a path planning between selected entry points 
        and target points within the boundary of hard constraints and optimization applied.
        """
        self.parent.acknowledgementText = """
This file was originally developed by Ezzat Aiman, King's College London, and funded by his own effort and dedication 
to finish this coursework and become a good student.
"""


#
# PPWidget
#

class PathPlannerWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """


    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)

        # Instantiate and connect widgets ...

        #
        # Parameters Area
        #
        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Parameters"
        self.layout.addWidget(parametersCollapsibleButton)

        # Layout within the dummy collapsible button
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        #
        # input volume selector
        #
        self.MainImageSelector = slicer.qMRMLNodeComboBox()
        self.MainImageSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.MainImageSelector.selectNodeUponCreation = True
        self.MainImageSelector.addEnabled = False
        self.MainImageSelector.removeEnabled = False
        self.MainImageSelector.noneEnabled = False
        self.MainImageSelector.showHidden = False
        self.MainImageSelector.showChildNodeTypes = False
        self.MainImageSelector.setMRMLScene(slicer.mrmlScene)
        self.MainImageSelector.setToolTip("Select the input volume.")
        parametersFormLayout.addRow("Target structure: ", self.MainImageSelector)

        #
        # input obstacle volume selector
        #
        self.Obstacle1Selector = slicer.qMRMLNodeComboBox()
        self.Obstacle1Selector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.Obstacle1Selector.selectNodeUponCreation = True
        self.Obstacle1Selector.addEnabled = False
        self.Obstacle1Selector.removeEnabled = False
        self.Obstacle1Selector.noneEnabled = False
        self.Obstacle1Selector.showHidden = False
        self.Obstacle1Selector.showChildNodeTypes = False
        self.Obstacle1Selector.setMRMLScene(slicer.mrmlScene)
        self.Obstacle1Selector.setToolTip("Select an obstacle file.")
        parametersFormLayout.addRow("Obstacle volume: ", self.Obstacle1Selector)

        #
        # input entry points
        #
        self.EntryFidSelector = slicer.qMRMLNodeComboBox()
        self.EntryFidSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.EntryFidSelector.selectNodeUponCreation = True
        self.EntryFidSelector.addEnabled = False
        self.EntryFidSelector.removeEnabled = False
        self.EntryFidSelector.noneEnabled = False
        self.EntryFidSelector.showHidden = False
        self.EntryFidSelector.showChildNodeTypes = False
        self.EntryFidSelector.setMRMLScene(slicer.mrmlScene)
        self.EntryFidSelector.setToolTip("Pick the input entry fiducials to the algorithm.")
        parametersFormLayout.addRow("Entry points: ", self.EntryFidSelector)

        #
        # input target points
        #
        self.TargetFidSelector = slicer.qMRMLNodeComboBox()
        self.TargetFidSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.TargetFidSelector.selectNodeUponCreation = True
        self.TargetFidSelector.addEnabled = False
        self.TargetFidSelector.removeEnabled = False
        self.TargetFidSelector.noneEnabled = False
        self.TargetFidSelector.showHidden = False
        self.TargetFidSelector.showChildNodeTypes = False
        self.TargetFidSelector.setMRMLScene(slicer.mrmlScene)
        self.TargetFidSelector.setToolTip("Pick the input target fiducials to the algorithm.")
        parametersFormLayout.addRow("Target points: ", self.TargetFidSelector)

        #
        # Apply Button
        #
        self.RunButton = qt.QPushButton("Run")
        self.RunButton.toolTip = "Run the algorithm."
        self.RunButton.enabled = False
        parametersFormLayout.addRow(self.RunButton)

        # connections
        self.RunButton.connect('clicked(bool)', self.onApplyButton)
        self.MainImageSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)

        # Add vertical spacer
        self.layout.addStretch(1)

        # Refresh run button state
        self.onSelect()

    def cleanup(self):
        pass

    def onSelect(self):
        self.RunButton.enabled = self.MainImageSelector.currentNode() and self.Obstacle1Selector.currentNode() and \
                                 self.EntryFidSelector.currentNode() and self.TargetFidSelector.currentNode()


    def onApplyButton(self):
        logic = PathPlannerLogic()
        # enableScreenshotsFlag = self.enableScreenshotsFlagCheckBox.checked
        logic.run(self.MainImageSelector.currentNode(), self.Obstacle1Selector.currentNode(),
                  self.EntryFidSelector.currentNode(),self.TargetFidSelector.currentNode())#,
                  #self.outputSelector.currentNode())


#
# PPLogic
#

class PathPlannerLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual computation done by your module.  The interface should be such that
    other python code can import this class and make use of the functionality without requiring an instance of the
    Widget. Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def hasImageData(self, volumeNode):
        """This is an example logic method that
        returns true if the passed in volume
        node has valid image data
        """
        if not volumeNode:
            logging.debug('hasImageData failed: no volume node')
            return False
        if volumeNode.GetImageData() is None:
            logging.debug('hasImageData failed: no image data in volume node')
            return False
        return True



    def run(self, image, obst1, entries, targets):#, targetInside):
        """
        Run the actual algorithm
        """
        logging.info('Processing started')

        targetInside = self.PointsInTarget(image, targets)

        constraintAB_entry, constraintAB_target, constraintABC_entry, constraintABC_target = self.solveConstraints(entries, targetInside, obst1)

        self.optimizeDistance(constraintABC_entry, constraintABC_target, obst1)

        self.DrawLine(constraintAB_entry, constraintAB_target, 'AB_hc')
        self.DrawLine(constraintABC_entry, constraintABC_target, 'ABC_hc')

        logging.info('Processing completed')

        return True



    def PointsInTarget(self, image, targets):

        targetInside = slicer.mrmlScene.AddNode(slicer.vtkMRMLMarkupsFiducialNode())
        targetInside.SetName('TargetInside')

        ijkMat = vtk.vtkMatrix4x4()
        image.GetRASToIJKMatrix(ijkMat)
        transform = vtk.vtkTransform()
        transform.SetMatrix(ijkMat)

        for x in range(0, targets.GetNumberOfFiducials()):
            pos = [0, 0, 0]
            targets.GetNthFiducialPosition(x, pos)
            index = transform.TransformPoint(pos)
            Voxel = image.GetImageData().GetScalarComponentAsDouble(int(index[0]), int(index[1]), int(index[2]), 0)

            # Constraint A: fiducial point is inside the target structure
            if Voxel == 1:
                targetInside.AddFiducial(pos[0], pos[1], pos[2])

        return targetInside



    def solveConstraints(self, entries, targets, obst1):

        obbTree1 = self.obbTree(obst1)
        constraintAB_entry = []
        constraintAB_target = []
        constraintABC_entry = []
        constraintABC_target = []
        threshold = 50

        for i in range(0, entries.GetNumberOfFiducials()):
            entry = numpy.array([0, 0, 0])
            entries.GetNthFiducialPosition(i, entry)

            for j in range(0, targets.GetNumberOfFiducials()):
                target = numpy.array([0, 0, 0])
                targets.GetNthFiducialPosition(j, target)

                # Constraint B: Avoid Critical Structures
                if obbTree1.IntersectWithLine(entry, target, vtk.vtkPoints(), vtk.vtkIdList()):
                    continue
                else:
                    constraintAB_entry.append(entry)
                    constraintAB_target.append(target)
                    length = numpy.sqrt(numpy.sum((entry - target) ** 2))
                    # Constraint C: Maximum length
                    if length < threshold:
                        constraintABC_entry.append(entry)
                        constraintABC_target.append(target)

        return constraintAB_entry, constraintAB_target, constraintABC_entry, constraintABC_target



    def obbTree(self,obstacle):

        mesh = vtk.vtkDiscreteMarchingCubes()
        mesh.SetInputData(obstacle.GetImageData())
        mesh.SetValue(0, 1)
        mesh.Update()


        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(mesh.GetOutput())
        obbTree.BuildLocator()

        return obbTree



    def optimizeDistance(self, constraintABC_entry, constraintABC_target, obst1):

        pathOptimized = slicer.mrmlScene.AddNode(slicer.vtkMRMLMarkupsFiducialNode())
        pathOptimized.SetName('pathOptimized')

        pathOptimized_transformed = slicer.mrmlScene.AddNode(slicer.vtkMRMLMarkupsFiducialNode())
        pathOptimized_transformed.SetName('pathOptimized_transformed')

        obst1_map, transform1 = self.ComputeDistanceMap(obst1)
        obst1_dist_intensities = []


        for i in range(0, len(constraintABC_entry)):

            entry = constraintABC_entry[i]
            target = constraintABC_target[i]

            n = 25

            x = numpy.linspace(entry[0], target[0], n)
            y = numpy.linspace(entry[1], target[1], n)
            z = numpy.linspace(entry[2], target[2], n)

            curr_intensity_1 = 0


            for j in range(0, n - 1):
                pos = [x[j], y[j], z[j]]

                ind1 = transform1.TransformPoint(pos)
                a = obst1_map.GetImageData().GetScalarComponentAsDouble(int(ind1[0]), int(ind1[1]), int(ind1[2]), 0)
                curr_intensity_1 += a


            obst1_dist_intensities.append(curr_intensity_1)


        maxIndex1 = obst1_dist_intensities.index(max(obst1_dist_intensities))

        obst1_entry_optimized = [constraintABC_entry[maxIndex1]]
        obst1_target_optimized = [constraintABC_target[maxIndex1]]
        # Reset the markups if it existed before
        pathOptimized.RemoveAllMarkups()

        pathOptimized.AddFiducial(obst1_entry_optimized[0][0],obst1_entry_optimized[0][1],obst1_entry_optimized[0][2])
        pathOptimized.SetNthFiducialLabel(0,'Entry')

        pathOptimized.AddFiducial(obst1_target_optimized[0][0], obst1_target_optimized[0][1],
                                  obst1_target_optimized[0][2])
        pathOptimized.SetNthFiducialLabel(1, 'Target')

        pathOptimized_transformed.AddFiducial(obst1_entry_optimized[0][0]*-1, obst1_entry_optimized[0][1]*-1,
                                              obst1_entry_optimized[0][2])
        pathOptimized_transformed.SetNthFiducialLabel(0, 'Entry')

        pathOptimized_transformed.AddFiducial(obst1_target_optimized[0][0]*-1, obst1_target_optimized[0][1]*-1,
                                              obst1_target_optimized[0][2])
        pathOptimized_transformed.SetNthFiducialLabel(1, 'Target')



        self.DrawLine(obst1_entry_optimized,obst1_target_optimized, 'ObstacleOptimized')



    def ComputeDistanceMap(self, input):
        sitkInput = su.PullVolumeFromSlicer(input)
        distanceFilter = sitk.DanielssonDistanceMapImageFilter()
        sitkOutput = distanceFilter.Execute(sitkInput)
        outputVolume = su.PushVolumeToSlicer(sitkOutput, None, 'distanceMap')

        mat = vtk.vtkMatrix4x4()
        outputVolume.GetRASToIJKMatrix(mat)

        transform = vtk.vtkTransform()
        transform.SetMatrix(mat)

        return outputVolume, transform



    def DrawLine(self, entry_list, target_list, nodeName):
        lines = vtk.vtkCellArray()
        points = vtk.vtkPoints()

        for i in range(0, len(target_list)):
            entry = entry_list[i]
            target = target_list[i]

            points.InsertNextPoint(entry[0], entry[1], entry[2])
            points.InsertNextPoint(target[0], target[1], target[2])

            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, i)
            line.GetPointIds().SetId(1, i + 1)
            lines.InsertNextCell(line)

        path = vtk.vtkPolyData()
        path.SetPoints(points)
        path.SetLines(lines)
        pathPlan = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', nodeName)
        pathPlan.SetAndObserveMesh(path)


class PathPlannerTest(ScriptedLoadableModuleTest):
    """
This is the test case for your scripted module.
Uses ScriptedLoadableModuleTest base class, available at:
https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
"""

    def setUp(self):
        """ Do whatever is needed to reset the state - typically a scene clear will be enough.
"""
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        """Run as few or as many tests as needed here.
        """
        self.setUp()
        self.test_PP1()

    def test_PP1(self):
        """ Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (  both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")
        #
        # first, get some data
        #
        import SampleData
        SampleData.downloadFromURL(
            nodeNames='FA',
            fileNames='FA.nrrd',
            uris='http://slicer.kitware.com/midas3/download?items=5767')
        self.delayDisplay('Finished with download and loading')

        volumeNode = slicer.util.getNode(pattern="FA")
        logic = PathPlannerLogic()
        self.assertIsNotNone(logic.hasImageData(volumeNode))
        self.delayDisplay('Test passed!')