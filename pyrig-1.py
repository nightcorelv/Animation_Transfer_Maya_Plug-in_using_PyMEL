from PySide2 import QtCore
from PySide2 import QtWidgets
from PySide2 import QtGui
from functools import partial

import pymel.core
import pymel.core.datatypes

jointLS = []
jointLT = []

# window
wind = QtWidgets.QWidget()


# buttons
run = QtWidgets.QPushButton("Transfer", wind)
deleteSource = QtWidgets.QPushButton("Remove", wind)
deleteTarget = QtWidgets.QPushButton("Remove", wind)
upSource = QtWidgets.QPushButton("UP", wind)
upTarget = QtWidgets.QPushButton("UP", wind)
downSource = QtWidgets.QPushButton("Down", wind)
downTarget = QtWidgets.QPushButton("Down", wind)
clearSource = QtWidgets.QPushButton("Clear", wind)
clearTarget = QtWidgets.QPushButton("Clear", wind)
addSelectionSource = QtWidgets.QPushButton("Add Selection", wind)
addSelectionTarget = QtWidgets.QPushButton("Add Selection", wind)

# list widget
listSource = QtWidgets.QListWidget(wind)
listTarget = QtWidgets.QListWidget(wind)

# listSource.addItem(listItem)
# listSource.insertItem(0, "ggop")

# labels
sourceName = QtWidgets.QLabel("None             ", wind)
targetName = QtWidgets.QLabel("None             ", wind)

# line edit
fromF = QtWidgets.QLineEdit(wind)
toF = QtWidgets.QLineEdit(wind)

offset = QtWidgets.QLineEdit(wind)

intValidator = QtGui.QIntValidator(-9999, 99999)

#flipCheckBox = QtWidgets.QCheckBox("Flip",wind)
invertCheckBox = QtWidgets.QCheckBox("Invert",wind)
inPlaceCheckBox = QtWidgets.QCheckBox("in place",wind)

def getChildren(root, childList):
    listTemp = childList
    for child in root.getChildren():
        listTemp.append(child)
        if child.numChildren() > 0:
            getChildren(child, listTemp)
    return listTemp

def getNextChild(root):
    return root.getChildren[0]

def displayJoints(jointList, listView):
    for joint in jointList:
        # separate parent names
        tree = str(joint)
        # apply last index to list view
        listView.addItem(tree.split("|")[-1])

def getAllParents(joint):
    parents = []
    while joint:
        parents.append(joint)
        joint = joint.getParent()
    parents.pop(0)
    return parents

def getBindPose(parents):
    result = 1  # set 1 to keep first value
    for parent in parents:
        if parent:
            result = parent.getRotation().asMatrix() * parent.getOrientation().asMatrix() * result
    return result

def transfer():
    # return if list is empty
    if len(jointLS) == 0 or len(jointLT) == 0:
        return

    RMDS_zero = {}
    OMDS_zero = {}

    RMDT_zero = {}
    OMDT_zero = {}

    parentDS = {}
    parentDT = {}

    bindPoseDS = {}
    bindPoseDT = {}

    isoRDS = {}

    matchDT = {}

    # start with frame 0 to calculate bind pose
    setCurrentTimeline(0)

    for joint in jointLS:
        # put frame 0 source rotation matrix to dic
        RMDS_zero[joint] = joint.getRotation().asMatrix()
        # put frame 0 source orientation matrix to dic
        OMDS_zero[joint] = joint.getOrientation().asMatrix()
        # put all parents for source to dic
        parentDS[joint] = getAllParents(joint)
        # calculate and put bind pose matrix for source to dic
        result = getBindPose(parentDS[joint]) # set 1 to keep first value
        bindPoseDS[joint] = result

    for joint in jointLT:
        # put frame 0 target rotation matrix to dic
        RMDT_zero[joint] = joint.getRotation().asMatrix()
        # put frame 0 target orientation matrix to dic
        OMDT_zero[joint] = joint.getOrientation().asMatrix()
        # put all parents for target to dic
        parentDT[joint] = getAllParents(joint)
        # calculate and put bind pose matrix for source to dic
        result = getBindPose(parentDT[joint]) # set 1 to keep first value
        bindPoseDT[joint] = result

    # put target joint match source joint to dic
    for i in range(len(jointLT)):
        matchDT[jointLT[i]] = jointLS[i]


    #loop from start frame to end frame
    for currentFrame in range(int(fromF.text()),int(toF.text()) + 1):
        # set current frame
        setCurrentTimeline(currentFrame)

        RMDS_current = {}
        OMDS_current = {}

        RMDT_current = {}
        OMDT_current = {}

        worldRMDS = {}

        rootT = jointLS[0].getTranslation()
        rootR = jointLS[0].getRotation()

        if not inPlaceCheckBox.isChecked():
            setCurrentTimeline(currentFrame + int(offset.text()))
            # set target root translation
            jointLT[0].setTranslation(rootT)
            # set target root rotation
            jointLT[0].setRotation(rootR)
            # set key frame
            pymel.core.setKeyframe(jointLT[0])
            setCurrentTimeline(currentFrame)

        for joint in jointLS:
            if joint != jointLS[0]:
                # put current frame source rotation matrix to dic
                RMDS_current[joint] = joint.getRotation().asMatrix()
                # put current frame source orientation matrix to dic
                OMDS_current[joint] = joint.getOrientation().asMatrix()
                # calculate isolated rotation for source
                isoRDS[joint] = RMDS_zero[joint].inverse() * RMDS_current[joint]
                # calculate world space rotation
                worldRMDS[joint] = OMDS_current[joint].inverse() * bindPoseDS[joint].inverse() * isoRDS[joint] * bindPoseDS[joint] * OMDS_current[joint]

        for joint in jointLT:
            if joint != jointLT[0]:
                # put current frame target rotation matrix to dic
                RMDT_current[joint] = joint.getRotation().asMatrix()
                # put current frame target orientation matrix to dic
                OMDT_current[joint] = joint.getOrientation().asMatrix()
                # calculate final rotation in world space
                finalR = OMDT_current[joint] * bindPoseDT[joint] * worldRMDS[matchDT[joint]] * bindPoseDT[joint].inverse() * OMDT_current[joint].inverse()
                # convert to target coordinate system rotation matrix
                convertedRM = RMDT_zero[joint] * finalR

                setCurrentTimeline(currentFrame + int(offset.text()))
                if invertCheckBox.isChecked():
                    joint.setRotation(pymel.core.datatypes.EulerRotation(convertedRM.inverse()))
                else:
                    joint.setRotation(pymel.core.datatypes.EulerRotation(convertedRM))
                #if flipCheckBox.isChecked():
                #    r = joint.getRotation()
                #    joint.r.set(r[0] * -1, r[1], r[2] * -1)

                pymel.core.setKeyframe(joint)
                setCurrentTimeline(currentFrame)



def setCurrentTimeline(value):
    pymel.core.currentTime(value)

def getCurrentTimeline():
    print(pymel.core.currentTime())

def remove(jointList, listView):
    selectedIndex = listView.currentRow()
    if selectedIndex != -1:
        selectedName = listView.takeItem(selectedIndex).text()
        jointList.remove(selectedName)

def up(jointList, listView):
    selectedIndex = listView.currentRow()
    if(selectedIndex > 0):
        # swap list element
        jointList[selectedIndex], jointList[selectedIndex - 1] = jointList[selectedIndex - 1], jointList[selectedIndex]
        # get current item
        currentItem = listView.takeItem(selectedIndex)
        # insert to -1
        listView.insertItem(selectedIndex - 1, currentItem)
        listView.setCurrentRow(selectedIndex - 1,)


def down(jointList, listView):
    selectedIndex = listView.currentRow()
    if (selectedIndex < len(jointList) - 1):
        # swap list element
        jointList[selectedIndex], jointList[selectedIndex + 1] = jointList[selectedIndex + 1], jointList[selectedIndex]
        # get current item
        currentItem = listView.takeItem(selectedIndex)
        # insert to +1
        listView.insertItem(selectedIndex + 1, currentItem)
        listView.setCurrentRow(selectedIndex + 1)

def clear(jointList, listView, label):
    jointList.clear()
    label.setText("None             ")
    listView.clear()


def addSelection(jointList, listView, label):
    root = pymel.core.ls(sl = True)[0]
    jointList.append(root)
    jointList = getChildren(root, jointList)
    label.setText(str(root))
    displayJoints(jointList, listView)






# window
width = 600
height = 600
#wind.setWindowFlags(QtCore.Qt.WindowStaysOnBottomHint)
wind.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
wind.setFixedSize(width, height)
wind.setWindowTitle("Motion Transfer")

# run Button
runWidth = 75
runHeight = 25
run.resize(runWidth, runHeight)
run.move(width - runWidth - 10, height - runHeight - 10)

#check box
#flipCheckBox.move(350, height - 28)
invertCheckBox.move(520, height - 65)
inPlaceCheckBox.move(520, height - 85)
inPlaceCheckBox.setChecked(True)

# line edit
fromDisplay = QtWidgets.QLabel("From Key: ", wind)
fromDisplay.move(10, height - 28)

toDisplay = QtWidgets.QLabel("To Key: ", wind)
toDisplay.move(175, height - 28)

offsetDisplay = QtWidgets.QLabel("Offset: ", wind)
offsetDisplay.move(325, height - 28)
#
fromF.setValidator(intValidator)
toF.setValidator(intValidator)
offset.setValidator(intValidator)
fromF.move(65, height - 30)
toF.move(215, height - 30)
offset.move(365, height - 30)
fromF.setText("1")
toF.setText("60")
offset.setText("0")


# button
bWidth = 75
bHeight = 25
# delete Source Button
deleteSource.resize(bWidth, bHeight)
deleteSource.move(10, height - bHeight - 225)

# delete Target Button
deleteTarget.resize(bWidth, bHeight)
deleteTarget.move(width - bWidth - 10, height - bHeight - 225)

# up Source Button
upSource.resize(bWidth, bHeight)
upSource.move(10, height / 2 - bHeight - 35)

# up Target Button
upTarget.resize(bWidth, bHeight)
upTarget.move(width - bWidth - 10, height / 2 - bHeight - 35)

# down Source Button
downSource.resize(bWidth, bHeight)
downSource.move(10, height / 2 - bHeight - 0)

# down Target Button
downTarget.resize(bWidth, bHeight)
downTarget.move(width - bWidth - 10, height / 2 - bHeight - 0)

# clear Source Button
clearSource.resize(bWidth, bHeight)
clearSource.move(135, height - bHeight - 70)

# clear Target Button
clearTarget.resize(bWidth, bHeight)
clearTarget.move(width - bWidth - 135, height - bHeight - 70)

# add Selection Source Button
addSelectionSource.resize(bWidth, bHeight)
addSelectionSource.move(135, 60)

# add Selection Target Button
addSelectionTarget.resize(bWidth, bHeight)
addSelectionTarget.move(width - bWidth - 135, 60)

# connect
run.clicked.connect(transfer)

deleteSource.clicked.connect(partial(remove, jointLS, listSource))
deleteTarget.clicked.connect(partial(remove, jointLT, listTarget))

upSource.clicked.connect(partial(up, jointLS, listSource))
upTarget.clicked.connect(partial(up, jointLT, listTarget))

downSource.clicked.connect(partial(down, jointLS, listSource))
downTarget.clicked.connect(partial(down, jointLT, listTarget))

clearSource.clicked.connect(partial(clear, jointLS, listSource, sourceName))
clearTarget.clicked.connect(partial(clear, jointLT, listTarget, targetName))

addSelectionSource.clicked.connect(partial(addSelection, jointLS, listSource, sourceName))
addSelectionTarget.clicked.connect(partial(addSelection, jointLT, listTarget, targetName))

# list view
lWidth = 160
lHeight = 375
#font
boldFont = QtGui.QFont()
boldFont.setBold(True)
boldFont.setPointSize(16)
#
leftLabel = QtWidgets.QLabel(">>>", wind)
leftLabel.setFont(boldFont)
leftLabel.move(280, 290)

# list Widget Source
listSource.setFixedSize(lWidth, lHeight)
listSource.move(bWidth + 20, 125)

# list Widget Target
listTarget.setFixedSize(lWidth, lHeight)
listTarget.move(width - bWidth - lWidth - 10 - 10, 125)

# Display
sourceDisplay = QtWidgets.QLabel("Source: ", wind)
targetDisplay = QtWidgets.QLabel("Target: ", wind)
sourceDisplay.move(bWidth + 20, 100)
targetDisplay.move(width - bWidth - lWidth - 10 - 10, 100)

# joint name
sourceName.move(bWidth + 65, 100)
targetName.move(width - bWidth - lWidth + 25, 100)

wind.show()
