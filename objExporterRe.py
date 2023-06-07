from PySide2.QtWidgets import QFileDialog, QHBoxLayout, QVBoxLayout, QWidget, QApplication, QLabel, QPushButton, \
    QRadioButton, QCheckBox, QLineEdit, QGroupBox
import shutil
import pymel.core as pm

#region window
app = QApplication.instance()  # opens application ..kode inbetween
#Window
wid = QWidget()
wid.setWindowTitle("Obj Exporter")
wid.resize(350, 400)

#UI Elements .inside Window
groupBox_param = QGroupBox("Paramerters")
toogleBox_param = QGroupBox("Spaces")
groupBox_file = QGroupBox("File Save")

file_lable = QLabel("File Name")
line_ed_file = QLineEdit()
export_button = QPushButton("Export")
triangular_check = QCheckBox("Trinagulate Faces")
selection_check = QCheckBox("Export selection")
material_check = QCheckBox("Export Material")
local_rad = QRadioButton("Local Space")
global_rad = QRadioButton("Global Space")

global_rad.setChecked(True)  # set selected
#endregion

#region functions
def collect_Objects(selectedValue):
    if selectedValue:
        return pm.ls(selection=True)
    else:
        return pm.ls(geometry=True)
def triangular_Mesh(value, mesh):
    if value:
        pm.polyTriangulate(mesh)
def get_Vertex_Count(mesh):
    return pm.polyEvaluate(mesh, vertex=True)
def get_Face_Count(mesh):
    return pm.polyEvaluate(mesh, face=True)
def convert_Material_To_ShadingEngine(material):
    return pm.listConnections(material, type='shadingEngine')
def get_Vertex_Position(localValue, mesh):
    vtxPositionList = []
    for vertex in range(get_Vertex_Count(mesh)):
        if localValue:
            vtxPositionList.append(mesh.vtx[vertex].getPosition(space="object"))
        else:
            vtxPositionList.append(mesh.vtx[vertex].getPosition(space="world"))
    return vtxPositionList
def get_UvCoord_Count(mesh):
    return pm.polyEvaluate(mesh, uvcoord=True)
def get_Uv_Coord(mesh):
    uvPositionList = []
    for i in range(get_UvCoord_Count(mesh)):
        uvPositionList.append(mesh.getUV(i))
    return uvPositionList
def get_Vertex_Normal_XYZ_List(value, mesh):
    if value:
        return mesh.getNormals(space='object')
    else:
        return mesh.getNormals(space='world')
def get_All_Materials_From_Face(mesh):
    mat_List = []
    for i in range(get_Face_Count(mesh)):
        pm.select(mesh + ".f[" + str(i) + "]", replace=True)
        pm.hyperShade(shaderNetworksSelectMaterialNodes=True)
        mat = pm.ls(selection=True)
        mat_List.append(mat[0])
    return mat_List
def get_Materials_From_Mesh(mesh):
    return pm.listConnections(pm.listHistory(mesh, future=True), type="lambert")
def get_Material_Color(Material):
    return pm.getAttr(str(Material) + ".color")
def get_Material_Specular(Material):
    try:
        return pm.getAttr(str(Material) + ".specularColor")
    except:
        return None
def get_Material_Transparency(Material):
    return pm.getAttr(str(Material) + ".transparency")
def get_Material_Ambient_Color(Material):
    return pm.getAttr(str(Material) + ".transparency")
def get_Material_Refractive_Index(Material):
    return pm.getAttr(str(Material) + ".refractiveIndex")
def get_Index_From_String(stringValue):
    start = False
    value = ""
    for char in stringValue:
        if char == "[":
            start = True
        if char == "]":
            start = False
        if start and char != "[":
            value += char
    return int(value)
def write_Vertex_Position(vertex_List):
    string_List = []
    for vPosition in vertex_List:
        string_List.append("v " + str(vPosition[0]) + " " + str(vPosition[1]) + " " + str(vPosition[2]))
    return string_List
def write_Vertex_Texture(vertex_UvPosition_List):
    string_List = []
    for uvPosition in vertex_UvPosition_List:
        string_List.append("vt " + str(uvPosition[0]) + " " + str(uvPosition[1]))
    return string_List
def write_Vertex_Nornal(vn_List):
    string_List = []
    for i in range(len(vn_List)):
        string_List.append("vn " + str(vn_List[i][0]) + " " + str(vn_List[i][1]) + " " + str(vn_List[i][2]))
    return string_List
def get_Texture_Path(material):
    hypershaderFileNode = pm.listConnections(str(material) + ".color", type="file")
    if hypershaderFileNode is not None:
        filepath = pm.getAttr(hypershaderFileNode[0] + ".fileTextureName")
        return filepath
def copy_File(path, toPath):
    if toPath[-1] == "/":
        fileName = str(str(path).split("/")[-1])
        newPath = toPath + fileName
        shutil.copyfile(path, newPath)
    else:
        shutil.copyfile(path, toPath)
#endregion

# Main
def exportObj():
    #region filePath

    exported_file = QFileDialog.getSaveFileName(filter=u'Meshes (*.obj)')
    fileName = str(exported_file[0]).split("/")[-1].replace(".obj", "")
    folderPath = str(exported_file[0]).replace(fileName + ".obj", "")

    #endregion

    #meshes
    obj = collect_Objects(selection_check.isChecked())
    # loop Each Mesh
    for mesh in obj:
        #region collectInfo
        #TrinagulateMesh
        triangular_Mesh(triangular_check.isChecked(), mesh)
        # mesh name
        meshName = mesh.longName().split("|")[1]
        # vtx Data
        vtx_Position = get_Vertex_Position(local_rad.isChecked(), mesh)
        vtx_Position_Text_List = write_Vertex_Position(vtx_Position)
        # vt Data
        vt_Coord_List = get_Uv_Coord(mesh)
        vt_Coord_Text_List = write_Vertex_Texture(vt_Coord_List)
        # vn Data
        vn_List = get_Vertex_Normal_XYZ_List(local_rad.isChecked(), mesh)
        vn_Text_List = write_Vertex_Nornal(vn_List)
        # f data
        f_Text_List = []
        # materials
        current_Material = ""
        mesh_Material_List = get_Materials_From_Mesh(meshName)
        face_Material_List = get_All_Materials_From_Face(meshName)
        # smoothing group
        smoothingGroup_Text = "s " + "1"
        #endregion

        #region loopEveryFace
        for i in range(mesh.numFaces()):
            #region getShadingEngine
            # check material
            shadingEngine = convert_Material_To_ShadingEngine(str(face_Material_List[i]))
            if len(shadingEngine) == 1:
                if shadingEngine[0] != current_Material:
                    current_Material = shadingEngine[0]
                    f_Text_List.append("usemtl " + str(current_Material))
            else:
                if shadingEngine[1] != current_Material:
                    current_Material = shadingEngine[1]
                    f_Text_List.append("usemtl " + str(current_Material))
            #endregion

            #face
            f_Text_List.append("f ")
            for k in range(len(mesh.f[i].getVertices())):
                # vertex
                f_Text_List[-1] += str(mesh.f[i].getVertices()[k] + 1) + "/"
                # vertex Texture
                f_Text_List[-1] += str(mesh.f[i].getUVIndex(k) + 1) + "/"
                # vertex Normal
                f_Text_List[-1] += str(mesh.f[i].normalIndex(k) + 1) + " "
        #endregion

        #region writeObj
        # w for write a for append
        myFile = open(str(exported_file[0]), "a")  #create if not exist
        myFile.write("#@by Liye Yan \n")
        myFile.write("\n")
        myFile.write("mtllib " + str(fileName) + ".mtl" + "\n")
        myFile.write("g " + "default" + "\n")
        #write vertex position
        for v in vtx_Position_Text_List:
            myFile.write(v + "\n")
        #write vertex texture coordinate
        for vt in vt_Coord_Text_List:
            myFile.write(vt + "\n")
        #write vertex normal
        for vn in vn_Text_List:
            myFile.write(vn + "\n")
        #smoothing
        myFile.write(smoothingGroup_Text + "\n")
        #group
        myFile.write("g " + meshName + "\n")
        #write face
        for f in f_Text_List:
            myFile.write(f + "\n")

        myFile.close()
        #endregion

        # region Mtl
        #mtl check
        if material_check.isChecked():
            #create mtl file
            myMtlFile = open(folderPath + fileName + ".mtl", "a")  # create if not exist
            myMtlFile.write("#@by Liye Yan \n")
            myMtlFile.write("\n")
            #loop every Material
            for mat in mesh_Material_List:
                # get shading engine from material
                shadingEngine = convert_Material_To_ShadingEngine(str(mat))
                # 1 element
                if len(shadingEngine) == 1:
                    myMtlFile.write("newmtl " + str(shadingEngine[0]) + "\n")
                #default lambert1 has 2 element
                else:
                    myMtlFile.write("newmtl " + str(shadingEngine[1]) + "\n")

                #region writeMaterialValue

                #write illum Value
                myMtlFile.write("illum 4\n")

                #write material color
                diffuse_reflectivity_color = get_Material_Color(mat)
                myMtlFile.write("Kd " + str(diffuse_reflectivity_color[0]) + " " + str(diffuse_reflectivity_color[1]) + " " + str(diffuse_reflectivity_color[2]) + "\n")

                #write material ambient reflectivity color
                ambient_reflectivity_color = get_Material_Ambient_Color(mat)
                myMtlFile.write("Ka " + str(ambient_reflectivity_color[0]) + " " + str(ambient_reflectivity_color[1]) + " " + str(ambient_reflectivity_color[2]) + "\n")

                #write specular
                specular = get_Material_Specular(mat)
                if specular is not None:
                    myMtlFile.write("Ks " + str(specular[0]) + " " + str(specular[1]) + " " + str(specular[2]) + "\n")

                #write Transmission filters(transparency)
                transparency = get_Material_Transparency(mat)
                myMtlFile.write("Tf " + str(1 - transparency[0]) + " " + str(1 - transparency[1]) + " " + str(1 - transparency[2]) + "\n")

                #Optical density(index of refraction)
                refraction = get_Material_Refractive_Index(mat)
                myMtlFile.write("Ni " + str(refraction) + "\n")
                #endregion

                #region copyTexture
                #get file path
                texturePath = get_Texture_Path(mat)
                # check texture path,none = no texture
                if texturePath is not None:
                    copy_File(texturePath, folderPath)
                    texName = str(texturePath).split("/")[-1]
                    myMtlFile.write("map_Kd " + str(texName) + "\n")
                #endregion

                myMtlFile.close()
        # endregion

    print "\n------------------------\n"

#region addWidget

# parameters
parameters = QHBoxLayout()
parameters.addWidget(triangular_check)
parameters.addWidget(selection_check)
parameters.addWidget(material_check)
groupBox_param.setLayout(parameters)
#space
spaceToggle = QHBoxLayout()
spaceToggle.addWidget(local_rad)
spaceToggle.addWidget(global_rad)
toogleBox_param.setLayout(spaceToggle)

# saveFile
fileParameter = QVBoxLayout()
fileParameter.addWidget(file_lable)
fileParameter.addWidget(line_ed_file)
fileParameter.addWidget(export_button)
groupBox_file.setLayout(fileParameter)

# window
windowBox = QVBoxLayout()
windowBox.addWidget(groupBox_param)
windowBox.addWidget(toogleBox_param)
windowBox.addWidget(groupBox_file)
wid.setLayout(windowBox)

wid.show()  # showes the window

export_button.clicked.connect(exportObj)

app.exec_()  # ends application

#endregion
