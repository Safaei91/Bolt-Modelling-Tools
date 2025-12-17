from abaqus import *
from abaqusConstants import *
from driverUtils import *
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from odbAccess import openOdb
import sys
import os
import shutil
import json

def mdlbld(geom_prop, mat_prop, target_disp, imperfection=False, imper_prop=None, fracture=False, job_number=1):
    # Geometry:____________________________________________________________________
    if geom_prop["sec_type"] == "circ":
        g = 0.5*geom_prop["G"]
        d = 0.5*geom_prop["D"]
        if geom_prop["spec_type"] == "dog_bone":
            l = 0.5*geom_prop["L"]
            b = 0.5*geom_prop["B"]
            w = 0.5*geom_prop["W"]
            r = geom_prop["r"]

    elif geom_prop["sec_type"] == "rect":
        g = 0.5*geom_prop["G"]
        d = 0.5*geom_prop["D"]
        t = 0.5*geom_prop["T"]
        if geom_prop["spec_type"] == "dog_bone":
            l = 0.5*geom_prop["L"]
            b = 0.5*geom_prop["B"]
            w = 0.5*geom_prop["W"]
            r = geom_prop["r"]

    executeOnCaeStartup()

    if geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "prism":
        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
            -100.0), point2=(0.0, 100.0))
        mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
            mdb.models['Model-1'].sketches['__profile__'].geometry[2])
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
            point2=(d, g))
        mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
            DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['Part-1'].BaseSolidRevolve(angle=90.0, 
            flipRevolveDirection=OFF, sketch=
            mdb.models['Model-1'].sketches['__profile__'])
        del mdb.models['Model-1'].sketches['__profile__']
    
    elif geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "prism":
        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
            point2=(d, g))
        mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
            DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['Part-1'].BaseSolidExtrude(depth=t, sketch=
            mdb.models['Model-1'].sketches['__profile__'])
        del mdb.models['Model-1'].sketches['__profile__']

    elif geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "dog_bone":
        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
            -100.0), point2=(0.0, 100.0))
        mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
            mdb.models['Model-1'].sketches['__profile__'].geometry[2])
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
            point2=(w, l-b-r))
        mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
            addUndoState=False, entity1=
            mdb.models['Model-1'].sketches['__profile__'].vertices[0], entity2=
            mdb.models['Model-1'].sketches['__profile__'].geometry[2])
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, l-b-r), 
            point2=(d, l))
        mdb.models['Model-1'].sketches['__profile__'].autoTrimCurve(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[4], point1=(
            0.5*d, l-b-r))
        mdb.models['Model-1'].sketches['__profile__'].autoTrimCurve(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[10], point1=(
            0.5*d, l-b-r))
        mdb.models['Model-1'].sketches['__profile__'].FilletByRadius(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[9], curve2=
            mdb.models['Model-1'].sketches['__profile__'].geometry[11], nearPoint1=(
            0.5*w, l-b-r), nearPoint2=(w, 
            l-0.5*b), radius=r)
        mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
            DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['Part-1'].BaseSolidRevolve(angle=90.0, 
            flipRevolveDirection=OFF, sketch=
            mdb.models['Model-1'].sketches['__profile__'])
        del mdb.models['Model-1'].sketches['__profile__']

    elif geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "dog_bone":
        mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
            point2=(w, l-b-r))
        mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, l-b-r), 
            point2=(d, l))
        mdb.models['Model-1'].sketches['__profile__'].autoTrimCurve(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[3], point1=(
            0.5*d, l-b))
        mdb.models['Model-1'].sketches['__profile__'].autoTrimCurve(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[9], point1=(
            0.5*d, l-b))
        mdb.models['Model-1'].sketches['__profile__'].FilletByRadius(curve1=
            mdb.models['Model-1'].sketches['__profile__'].geometry[10], curve2=
            mdb.models['Model-1'].sketches['__profile__'].geometry[8], nearPoint1=(
            0.5*w, l-b-r), nearPoint2=(d, l-0.5*b), radius=r)
        mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
            DEFORMABLE_BODY)
        mdb.models['Model-1'].parts['Part-1'].BaseSolidExtrude(depth=t, sketch=
            mdb.models['Model-1'].sketches['__profile__'])
        del mdb.models['Model-1'].sketches['__profile__']
    
    if geom_prop["spec_type"] == "dog_bone":
        mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=l-b-r, 
        principalPlane=XZPLANE)
        mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=l-b, 
        principalPlane=XZPLANE)
        # mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=d, 
        # principalPlane=YZPLANE)
        mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#1 ]', ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[2])
        mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#2 ]', ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[3])
        # mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
        # mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#7 ]', ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[4])
    # Material:____________________________________________________________________
    E = mat_prop["E"]
    SSP = mat_prop["SSP"]

    mdb.models['Model-1'].Material(name='Material-1')
    mdb.models['Model-1'].materials['Material-1'].Density(table=((7.85e-09,),))
    mdb.models['Model-1'].materials['Material-1'].Elastic(table=((E, 0.3),))
    mdb.models['Model-1'].materials['Material-1'].Plastic(table=SSP)

    mdb.models['Model-1'].HomogeneousSolidSection(
        material='Material-1', name='Section-1', thickness=None)
    mdb.models['Model-1'].parts['Part-1'].Set(
        cells=mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#1 ]',),),
        name='Set-1')
    mdb.models['Model-1'].parts['Part-1'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
    cells=mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(
    mask=('[#1f ]', ), )), sectionName='Section-1', thicknessAssignment=
    FROM_SECTION)

    # Assembly:____________________________________________________________________
    mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
    mdb.models['Model-1'].rootAssembly.Instance(
        dependent=OFF, name='Part-1-1', part=mdb.models['Model-1'].parts['Part-1'])

    # Step:________________________________________________________________________
    mdb.models['Model-1'].StaticStep(
        initialInc=0.01, maxInc=0.01, minInc=0.0001, name='Step-1', nlgeom=ON, previous='Initial')
    mdb.models['Model-1'].steps['Step-1'].setValues(maxNumInc=100000)
    mdb.models['Model-1'].steps['Step-1'].setValues(adaptiveDampingRatio=0.05, 
    continueDampingFactors=False, matrixSolver=DIRECT, matrixStorage=
    UNSYMMETRIC, minInc=1e-14, stabilizationMagnitude=0.0002, 
    stabilizationMethod=DISSIPATED_ENERGY_FRACTION)

    # Boundary Conditions:_________________________________________________________
    if geom_prop["spec_type"] == "prism":
        if geom_prop["sec_type"] == "circ":
            bottom_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=g, zMin=0, xMax=d, yMax=g, zMax=d)
            mdb.models['Model-1'].rootAssembly.Set(faces=bottom_surf, name='Base_Face')
        elif geom_prop["sec_type"] == "rect":
            bottom_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=g, zMin=0, xMax=d, yMax=g, zMax=t)
            mdb.models['Model-1'].rootAssembly.Set(faces=bottom_surf, name='Base_Face')

    elif geom_prop["spec_type"] == "dog_bone":
        if geom_prop["sec_type"] == "circ":
            bottom_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=l, zMin=0, xMax=d, yMax=l, zMax=d)
            mdb.models['Model-1'].rootAssembly.Set(faces=bottom_surf, name='Base_Face')
        elif geom_prop["sec_type"] == "rect":
            bottom_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=l, zMin=0, xMax=d, yMax=l, zMax=t)
            mdb.models['Model-1'].rootAssembly.Set(faces=bottom_surf, name='Base_Face')

    mdb.models['Model-1'].DisplacementBC(
        amplitude=UNSET, createStepName='Initial', distributionType=UNIFORM,
        fieldName='', localCsys=None, name='BC-1', region=mdb.models['Model-1'].rootAssembly.sets['Base_Face'],
        u1=UNSET, u2=SET, u3=UNSET, ur1=SET, ur2=SET, ur3=SET)

    # YX Plain:____________________________________________________________________
    if geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "prism":
        yx_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=d, yMax=g, zMax=0)
        mdb.models['Model-1'].rootAssembly.Set(faces=yx_surf, name='YX_Face')

    elif geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "prism":
        yx_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=d, yMax=g, zMax=0)
        mdb.models['Model-1'].rootAssembly.Set(faces=yx_surf, name='YX_Face')
    
    elif geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "dog_bone":
        yx_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=w, yMax=l, zMax=0)
        mdb.models['Model-1'].rootAssembly.Set(faces=yx_surf, name='YX_Face')
    
    elif geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "dog_bone":
        yx_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=w, yMax=l, zMax=0)
        mdb.models['Model-1'].rootAssembly.Set(faces=yx_surf, name='YX_Face')

    mdb.models['Model-1'].DisplacementBC(
        amplitude=UNSET, createStepName='Initial', distributionType=UNIFORM,
        fieldName='', localCsys=None, name='BC-2', region=mdb.models['Model-1'].rootAssembly.sets['YX_Face'],
        u1=UNSET, u2=UNSET, u3=SET, ur1=SET, ur2=SET, ur3=SET)
    
    # YZ Plain:____________________________________________________________________    
    if geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "prism":
        yz_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=0, yMax=g, zMax=t)
        mdb.models['Model-1'].rootAssembly.Set(faces=yz_surf, name='YZ_Face')

    elif geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "prism":
        yz_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=0, yMax=g, zMax=d)
        mdb.models['Model-1'].rootAssembly.Set(faces=yz_surf, name='YZ_Face')
    
    elif geom_prop["sec_type"] == "rect" and geom_prop["spec_type"] == "dog_bone":
        yz_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=0, yMax=l, zMax=t)
        mdb.models['Model-1'].rootAssembly.Set(faces=yz_surf, name='YZ_Face')
    
    elif geom_prop["sec_type"] == "circ" and geom_prop["spec_type"] == "dog_bone":
        yz_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
        xMin=0, yMin=0, zMin=0, xMax=0, yMax=l, zMax=w)
        mdb.models['Model-1'].rootAssembly.Set(faces=yz_surf, name='YZ_Face')

    mdb.models['Model-1'].DisplacementBC(
        amplitude=UNSET, createStepName='Initial', distributionType=UNIFORM,
        fieldName='', localCsys=None, name='BC-3', region=mdb.models['Model-1'].rootAssembly.sets['YZ_Face'],
        u1=SET, u2=UNSET, u3=UNSET, ur1=SET, ur2=SET, ur3=SET)

    # Top surface: ________________________________________________________________
    if geom_prop["spec_type"] == "prism":
        if geom_prop["sec_type"] == "circ":
            top_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=0, zMin=0, xMax=d, yMax=0, zMax=d)
            mdb.models['Model-1'].rootAssembly.Set(faces=top_surf, name='Top_Face')
        elif geom_prop["sec_type"] == "rect":
            top_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=0, zMin=0, xMax=d, yMax=0, zMax=t)
            mdb.models['Model-1'].rootAssembly.Set(faces=top_surf, name='Top_Face')

    elif geom_prop["spec_type"] == "dog_bone":
        if geom_prop["sec_type"] == "circ":
            top_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=0, zMin=0, xMax=w, yMax=0, zMax=w)
            mdb.models['Model-1'].rootAssembly.Set(faces=top_surf, name='Top_Face')
        elif geom_prop["sec_type"] == "rect":
            top_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=0, yMin=0, zMin=0, xMax=w, yMax=0, zMax=t)
            mdb.models['Model-1'].rootAssembly.Set(faces=top_surf, name='Top_Face')

    mdb.models['Model-1'].DisplacementBC(
        amplitude=UNSET, createStepName='Step-1', distributionType=UNIFORM,
        fixed=OFF, localCsys=None, name='BC-4', region=mdb.models['Model-1'].rootAssembly.sets['Top_Face'],
        u1=UNSET, u2=-target_disp, u3=UNSET, ur1=0.0, ur2=0.0, ur3=0.0)

    # Meshing:_____________________________________________________________________
    if geom_prop["spec_type"] == "prism":
        mdb.models['Model-1'].rootAssembly.seedPartInstance(deviationFactor=0.1, 
            minSizeFactor=0.1, regions=(
            mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], ), size=geom_prop["mesh_size"])
        mdb.models['Model-1'].rootAssembly.deleteMesh(regions=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(
        ('[#1 ]', ), ))
        mdb.models['Model-1'].rootAssembly.setMeshControls(regions=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(('[#1 ]', ), ), technique=STRUCTURED)
        mdb.models['Model-1'].rootAssembly.generateMesh(regions=(mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], ))
    elif geom_prop["spec_type"] == "dog_bone":
        mdb.models['Model-1'].rootAssembly.seedPartInstance(deviationFactor=0.1, 
            minSizeFactor=0.1, regions=(
            mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], ), size=geom_prop["mesh_size"])
        mdb.models['Model-1'].rootAssembly.deleteMesh(regions=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(
        ('[#1 ]', ), ))
        mdb.models['Model-1'].rootAssembly.setMeshControls(algorithm=MEDIAL_AXIS, 
        elemShape=HEX_DOMINATED, regions=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(
        ('[#7 ]', ), ), technique=SWEEP)
        mdb.models['Model-1'].rootAssembly.generateMesh(regions=(
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], ))



    # Imperfection:________________________________________________________________
    if imperfection:
        imper_type = imper_prop["type"]
        imper_length = imper_prop["length"]
        reduction = imper_prop["reduction"]

        def imper_cal(x, y, z, red, imper_length, imper_type):
            red = red * float(d)
            if imper_type == "linear":
                delta = red * (y + imper_length - g) / imper_length

            elif imper_type == "poly":
                delta = red * (3*((y + imper_length - g)/imper_length)**2 - 2*((y + imper_length - g)/imper_length)**3)

            if y >= g - imper_length:
                if geom_prop["sec_type"] == "circ":
                    r_node = (x**2 + z**2)**0.5
                    delta_r = delta * r_node / float(d)
                    delta_x = x / float(d) * delta_r
                    delta_z = z / float(d) * delta_r
                elif geom_prop["sec_type"] == "rect":
                    delta_x = delta / float(d) * x
                    delta_z = 0

                delta_y = 0
            else:
                delta_x = 0
                delta_y = 0
                delta_z = 0
            

            return delta_x, delta_y, delta_z

        nodes = mdb.models["Model-1"].rootAssembly.instances['Part-1-1'].nodes
        
        for node in nodes:
            cords = node.coordinates
            x, y, z = cords[0], cords[1], cords[2]
            delta_x, delta_y, delta_z = imper_cal(x, y, z, reduction, imper_length, imper_type)
            new_cords = (x - delta_x, y - delta_y, z - delta_z)
            mdb.models['Model-1'].rootAssembly.editNode(
                coordinate1=new_cords[0], coordinate2=new_cords[1],
                coordinate3=new_cords[2], nodes=node)
        
        print("Nodes adjusted for imperfection.")

    print("The model has been created.")

    # Creating the Job__________________________________________________________________________________
    mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
    explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
    memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
    multiprocessingMode=DEFAULT, name='Job-Main-' + str(job_number), nodalOutputPrecision=SINGLE, 
    numCpus=4, numDomains=4, numGPUs=0, queue=None, resultsFormat=ODB, scratch=
    '', type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)

    # RUNNING THE ANALYSIS:______________________________________________________________________
    print("The analysis has been started.")

    mdb.jobs['Job-Main-' + str(job_number)].submit(consistencyChecking=OFF)
    
    mdb.jobs['Job-Main-' + str(job_number)].waitForCompletion()

    print("The analysis has been finished!")

    # closing the odb: __________________________________________________________________________
    # mdb.saveAs("Model.cae")

    mdb.close()

    # Reading the data: _________________________________________________________________________
    odb = openOdb("Job-Main-"+ str(job_number) +".odb")

    top_surf_nodes = odb.rootAssembly.nodeSets['TOP_FACE']

    frames = odb.steps["Step-1"].frames

    all_nodes = odb.rootAssembly.instances['PART-1-1'].nodes
    for node in all_nodes:
        if node.coordinates[0] == 0.0 and node.coordinates[1] == 0.0 and node.coordinates[2] == 0.0:
            target_node = node.label
            break

    all_elements = odb.rootAssembly.instances['PART-1-1'].elements
    for element in all_elements:
        if target_node in element.connectivity:
            target_element = element.label
            break

    F_react = []
    Uy = []
    S_misez = []
    S11 = []
    S22 = []
    S33 = []
    PEEQ = []
    for f in frames:
        RF_values = f.fieldOutputs["RF"].getSubset(region=top_surf_nodes).values
        F_react.append(-sum([float(rf.data[1]) for rf in RF_values]))
        Uy.append(-float(f.fieldOutputs["U"].getSubset(region=top_surf_nodes).values[0].data[1]))

        # Extract stress and PEEQ directly using the element label
        stress_field = f.fieldOutputs["S"]
        peeq_field = f.fieldOutputs["PEEQ"]

        for s in stress_field.values:
            if s.elementLabel == target_element:
                S_misez.append(float(s.mises))
                S11.append(float(s.data[0]))
                S22.append(float(s.data[1]))
                S33.append(float(s.data[2]))
                break
        
        for e in peeq_field.values:
            if e.elementLabel == target_element:
                PEEQ.append(float(e.data))
                break
    
    F_D = {
            "F": F_react,
            "D": Uy,
            }
    
    S_E = {
            "S_mises": S_misez,
            "S11": S11,
            "S22": S22,
            "S33": S33,
            "PEEQ": PEEQ
            }
    
    # Save mat_prop to a JSON file
    with open("F_D.json", "w") as f:
        json.dump(F_D, f)

    # Save mat_prop to a JSON file
    with open("S_E.json", "w") as f:
        json.dump(S_E, f)

    # closing the odb file: ___________________________________________________________________________________
    odb.close()

    # Deleting the odb and cae files: _________________________________________________________________________
    # os.remove("Model.cae")

    os.remove("Job-Main-"+ str(job_number) +".odb")

    # Check if lock file exists and delete it
    lock_file = "Model.lck"

    if os.path.exists(lock_file):
        
        os.remove(lock_file)
    
    # Close Abaqus/CAE
    sys.exit()

with open("mat_prop.json", "r") as f:
    mat_prop = json.load(f)

with open("geom_prop.json", "r") as f:
    geom_prop = json.load(f)

with open("disp.json", "r") as f:
    disp = json.load(f)

with open("iter.json", "r") as f:
    iter = json.load(f)

mdlbld(geom_prop=geom_prop, mat_prop=mat_prop,
       target_disp=disp, imperfection=False,
       imper_prop={"type": "poly", "length": geom_prop["D"], "reduction": 0.001},
       job_number=iter)