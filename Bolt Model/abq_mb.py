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
import math

def mdlbld(geom_prop, mat_data, target_disp, target_rot, moment_value, n_strip, mesh_size, job_number):
    
    d_shank = geom_prop["d_shank"]
    d_thread = geom_prop["d_thread"]
    d_head = geom_prop["d_head"]
    d_nut = geom_prop["d_nut"]
    L_shank = geom_prop["L_shank"]
    L_thread = geom_prop["L_thread"]
    L_head = geom_prop["L_head"]
    L_nut = geom_prop["L_nut"]
    L_res = geom_prop["L_res"]

    executeOnCaeStartup()

    # PARTS:______________________________________________________
    # Body:
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
        -100.0), point2=(0.0, 100.0))
    mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, 0.0), point2=(
        0.5*d_thread, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[3])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[3])
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[0], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_thread, 0.0), point2=(
        0.5*d_thread, L_thread+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[4])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[4])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_thread, L_thread+L_res+L_nut), point2=(
        0.5*d_shank, L_thread+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[5])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[4], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[5])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_shank, L_thread+L_res+L_nut), point2=
        (0.5*d_shank, L_thread+L_shank+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[6])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[5], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[6])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_shank, L_thread+L_shank+L_res+L_nut), point2=
        (0.5*d_head, L_thread+L_shank+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[7])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[6], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[7])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_head, L_thread+L_shank+L_res+L_nut), point2=
        (0.5*d_head, L_thread+L_shank+L_head+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[8])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[7], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[8])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.5*d_head, L_thread+L_shank+L_head+L_res+L_nut), point2=
        (0.0, L_thread+L_shank+L_head+L_res+L_nut))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[9])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[8], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[9])
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[7], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, L_thread+L_shank+L_head+L_res+L_nut), point2=(
        0.0, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[10])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[9], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[10])
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
        DEFORMABLE_BODY)
    mdb.models['Model-1'].parts['Part-1'].BaseSolidRevolve(angle=360.0, 
        flipRevolveDirection=OFF, sketch=
        mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']

    # Partition:
    mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=L_res, 
    principalPlane=XZPLANE)
    mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=L_res+L_nut, 
    principalPlane=XZPLANE)
    mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=L_res+L_nut+L_thread, 
    principalPlane=XZPLANE)
    mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=L_res+L_nut+L_thread+L_shank, 
    principalPlane=XZPLANE)
    mdb.models['Model-1'].parts['Part-1'].DatumPlaneByPrincipalPlane(offset=0.0, 
    principalPlane=XYPLANE)

    if n_strip > 2:
        angle_dif = 180.0 / float(n_strip/2)
        angle_cur = 0.0
        for i in range(n_strip/2 - 1):
            angle_cur += angle_dif
            mdb.models['Model-1'].parts['Part-1'].DatumPlaneByRotation(angle=angle_cur, axis=
            mdb.models['Model-1'].parts['Part-1'].datums[1], plane=
            mdb.models['Model-1'].parts['Part-1'].datums[6])
    else:
        mdb.models['Model-1'].parts['Part-1'].DatumPlaneByRotation(angle=90, axis=
        mdb.models['Model-1'].parts['Part-1'].datums[1], plane=
        mdb.models['Model-1'].parts['Part-1'].datums[6])

    mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
    mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#1 ]', 
    ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[2])
    mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
    mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#1 ]', 
    ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[3])
    mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
    mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#1 ]', 
    ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[4])
    mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
    mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(('[#f ]', 
    ), ), datumPlane=mdb.models['Model-1'].parts['Part-1'].datums[6])

    if n_strip > 2:
        angle_dif = 180.0 / float(n_strip/2)
        angle_cur = 0.0
        datum_num = 6
        string = "f"
        for i in range(n_strip/2 - 1):
            angle_cur += angle_dif
            datum_num += 1
            if i > 0:
                string += "fff"

            else:
                string += "f"

            mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
            mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask((
            '[#' + string + ']', ), ), datumPlane=
            mdb.models['Model-1'].parts['Part-1'].datums[datum_num])
    else:
        mdb.models['Model-1'].parts['Part-1'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask((
        '[#' + "ff" + ']', ), ), datumPlane=
        mdb.models['Model-1'].parts['Part-1'].datums[7])

    # Nut:
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
        -100.0), point2=(0.0, 100.0))
    mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.5*d_thread, 0.0), 
        point2=(0.5*d_nut, L_nut))
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-2', type=
        DEFORMABLE_BODY)
    mdb.models['Model-1'].parts['Part-2'].BaseSolidRevolve(angle=360.0, 
        flipRevolveDirection=OFF, sketch=
        mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']

    mdb.models['Model-1'].parts['Part-2'].DatumPlaneByPrincipalPlane(offset=0.0, 
    principalPlane=XYPLANE)

    if n_strip > 2:
        angle_dif = 180.0 / float(n_strip/2)
        angle_cur = 0.0
        for i in range(n_strip/2 - 1):
            angle_cur += angle_dif
            mdb.models['Model-1'].parts['Part-2'].DatumPlaneByRotation(angle=angle_cur, axis=
            mdb.models['Model-1'].parts['Part-2'].datums[1], plane=
            mdb.models['Model-1'].parts['Part-2'].datums[2])
    else:
        mdb.models['Model-1'].parts['Part-2'].DatumPlaneByRotation(angle=90, axis=
        mdb.models['Model-1'].parts['Part-2'].datums[1], plane=
        mdb.models['Model-1'].parts['Part-2'].datums[2])


    mdb.models['Model-1'].parts['Part-2'].PartitionCellByDatumPlane(cells=
    mdb.models['Model-1'].parts['Part-2'].cells.getSequenceFromMask(('[#f ]', 
    ), ), datumPlane=mdb.models['Model-1'].parts['Part-2'].datums[2])

    if n_strip > 2:
        angle_dif = 180.0 / float(n_strip/2)
        angle_cur = 0.0
        datum_num = 2
        string = "f"
        for i in range(n_strip/2 - 1):
            angle_cur += angle_dif
            datum_num += 1
            if i > 0:
                string += "fff"

            else:
                string += "f"

            mdb.models['Model-1'].parts['Part-2'].PartitionCellByDatumPlane(cells=
            mdb.models['Model-1'].parts['Part-2'].cells.getSequenceFromMask((
            '[#' + string + ']', ), ), datumPlane=
            mdb.models['Model-1'].parts['Part-2'].datums[datum_num])
    else:
        mdb.models['Model-1'].parts['Part-2'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-2'].cells.getSequenceFromMask((
        '[#' + "ff" + ']', ), ), datumPlane=
        mdb.models['Model-1'].parts['Part-2'].datums[3])

    # MATERIAL:________________________________________________________
    E = mat_data["base"]["E"]
    SSP = mat_data["base"]["SSP"]
    DucDam = mat_data["base"]["DucDam"]
    FracEng = mat_data["base"]["FracEng"]

    mdb.models['Model-1'].Material(name='Material-1')
    mdb.models['Model-1'].materials['Material-1'].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].materials['Material-1'].Elastic(table=((E, 0.3), 
        ))
    mdb.models['Model-1'].materials['Material-1'].Plastic(table=SSP)
    mdb.models['Model-1'].HomogeneousSolidSection(material='Material-1', name=
    'Section-1', thickness=None)

    mdb.models['Model-1'].parts['Part-1'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
    cells=mdb.models['Model-1'].parts['Part-1'].cells.getSequenceFromMask(
    mask=('[#ffffffff #ff ]', ), )), sectionName='Section-1', 
    thicknessAssignment=FROM_SECTION)
    mdb.models['Model-1'].parts['Part-2'].SectionAssignment(offset=0.0, 
    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
    cells=mdb.models['Model-1'].parts['Part-2'].cells.getSequenceFromMask(
    mask=('[#3ff ]', ), )), sectionName='Section-1', thicknessAssignment=
    FROM_SECTION)

    mdb.models['Model-1'].materials['Material-1'].DuctileDamageInitiation(table=DucDam)
    mdb.models['Model-1'].materials['Material-1'].ductileDamageInitiation.DamageEvolution(
    table=((FracEng, ), ), type=ENERGY)

    # ASSEMBLY:________________________________________________________
    mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
    mdb.models['Model-1'].rootAssembly.Instance(dependent=OFF, name='Part-1-1', 
        part=mdb.models['Model-1'].parts['Part-1'])
    mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-2-1', 
        part=mdb.models['Model-1'].parts['Part-2'])
    mdb.models['Model-1'].rootAssembly.translate(instanceList=('Part-2-1', ), 
    vector=(0.0, L_res, 0.0))

    # STEPS:________________________________________________________
    mdb.models['Model-1'].StaticStep(initialInc=0.1, maxInc=0.1, maxNumInc=1000, 
    minInc=0.001, name='Step-1', nlgeom=ON, previous='Initial')
    mdb.models['Model-1'].StaticStep(initialInc=0.025, maxInc=0.025, maxNumInc=1000, 
    minInc=0.01, name='Step-2', previous='Step-1')
    mdb.models['Model-1'].steps['Step-2'].setValues(adaptiveDampingRatio=0.05, 
    continueDampingFactors=False, matrixSolver=DIRECT, matrixStorage=
    UNSYMMETRIC, minInc=1e-14, stabilizationMagnitude=0.0002, 
    stabilizationMethod=DISSIPATED_ENERGY_FRACTION)

    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'CSTRESS', 'CDISP', 
    'SDEG', 'DMICRT', 'STATUS'))

    # INTERACTION:__________________________________________________
    mdb.models['Model-1'].ContactProperty('IntProp-1')
    mdb.models['Model-1'].interactionProperties['IntProp-1'].TangentialBehavior(
    formulation=FRICTIONLESS)
    mdb.models['Model-1'].interactionProperties['IntProp-1'].NormalBehavior(
    allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
    pressureOverclosure=HARD)

    mdb.models['Model-1'].ContactStd(createStepName='Initial', name='Int-1')
    mdb.models['Model-1'].interactions['Int-1'].includedPairs.setValuesInStep(
    stepName='Initial', useAllstar=ON)
    mdb.models['Model-1'].interactions['Int-1'].contactPropertyAssignments.appendInStep(
    assignments=((GLOBAL, SELF, 'IntProp-1'), ), stepName='Initial')

    RP1 = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(0.0, L_res+L_nut, 0.0))
    RP2 = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(0.0, L_thread+L_shank+L_res+L_nut, 0.0))

    nut_surf = mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.getByBoundingBox(
                xMin=-d_nut, yMin=L_res+L_nut, zMin=-d_nut, xMax=d_nut, yMax=L_res+L_nut, zMax=d_nut)
    mdb.models['Model-1'].rootAssembly.Set(faces=nut_surf, name='NUT_SURF')

    head_surf = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getByBoundingBox(
                xMin=-d_head, yMin=L_thread+L_shank+L_res+L_nut, zMin=-d_head, xMax=d_head, yMax=L_thread+L_shank+L_res+L_nut, zMax=d_head)
    mdb.models['Model-1'].rootAssembly.Set(faces=head_surf, name='HEAD_SURF')

    mdb.models['Model-1'].Coupling(
    controlPoint=Region(referencePoints=(
        mdb.models['Model-1'].rootAssembly.referencePoints[RP1.id],)),
    couplingType=KINEMATIC,
    influenceRadius=WHOLE_SURFACE,
    localCsys=None,
    name='Constraint-1',
    surface=Region(
        side1Faces=mdb.models['Model-1'].rootAssembly.sets['NUT_SURF'].faces),
    u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
    weightingMethod=CUBIC)

    mdb.models['Model-1'].Coupling(
    controlPoint=Region(referencePoints=(
        mdb.models['Model-1'].rootAssembly.referencePoints[RP2.id],)),
    couplingType=KINEMATIC,
    influenceRadius=WHOLE_SURFACE,
    localCsys=None,
    name='Constraint-2',
    surface=Region(
        side1Faces=mdb.models['Model-1'].rootAssembly.sets['HEAD_SURF'].faces),
    u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
    weightingMethod=CUBIC)

    # BOUNDARY CONDITION AND LAODING:______________________________________________
    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Initial', 
    distributionType=UNIFORM, fieldName='', localCsys=None, name='BC-1', 
    region=Region(referencePoints=(
    mdb.models['Model-1'].rootAssembly.referencePoints[RP1.id], )), u1=SET, u2=SET, 
    u3=SET, ur1=SET, ur2=SET, ur3=SET)

    mdb.models['Model-1'].Moment(cm1=moment_value[0], cm3=moment_value[1], createStepName='Step-1', 
    distributionType=UNIFORM, field='', localCsys=None, name='Load-1', region=
    Region(referencePoints=(
    mdb.models['Model-1'].rootAssembly.referencePoints[RP2.id], )))

    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Step-2', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'BC-2', region=Region(referencePoints=(
    mdb.models['Model-1'].rootAssembly.referencePoints[RP2.id], )), u1=UNSET, u2=
    target_disp, u3=UNSET, ur1=target_rot, ur2=UNSET, ur3=UNSET)

    y_list = [d_thread * n_strip / 4.0 / math.pi * (math.cos(2*i*math.pi/n_strip) - math.cos(2*(i+1)*math.pi/n_strip)) for i in range(n_strip)]
    x_list = [d_thread * n_strip / 4.0 / math.pi * (- math.sin(2*i*math.pi/n_strip) + math.sin(2*(i+1)*math.pi/n_strip)) for i in range(n_strip)]

    K_spring = mat_data["strip"]["K"]
    FDP = mat_data["strip"]["FDP"]
    FracEng = mat_data["strip"]["FracEng"]

    D_angle = 2*math.pi / float(n_strip)
    angle = D_angle/2
    constr_num = 2
    for i in range(n_strip):
        if n_strip == 1:
            RPi = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(0.0, L_res+0.5*L_nut+0.5, 0.0))
            RPj = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(0.0, L_res+0.5*L_nut-0.5, 0.0))

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(3*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(3*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(5*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(5*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(7*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(7*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(3*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(3*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(5*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(5*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=DISTRIBUTING,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(7*math.pi/4)/2.0, L_res+0.5*L_nut, d_thread*math.sin(7*math.pi/4)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

        elif n_strip == 2:
            RPi = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(x_list[i], L_res+0.5*L_nut+0.5, y_list[i]))
            RPj = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(x_list[i], L_res+0.5*L_nut-0.5, y_list[i]))

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(angle-math.pi/4.0)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle-math.pi/4.0)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(angle+math.pi/4.0)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle+math.pi/4.0)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(angle-math.pi/4.0)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle-math.pi/4.0)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(angle+math.pi/4.0)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle+math.pi/4.0)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            
        else:
            RPi = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(x_list[i], L_res+0.5*L_nut+0.5, y_list[i]))
            RPj = mdb.models['Model-1'].rootAssembly.ReferencePoint(point=(x_list[i], L_res+0.5*L_nut-0.5, y_list[i]))
            
            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.findAt(
                    ((d_thread*math.cos(angle)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

            constr_num += 1
            mdb.models['Model-1'].Coupling(
            controlPoint=Region(referencePoints=(mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id], )),
            couplingType=STRUCTURAL,
            influenceRadius=WHOLE_SURFACE,
            localCsys=None,
            name='Constraint-' + str(constr_num),
            surface=Region(
                side1Faces=mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].faces.findAt(
                    ((d_thread*math.cos(angle)/2.0, L_res+0.5*L_nut, d_thread*math.sin(angle)/2.0),)
                )
            ),
            u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON,
            weightingMethod=UNIFORM)

        mdb.models['Model-1'].ConnectorSection(name='ConnSect-{}'.format(i+1), translationalType=
        AXIAL, u1ReferenceLength=1.0)
        mdb.models['Model-1'].sections['ConnSect-{}'.format(i+1)].setValues(behaviorOptions=(
            ConnectorElasticity(table=((K_spring, ), ), independentComponents=(), 
            components=(1, )), ConnectorPlasticity(isotropicTable=FDP, kinematicTable=(), components=(1, )), 
            ConnectorDamage(criterion=PLASTIC_MOTION, evolutionType=ENERGY_TYPE, useAffected=ON, 
            initiationTable=((FDP[-1][1], 0.0), ), evolutionTable=((FracEng, ), ), 
            affectedComponents=(1, ), components=(1, ))))
        mdb.models['Model-1'].sections['ConnSect-{}'.format(i+1)].behaviorOptions[0].ConnectorOptions(
            )
        mdb.models['Model-1'].sections['ConnSect-{}'.format(i+1)].behaviorOptions[1].IsotropicOptions(
            )
        mdb.models['Model-1'].rootAssembly.DatumCsysByThreePoints(coordSysType=
            CARTESIAN, origin=mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], 
            point1=mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id])
        mdb.models['Model-1'].rootAssembly.WirePolyLine(mergeType=IMPRINT, meshable=
            False, points=((mdb.models['Model-1'].rootAssembly.referencePoints[RPi.id], 
            mdb.models['Model-1'].rootAssembly.referencePoints[RPj.id]), ))
        mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Wire-{}'.format(i+1), 
            toName='Wire-{}'.format(i+1))
        mdb.models['Model-1'].rootAssembly.Set(edges=
            mdb.models['Model-1'].rootAssembly.edges.getSequenceFromMask(('[#1 ]', ), )
            , name='Wire-{}-Set-{}'.format(i+1, i+1))
        mdb.models['Model-1'].rootAssembly.SectionAssignment(region=
            mdb.models['Model-1'].rootAssembly.sets['Wire-{}-Set-{}'.format(i+1, i+1)], sectionName=
            'ConnSect-{}'.format(i+1))
        mdb.models['Model-1'].rootAssembly.sectionAssignments[0].getSet()
        mdb.models['Model-1'].rootAssembly.ConnectorOrientation(localCsys1=
            mdb.models['Model-1'].rootAssembly.datums[1], region=
            mdb.models['Model-1'].rootAssembly.allSets['Wire-{}-Set-{}'.format(i+1, i+1)])
        angle += D_angle
    
    # MESHING:____________________________________________________________
    mdb.models['Model-1'].rootAssembly.makeIndependent(instances=(
    mdb.models['Model-1'].rootAssembly.instances['Part-2-1'], ))

    mdb.models['Model-1'].rootAssembly.seedPartInstance(deviationFactor=0.1, 
    minSizeFactor=0.1, regions=(
    mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], 
    mdb.models['Model-1'].rootAssembly.instances['Part-2-1']), size=mesh_size)

    mdb.models['Model-1'].rootAssembly.setMeshControls(algorithm=ADVANCING_FRONT, 
    elemShape=HEX_DOMINATED, regions=
    mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(
    mask=('[#ffffff ]', ), )+\
    mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].cells.getSequenceFromMask(
    mask=('[#3f ]', ), ), technique=SWEEP)
    mdb.models['Model-1'].rootAssembly.generateMesh(regions=(
    mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], 
    mdb.models['Model-1'].rootAssembly.instances['Part-2-1']))

    mdb.models['Model-1'].rootAssembly.setElementType(elemTypes=(ElemType(
    elemCode=C3D8R, elemLibrary=STANDARD, secondOrderAccuracy=ON, 
    kinematicSplit=AVERAGE_STRAIN, hourglassControl=ENHANCED, 
    distortionControl=ON, lengthRatio=0.100000001490116, elemDeletion=ON, 
    maxDegradation=0.98), ElemType(elemCode=C3D6, elemLibrary=STANDARD), 
    ElemType(elemCode=C3D4, elemLibrary=STANDARD)), regions=(
    mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].cells.getSequenceFromMask(
    mask=('[#ffff ]', ), )+\
    mdb.models['Model-1'].rootAssembly.instances['Part-2-1'].cells.getSequenceFromMask(
    mask=('[#f ]', ), ), ))

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

    top_node = odb.rootAssembly.nodeSets['ASSEMBLY_CONSTRAINT-2_REFERENCE_POINT']

    frames = odb.steps["Step-2"].frames

    Uy = []
    Ny = []
    Mx = []
    Mz = []
    for f in frames:
        N_top = f.fieldOutputs["RF"].getSubset(region=top_node).values[0].data
        U_top = f.fieldOutputs["U"].getSubset(region=top_node).values[0].data
        M_top = f.fieldOutputs["RM"].getSubset(region=top_node).values[0].data

        Ny.append(float(N_top[1]))
        Uy.append(float(U_top[1]))
        Mx.append(float(M_top[0]))
        Mz.append(float(M_top[2]))
    
    N_U = {"N": Ny, "U": Uy, "Mx": Mx, "Mz": Mz}

    # Save mat_prop to a JSON file
    with open("N_U.json", "w") as f:
        json.dump(N_U, f)
    
    # closing the odb file: ___________________________________________________________________________________
    odb.close()

    # Deleting the odb and cae files: _________________________________________________________________________
    # os.remove("Model.cae")

    try:
        os.remove("Job-Main-"+ str(job_number) +".odb")
    except:
        pass

    # Check if lock file exists and delete it
    lock_file = "Model.lck"

    if os.path.exists(lock_file):
        
        os.remove(lock_file)
    
    # Close Abaqus/CAE
    sys.exit()
    
# load geom_prop:
with open("geom_prop.json", "r") as f:
    geom_prop = json.load(f)  # Load force-displacement data

# load mat_data:
with open("mat_data.json", "r") as f:
    mat_data = json.load(f)  # Load force-displacement data

# load disp_moment:
with open("disp_moment.json", "r") as f:
    disp_moment = json.load(f)  # Load force-displacement data

# load disp_moment:
with open("mesh_strip_job.json", "r") as f:
    mesh_strip_job = json.load(f)  # Load force-displacement data

mdlbld(geom_prop=geom_prop, mat_data=mat_data, target_disp=disp_moment[0], target_rot=disp_moment[1],
       moment_value=disp_moment[2], mesh_size=mesh_strip_job[0],
       n_strip=mesh_strip_job[1], job_number=mesh_strip_job[2])