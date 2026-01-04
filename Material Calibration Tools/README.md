#Ductile Fracture Calibration (Abaqus–Python)

This code calibrates ductile fracture parameters of metallic materials using tensile coupon test data. It couples a constitutive material model with Abaqus finite element simulations and optimizes the rupture stress by minimizing the difference between experimental and simulated stress–strain curves.

Key Inputs

Specimen geometry

L : total length (mm)

r : fillet radius (mm)

D : gauge diameter (circular) or width (rectangular) (mm)

T : thickness for rectangular sections (mm)

W : end-grip diameter or width (mm)

G : gauge length (mm)

mesh_size : mesh element size (mm)

sec_type : "circ" or "rect"

spec_type : "dog_bone" or "prism"

Experimental data

Experimental stress–strain data are read automatically from the Data/ directory.

What the Code Does

Builds a true stress–plastic strain material model

Runs an Abaqus coupon simulation

Extracts force–displacement results

Converts results to engineering stress–strain

Optimizes the rupture stress parameter

Computes rupture strain and reference strain

Plots experimental and simulated curves

Example

Define the specimen geometry and run:

python main.py

The script automatically launches Abaqus, performs the optimization, and saves the results and comparison plot (fig.pdf).
