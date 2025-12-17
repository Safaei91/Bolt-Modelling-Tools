import EMat as em
import DucFrac as dfr
import numpy as np
import pandas as pd
import Pre_Neck_Opt
from paramEval import * 
import subprocess
import json
from math import pi
from scipy.interpolate import interp1d
from scipy.optimize import minimize
import os
import matplotlib.pyplot as plt
#_______________________________________________________________________________________
# Coupon Specimen Geometric Properties:
geom_prop = {"L": 80, "r": 6.35, "D": 7.94, "B": 16.0, "T": 12.827, "W": 25.58, "G": 40,
             "mesh_size": 1.0, "sec_type": "circ", "spec_type": "dog_bone"}


#_______________________________________________________________________________________
# This is the objective function to be minimized:
def obj_func(Fr_mod_fac, params, SS_list, geom_prop):
    Fr_mod_fac = Fr_mod_fac[0]
    print("The selected factor is: " + str(Fr_mod_fac))
    iter = int(Fr_mod_fac * 100)
    Delta_Fr_mod = params["Fu"] * (params["eps_r"] - params["eps_u"])
    Fr_mod = params["Fu"] + Fr_mod_fac * Delta_Fr_mod
    stress_sim, e_sim = em.eMat(
            E=params["E"],
            Fy=params["Fy"],
            Fu=params["Fu"],
            Fr=Fr_mod,
            eps_yp=params["eps_yp"],
            eps_u=params["eps_u"],
            eps_r=params["eps_r"],
            b_list=params["b_list"],
            n_list=params["n_list"],
            strain_hist=np.linspace(0, 5*SS_list[3][-1], int(1e5)),
            type="true")

    plastic_strain = e_sim - stress_sim / params["E"]
    for i in range(len(e_sim)):
        if e_sim[i] >= params["Fy"][1]/params["E"]:
            plastic_strain = plastic_strain[i:] - plastic_strain[i]
            plastic_stress = stress_sim[i:]
            break
    SSP = tuple([(i, j) for i, j in zip(plastic_stress, plastic_strain)])
    mat_prop = {"E": params["E"], "SSP": SSP}

    # Save mat_prop to a JSON file
    with open("mat_prop.json", "w") as f:
        json.dump(mat_prop, f)
    
    # Save geom_prop to a JSON file
    with open("geom_prop.json", "w") as f:
        json.dump(geom_prop, f)

    # Save mat_prop to a JSON file
    with open("disp.json", "w") as f:
        target_disp = 0.5*geom_prop["G"] * SS_list[3][-1]
        json.dump(target_disp, f)

    # Save mat_prop to a JSON file
    with open("iter.json", "w") as f:
        json.dump(iter, f)

    # Define the Abaqus command and script
    abaqus_command = "abaqus cae"
    script_name = "abq_mb.py"

    try:
        os.remove("F_D.json")
    except:
        pass

    try:
        # Run the Abaqus command
        result = subprocess.run(
            f"{abaqus_command} script={script_name}", 
            shell=True, 
            check=True, 
            capture_output=True, 
            text=True
        )
        # Print the standard output from the command
        print("Abaqus Output:")
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        # Handle errors
        print("Error running Abaqus:")
        print(e.stderr)
    
    # Load F-D results from the Abaqus script (assumes F_D is saved as JSON)
    with open("F_D.json", "r") as f:
        F_D = json.load(f)  # Load force-displacement data
    
    # eng stress strain:
    if geom_prop["sec_type"] == "circ":
        A = 0.25 * pi * geom_prop["D"]**2 / 4
        stress_sim_eng = np.array(F_D["F"]) / A
        strain_sim_eng = np.array(F_D["D"]) / 0.5 / float(geom_prop["G"])
    
    elif geom_prop["sec_type"] == "rect":
        A = 0.25 * geom_prop["T"] * geom_prop["D"]
        stress_sim_eng = np.array(F_D["F"]) / A
        strain_sim_eng = np.array(F_D["D"]) / 0.5 / float(geom_prop["G"])

    for i, e in enumerate(strain_sim_eng):
        if e > stress_sim_eng[i] / params["E"] + 0.002:
            Fy_max = max(stress_sim_eng[:i])
            break

    interpol_function = interp1d(strain_sim_eng, stress_sim_eng, kind='linear')
    interpol_stresses = interpol_function(SS_list[3]) * max(params["Fy"])/Fy_max

    # Calculate area-based error
    strain_incr = np.insert(np.diff(SS_list[3]), 0, 0)
    test_area = np.sum(strain_incr * SS_list[4])
    sim_test_diff = np.sum(abs(strain_incr * (SS_list[4] - interpol_stresses)))
    error = sim_test_diff / test_area * 100

    df = pd.DataFrame({"strain": strain_sim_eng, "stress": stress_sim_eng})
    df.to_csv('output_file.csv', index=False)

    print("The current error is: " + str(error))
    return error

SS_list = Pre_Neck_Opt.pno("Data")
params = SS_list[0]
bounds = [(0.0, 1.0)]  # Fr_mod_fac should be between 0.0 and 1.0
initial_guess = [0.5]  # Initial guess for Fr_mod_fac

# Perform the optimization
result = minimize(
    obj_func,
    x0=initial_guess,  # Starting point for Fr_mod_fac
    args=(params, SS_list, geom_prop),  # Additional parameters for the objective function
    bounds=bounds,  # Bounds for Fr_mod_fac
    method='L-BFGS-B',  # Optimization method
    options={
        'ftol': 1e-2,       # Function tolerance (adjust as needed)
        'maxiter': 1,      # Maximum number of iterations (adjust as needed)
        'eps': 1e-2
        }
)

# final run with optimized value:
obj_func(result.x, params, SS_list, geom_prop)

with open("S_E.json", "r") as f:
    S_E = json.load(f)

# Extracting Ductile Fracture parameters:
Fr = S_E["S_mises"][-1]
eps_fp = S_E["PEEQ"][-1]
eps_r = Fr/params["E"] + eps_fp

s11 = np.array(S_E["S11"])
s22 = np.array(S_E["S22"])
s33 = np.array(S_E["S33"])
smz = np.array(S_E["S_mises"])

PEEQ = S_E["PEEQ"]
plas_st_pn_dif = np.diff(PEEQ)

p = 1/3 * (s11 + s22 + s33)
T = p / smz
T[0] = T[1]
T_mid = (T[:-1] + T[1:]) / 2

epsilon_ref = sum(np.exp(3 / 2 * T_mid) * plas_st_pn_dif)

# Printing the optimization results:
print(f"  Rupture Strain (ε_r): {eps_r:.5f}")
print(f"  Ultimate Stress (Fr): {Fr:.2f} MPa")
print(f"  Reference Strain (ε_ref): {epsilon_ref:.5f}")

# Exporting to an excel file:
stress_sim, e_sim = em.eMat(
        E=params["E"],
        Fy=params["Fy"],
        Fu=params["Fu"],
        Fr=Fr,
        eps_yp=params["eps_yp"],
        eps_u=params["eps_u"],
        eps_r=eps_r,
        b_list=params["b_list"],
        n_list=params["n_list"],
        strain_hist=np.linspace(0, eps_r, int(1e4)),
        type="true")

df = pd.read_csv('output_file.csv')

# Extract stress and strain from the DataFrame
strain_sim_eng = df['strain']
stress_sim_eng = df['stress']

# Plot settings for a professional look
plt.figure(figsize=(6, 4))  # Set figure size for better resolution
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'cm'
# Plotting experimental data and simulation result
plt.plot(SS_list[3], SS_list[4], color="black", label="Exp_eng")
plt.plot(strain_sim_eng, stress_sim_eng, color="gray", linestyle="-.", label="Sim_eng")
plt.plot(e_sim, stress_sim, color="red", linestyle="--", label="Sim_true")
# Labeling axes with units and clear descriptions
plt.xlabel(r"Strain, $\epsilon$ (decimal)", fontsize=18)
plt.ylabel(r"Stress, $\sigma$ (MPa)", fontsize=18)
# Formatting axis ticks and adding grid
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.tick_params(axis='both', direction='in', length=6, width=1.2)
plt.grid(visible=True, linestyle="--", linewidth=0.5)
# Adding horizontal and vertical axis lines at origin
plt.axhline(0, color='black', linewidth=1.2)
plt.axvline(0, color='black', linewidth=1.2)
# Set plot boundaries to avoid clipped edges
plt.gca().spines['top'].set_linewidth(1.2)
plt.gca().spines['right'].set_linewidth(1.2)
plt.gca().spines['bottom'].set_linewidth(1.2)
plt.gca().spines['left'].set_linewidth(1.2)
# Adding legend with specific positioning
plt.legend(fontsize=14, loc="lower right")
# Adjust layout and save the figure as a PDF
plt.tight_layout()
plt.savefig("fig.pdf", format="pdf")
# Display the plot

plt.show()  
