import numpy as np
import pandas as pd
import EMat as em
import json
import os
import subprocess
import matplotlib.pyplot as plt

def bolt_mbld(geom_prop, base_mat_prop, strip_mat_prop, mesh_size, target_disp=0.01, target_rot=0.01, moments=[1, 1], strip_num=6, job_num=1, plot= False):

    eps_f = 0.614 * np.exp(-0.586 * mesh_size) + 0.381 
    eps_t0 = 2.069/3 * np.exp(-0.837 * mesh_size) + 0.815
    Fr = 1207.18

    stress_true, strain_true = em.eMat(E=base_mat_prop["E"],
                                    Fy=base_mat_prop["Fy"],
                                    Fu=base_mat_prop["Fu"],
                                    Fr=Fr,
                                    eps_yp=base_mat_prop["eps_yp"],
                                    eps_u=base_mat_prop["eps_u"],
                                    eps_r=eps_f,
                                    b_list=base_mat_prop["b_list"],
                                    n_list=base_mat_prop["n_list"],
                                    strain_hist=None,
                                    type="true")

    strip_force, strip_disp = em.eMat(E=strip_mat_prop["E"],
                                    Fy=strip_mat_prop["Fy"],
                                    Fu=strip_mat_prop["Fu"],
                                    Fr=strip_mat_prop["Fr"],
                                    eps_yp=strip_mat_prop["eps_yp"],
                                    eps_u=strip_mat_prop["eps_u"],
                                    eps_r=strip_mat_prop["eps_r"],
                                    b_list=strip_mat_prop["b_list"],
                                    n_list=[1, 1, 1],
                                    strain_hist=None,
                                    type="eng")

    strain_true_mod = strain_true - stress_true / base_mat_prop["E"]
    true_plastic_strain = []
    yield_stress = []
    for i, s in enumerate(strain_true_mod):
        if s >= 1e-4:
            true_plastic_strain.append(s)
            yield_stress.append(stress_true[i])

    true_plastic_strain[0] = 0.0
    SSP = tuple([(i, j) for (i, j) in zip(yield_stress, true_plastic_strain)])
    
    TriAx = np.linspace(-1/3, 2, 100)
    eps_trx = eps_t0 * np.exp(- 3 / 2 * TriAx)
    duc_dam_mat = tuple([(i, j, 0.0) for i, j in zip(eps_trx, TriAx)])

    frac_eng_mat = 0.5 * max(base_mat_prop["Fy"])**2 / base_mat_prop["E"]

    strip_disp_mod = strip_disp - strip_force / strip_mat_prop["E"]
    plastic_disp = []
    yield_strength = []
    for i, s in enumerate(strip_disp_mod):
        if s >= 1e-4:
            plastic_disp.append(s)
            yield_strength.append(strip_force[i])

    plastic_disp[0] = 0.0
    FDP = tuple([(i / strip_num, j, 0) for (i, j) in zip(yield_strength, plastic_disp)])

    eng_frac_strip = 0.5 * max(strip_mat_prop["Fy"])**2 / strip_mat_prop["E"] / strip_num

    mat_data = {"base": {"E": base_mat_prop["E"], "SSP": SSP, "DucDam": duc_dam_mat, "FracEng": frac_eng_mat},
                "strip": {"K": strip_mat_prop["E"]/strip_num, "FDP": FDP, "FracEng": eng_frac_strip}}

    # Save geom_prop to a JSON file
    with open("geom_prop.json", "w") as f:
        json.dump(geom_prop, f)

    # Save mat_data to a JSON file
    with open("mat_data.json", "w") as f:
        json.dump(mat_data, f)
    
    # Save disp_moment data to a JSON file
    with open("disp_moment.json", "w") as f:
        json.dump([target_disp, target_rot, moments], f)

    # Save mesh_strip_job data to a JSON file
    with open("mesh_strip_job.json", "w") as f:
        json.dump([mesh_size, strip_num, job_num], f)

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
    
    # load geom_prop:
    with open("N_U.json", "r") as f:
        N_U = json.load(f)  # Load force-displacement data
    
    N = N_U["N"]
    U = N_U["U"]
    Mx = N_U["Mx"]

    df = pd.DataFrame({"N": N, "U": U, "Mx": Mx})
    df.to_csv(f"output_{job_num}.csv")

    if plot == True:
        test = pd.read_csv("test.csv")
        sim = pd.read_csv(f"output_{job_num}.csv")

        # Plotting experimental vs. simulated data
        plt.figure(figsize=(6, 4))  # Define plot size
        plt.rcParams['font.family'] = 'Times New Roman'  # Set font style
        plt.rcParams['mathtext.fontset'] = 'cm'  # Set math font

        # Plot experimental data
        plt.plot(test["Disp_tot"], test["Force"], color="black", label="Exp")
        # Plot simulated data
        plt.plot(sim["U"], sim["N"], color="red", linestyle="dashed", label="Sim")

        # Adding labels and customizing the plot
        plt.xlabel(r"Displacement, $\delta$ (mm)", fontsize=18)
        plt.ylabel(r"Force, $F$ (kN)", fontsize=18)
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)
        plt.tick_params(axis='both', direction='in', length=6, width=1.2)
        plt.grid(visible=True, linestyle="--", linewidth=0.5)

        # Add axis lines
        plt.axhline(0, color='black', linewidth=1.2)
        plt.axvline(0, color='black', linewidth=1.2)

        # Customize plot borders
        plt.gca().spines['top'].set_linewidth(1.2)
        plt.gca().spines['right'].set_linewidth(1.2)
        plt.gca().spines['bottom'].set_linewidth(1.2)
        plt.gca().spines['left'].set_linewidth(1.2)

        # Add legend and finalize the plot
        plt.legend(fontsize=14, loc="lower right")
        plt.tight_layout()
        plt.savefig(f"fig_{job_num}.pdf")  # Save the figure as a PDF
        plt.show()  # Display the plot

