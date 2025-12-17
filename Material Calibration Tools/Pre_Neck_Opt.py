import pandas as pd
import matplotlib.pyplot as plt
from paramEval import *
import EMat as em

def pno(data_file_name):
    # Specify data file name and read data from Excel
    data_frame = pd.read_excel(f"{data_file_name}.xlsx")

    # Extracting stress and strain data, converting strain to decimal form
    strain = data_frame.iloc[1:, 0] / 100
    stress = data_frame.iloc[1:, 1]

    # Curve modification to adjust stress-strain data, retrieve essential parameters
    strain_true, stress_true, E_true, Fy_min_true, Fy_max_true, Fu_true, Fr_true, e_u_true, e_r_true = crv_mod(stress, strain)
    strain_eng, stress_eng, E_eng, Fy_min_eng, Fy_max_eng, Fu_eng, Fr_eng, e_u_eng, e_r_eng = crv_mod(stress, strain, type="eng")

    # Optimizing parameters based on modified curve data
    prms, knowns = optimize(E_true, Fy_min_true, Fy_max_true, Fu_true, Fr_true, e_u_true, e_r_true, data=data_frame)

    Fr_lin_try = Fu_true + Fu_true * (e_r_true - e_u_true)

    # Simulating material behavior using EMat model
    stress_sim, e_sim = em.eMat(
        E=E_true,
        Fy=[Fy_min_true, Fy_max_true],
        Fu=Fu_true,
        Fr=Fr_lin_try,
        eps_yp=prms[0],
        eps_u=e_u_true,
        eps_r=e_u_true,
        b_list=[prms[1], prms[2]],
        n_list=[1.0, prms[3], 1.0],
        type="true"
    )

    params = {"E": E_true, "Fy": [Fy_min_true, Fy_max_true], "Fu": Fu_true, "Fr": Fr_lin_try,
              "eps_yp": prms[0], "eps_u": e_u_true, "eps_r": e_r_true, "b_list": [prms[1], prms[2]],
              "n_list": [1.0, prms[3], 1.0]}

    # Plot settings for a professional look
    plt.figure(figsize=(6, 4))  # Set figure size for better resolution
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'cm'
    # Plotting experimental data and simulation result
    plt.plot(strain_true, stress_true, color="black", label="Exp_true")
    plt.plot(strain_eng, stress_eng, color="gray", label="Exp_eng")
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
    # plt.savefig(f"{data_file_name}.pdf", format="pdf")
    # Display the plot
    plt.show()

    return params, strain_true, stress_true, strain_eng, stress_eng