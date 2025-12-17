# Importing required packages
import numpy as np
from scipy.stats import linregress
import EMat as em
from pymoo.optimize import minimize
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.soo.nonconvex.de import DE

# Function to modify stress-strain curve and estimate material properties
def crv_mod(stress_eng, strain_eng, Elas_range=[200, 300], type="true"):
    """
    Modifies the true stress-strain curve and estimates the modulus of elasticity, yield stress, and ultimate stress.

    Parameters:
    - stress_eng: Series or array of engineering stress values.
    - strain_eng: Series or array of engineering strain values.
    - Elas_range: Range of stress values to estimate the modulus of elasticity.

    Returns:
    - strain_mod, stress_mod: Modified strain and stress arrays.
    - E: Estimated modulus of elasticity.
    - Fy_min, Fy_max: Minimum and maximum yield stress.
    - Fu, Fr: Ultimate and rupture stress.
    - e_u, e_r: Strain at ultimate and rupture.
    """
    # Clean data by removing NaN values and converting to numpy arrays
    stress = stress_eng.dropna().to_numpy(dtype=float)
    strain = strain_eng.dropna().to_numpy(dtype=float)

    # Estimate modulus of elasticity using linear regression within specified range
    indices = [i for i, s in enumerate(stress) if Elas_range[0] < s < Elas_range[1]]
    slope, _, _, _, _ = linregress(strain[indices], stress[indices])
    E = slope  # Elastic modulus

    # Adjust strain to ensure smooth continuation
    middle_index = indices[len(indices) // 2]
    strain_mod = strain - (strain[middle_index] - stress[middle_index] / E)
    strain_mod = np.concatenate((np.linspace(0, strain_mod[middle_index], 100), strain_mod[middle_index:]))
    stress_mod = np.concatenate((np.linspace(0, stress[middle_index], 100), stress[middle_index:]))

    # Estimate yield stress (Fy) using 0.2% offset method
    for i, e in enumerate(strain_mod):
        if e > stress_mod[i] / E + 0.002:
            Fy_max = max(stress_mod[:i])
            Fy_max_index = np.argmax(stress_mod[:i])
            break

    # Determine ultimate stress (Fu) and minimum yield stress (Fy_min)
    max_index = np.argmax(stress_mod)
    Fu = stress_mod[max_index]
    Fy_min = min(stress_mod[Fy_max_index:max_index])
    e_u = strain_mod[max_index]
    Fr = stress_mod[-1]
    e_r = strain_mod[-1]

    if type == "true":
        strain_mod = np.log(1 + strain_mod)
        stress_mod = stress_mod * (1 + strain_mod)
        Fy_min = min(stress_mod[Fy_max_index:max_index])
        Fy_max = stress_mod[Fy_max_index]
        Fu = stress_mod[max_index]
        Fr = stress_mod[-1]
        e_u = strain_mod[max_index]
        e_r = strain_mod[-1]
        strain_mod = strain_mod[: max_index]
        stress_mod = stress_mod[: max_index]

    return strain_mod, stress_mod, E, Fy_min, Fy_max, Fu, Fr, e_u, e_r

# Objective function for optimization
def obj_funcs(params, data, type="true"):
    """
    Objective function to minimize error between simulated and experimental stress-strain curves.

    Parameters:
    - params: Material parameters array.
    - data: Experimental data for stress and strain.

    Returns:
    - float: Error value.
    """
    stress = data.iloc[1:, 1]
    strain = data.iloc[1:, 0] / 100
    strain_mod, stress_mod, E, Fy_min, Fy_max, Fu, Fr, e_u, e_r = crv_mod(stress, strain)

    # Simulate material behavior using parameters
    if type == "eng":
        stress_sim, e_sim = em.eMat(E=E, Fy=[Fy_min, Fy_max], Fu=Fu, Fr=Fr, eps_yp=params[0], eps_u=e_u, eps_r=e_r,
                                    b_list=[params[1], params[2], params[3]], strain_hist=strain_mod, type="eng")
    elif type == "true":
        stress_sim, e_sim = em.eMat(E=E, Fy=[Fy_min, Fy_max], Fu=Fu, Fr=Fr, eps_yp=params[0], eps_u=e_u, eps_r=e_r,
                                    b_list=[params[1], params[2]], n_list=[1.0, params[3], 1.0], strain_hist=strain_mod, type="true")
    
    # Calculate area-based error
    if type == "eng":
        strain_incr = np.insert(np.diff(strain_mod), 0, 0)
        test_area = abs(np.sum(strain_incr * stress_mod))
        sim_test_diff = np.sum(strain_incr * (stress_mod - stress_sim)**2)
        error = sim_test_diff / test_area * 100
    
    elif type == "true":
        slp_test = (stress_mod[-1] - stress_mod[-20]) / (strain_mod[-1] - strain_mod[-20])
        slp_sim = (stress_sim[-1] - stress_sim[-2]) / (strain_mod[-1] - strain_mod[-2])
        slp_diff = (abs(slp_test - slp_sim))
        strain_incr = np.insert(np.diff(strain_mod), 0, 0)
        test_area = abs(np.sum(strain_incr * stress_mod))
        sim_test_diff = np.sum(strain_incr * (stress_mod - stress_sim)**2)
        error = (sim_test_diff / test_area + slp_diff / slp_test) * 100

    print(f"Current error: {error:.2f}%")
    return error

# Custom problem class for Differential Evolution optimization
def optimize(E, Fy_min, Fy_max, Fu, Fr, e_u, e_r, data, type="true"):
    """
    Optimizes material parameters to minimize error between experimental and simulated stress-strain curves.

    Parameters:
    - E, Fy_min, Fy_max, Fu, Fr, e_u, e_r: Material properties.
    - data: Experimental data for stress-strain.

    Returns:
    - BSPs: Optimized material parameters.
    """
    class OptProblem(ElementwiseProblem):
        def __init__(self, *args, tol=5e-2, **kwargs):
            super().__init__(*args, **kwargs)
            self.tol = tol

        def _evaluate(self, values, out, *args, **kwargs):
            out["F"] = obj_funcs(params=values, data=data)
            if out["F"] <= self.tol:
                out["F"] = self.tol  # Early stopping if error within tolerance

    # Set parameter bounds and optimization problem
    if type == "eng":
        prob = OptProblem(n_var=4, n_obj=1, xl=np.array([Fy_max / E, 0, 0, 0]), 
                        xu=np.array([e_u, 300, 1000, 1000]), tol=5e-2)
    elif type == "true":
        prob = OptProblem(n_var=4, n_obj=1, xl=np.array([Fy_max / E, 0, 0, 0.8]), 
                        xu=np.array([Fy_max / E, 300, 1000, 1.0]), tol=5e-2)


    # Configure the Differential Evolution algorithm
    algo = DE(pop_size=40, variant="DE/rand/1/bin", CR=0.9, F=0.8)
    stop_criteria = ("n_gen", 200)  # Stop after 100 generations

    # Run optimization
    result = minimize(problem=prob, algorithm=algo, termination=stop_criteria)
    BSPs = result.X  # Best Solution Parameters

    # Display the optimized parameters in a readable format
    print("\nOptimized Material Parameters:")
    print(f"  Elastic Modulus (E): {E:.2f} MPa")
    print(f"  Yield Stress (Fy_min): {Fy_min:.2f} MPa")
    print(f"  Yield Stress (Fy_max): {Fy_max:.2f} MPa")
    print(f"  Ultimate Stress (Fu): {Fu:.2f} MPa")
    print(f"  Rupture Stress (Fr): {Fr:.2f} MPa")
    print(f"  Yield Plateau Strain (ε_yp): {result.X[0]:.5f}")
    print(f"  Ultimate Strain (ε_u): {e_u:.5f}")
    print(f"  Parameter b1: {result.X[1]:.3f}")
    print(f"  Parameter b2: {result.X[2]:.3f}")
    print(f"  Parameter n2: {result.X[3]:.3f}")
    if type == "eng":
        print(f"  Parameter b3: {result.X[3]:.3f}")

    return BSPs, [E, Fy_min, Fy_max, Fu, Fr, e_u, e_r]