import numpy as np
from scipy.optimize import fsolve

def eMat(E, Fy, Fu, Fr, eps_yp, eps_u, eps_r, b_list=None, n_list=[1, 1, 1], strain_hist=None, type="eng"):
    """
    Calculate the stress-strain curve for a steel material model with regions:
    - Elastic, Yield Plateau, Strain Hardening, and Softening.

    Parameters:
        E (float): Young's modulus of elasticity (initial slope of stress-strain curve).
        Fy (tuple): Tuple containing minimum and maximum yield stresses, (Fy_min, Fy_max).
        Fu (float): Ultimate tensile strength.
        Fr (float): Stress at rupture point.
        eps_yp (float): Strain at the end of the yield plateau.
        eps_u (float): Strain at the ultimate tensile strength.
        eps_r (float): Strain at rupture point.
        rate_list (list): List of rates for exponential backstress in each region.
                          Format: [rate_yield_plateau, rate_hardening, rate_softening].
        strain_hist (list or array, optional): Optional list of strain values to use for
                                               stress-strain history.
        type (str): 'eng' for engineering stress-strain, 'true' for true stress-strain.
    Returns:
        stress (array): Calculated stress values for the given strain history.
        strain (array): Strain values (input strain history or generated automatically).
    """
    
    # Define the exponential function for backstress modeling
    def back_stress(eps, delta_sigma, delta_epsilon, b, n):

        return delta_sigma * (1 - np.exp(-b * eps**n)) / (1 - np.exp(-b * delta_epsilon**n))
    
    def calculate_b(slb, delta_sigma, delta_epsilon):

        def equation(b):
            slope = b * delta_sigma / (1 - np.exp(-b * delta_epsilon))
            return slope - slb  # Equation to solve
            
        try:
            b_guess = 10  # Initial guess for b
            b_solution = fsolve(equation, b_guess)
            if b_solution.size > 0:
                return b_solution[0]
            else:
                raise ValueError("No solution found for 'b'.")
        except Exception as e:
            print(f"Error calculating 'b': {e}")
            return None
    
    # Extract yield stress range and compute stress differences
    Fy_min = min(Fy)
    Fy_max = max(Fy)
    D_sigma_y = Fy_max - Fy_min        # Yield plateau stress drop
    D_sigma_u = Fu - Fy_min            # Stress increase to ultimate
    D_sigma_r = Fu - Fr                # Stress drop to rupture

    # Generate strain history if not provided
    if strain_hist is None:
        strain = np.linspace(0, eps_r, int(1e4))  # Divide strain range into 10,000 points
    else:
        strain = np.array(strain_hist)

    # Define key strain regions
    e_y = Fy_max / E                   # Elastic limit strain
    e_yp = eps_yp - e_y                # Yield plateau strain range
    e_u = eps_u - eps_yp               # Strain hardening strain range
    e_r = eps_r - eps_u                # Softening strain range

    # Initialize stress list
    stress = []
    s_temp = []
    e_temp = []
    for e in strain:
        if e < e_y:
            # Elastic region: linear stress-strain
            stress.append(E * e)
        elif e <= eps_yp:
            # Yield plateau region
            e_adj = e - e_y
            sigma = back_stress(e_adj, -D_sigma_y, e_yp, b_list[0], n_list[0])
            stress.append(Fy_max + sigma)
        elif e <= eps_u:
            # Strain hardening region
            e_adj = e - eps_yp
            sigma = back_stress(e_adj, D_sigma_u, e_u, b_list[1], n_list[1])
            stress.append(Fy_min + sigma)
            s_temp.append(sigma)
            e_temp.append(e)
        else:
            # Softening region
            e_adj = e - eps_u
            if type == "eng":
                sigma = back_stress(e_adj, D_sigma_r, e_r, b_list[2], n_list[2])
                stress.append(Fr + sigma)
            elif type == "true":
                if len(b_list) == 2:
                    slb = (s_temp[-1] - s_temp[-2]) / (e_temp[-1] - e_temp[-2])
                    b = calculate_b(slb, -D_sigma_r, e_r)
                    if b is not None:
                        b_list.append(b)
                    else:
                        raise ValueError("Failed to calculate 'b' for true stress-strain.")
                sigma = back_stress(e_adj, -D_sigma_r, e_r, b_list[2], n_list[2])
                stress.append(Fu + sigma)

    return np.array(stress), strain