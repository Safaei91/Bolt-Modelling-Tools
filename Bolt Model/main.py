import PM_param_eval as pm

# Geometrical properties (units: mm)
geom_prop = {
    "d_shank": 23.5,     # Diameter of the bolt shank
    "d_thread": 21.2,   # Diameter of the bolt thread
    "d_head":   40.6,    # Diameter of the bolt head
    "d_nut": 40.7,       # Outer diameter of the nut
    "L_shank": 73.0,     # Length of the shank
    "L_thread": 7.1,     # Length of the thread
    "L_head": 16.2,      # Length of the head
    "L_nut": 19.9,       # Length of the nut
    "L_res": 19.6        # Length of the residual thread part
    }

base_mat_prop = {
    "E": 191434.34,                    # Young's modulus
    "Fy": [1075.47, 1075.47],          # Yield stress (upper and lower bounds)
    "Fu": 1158.91,                     # Ultimate stress
    "eps_yp": 0.00562,                 # Yield plateau strain
    "eps_u": 0.04204,                  # Ultimate strain
    "b_list": [1.0,  30.176],          # Strain hardening parameters (b)
    "n_list": [1.0, 1.0, 1.0]          # Strain hardening parameters (n)
    }

strip_mat_prop = {
    "E": 168136.80,                    # Young's modulus
    "Fy": [329840.00, 329840.00],      # Yield stress (upper and lower bounds)
    "Fu": 385500.00,                   # Ultimate stress
    "Fr": 329100.00,                   # Rupture stress
    "eps_yp": 329840.00/168136.80,     # Yield plateau strain
    "eps_u": 3.24740,                  # Ultimate strain
    "eps_r": 3.69310,                  # Rupture strain
    "b_list": [1.0, 3.250, 12.760]     # Strain hardening parameters
    }

n = 5
delta = 5.0
for i in range(n + 2):
    if i == 0:
        theta = 0.0
    
    elif i == n + 1:
        theta = delta / (1.2 + 1.3 / (n - 1)) / (geom_prop["d_shank"] + 2)
        delta = 0.0
    
    else:
        theta = delta / (1.2 + 1.3 / (n - 1) * i) / (geom_prop["d_shank"] + 2)

    pm.bolt_mbld(geom_prop=geom_prop, base_mat_prop=base_mat_prop, strip_mat_prop=strip_mat_prop, strip_num=6,
                 mesh_size=1.25, target_disp=delta, target_rot=theta, job_num=i+1, plot=True)