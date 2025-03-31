# eMat: Stress-Strain Curve Generation for Steel Materials

This repository contains the `eMat` function, which computes both **engineering** and **true (equivalent)** stress-strain curves for steel materials. The function provides a robust approach to modeling different phases of material behavior, including:
- **Elastic region**
- **Yield plateau**
- **Strain hardening**
- **Softening phase**

## Function Signature
```python
def eMat(E, Fy, Fu, Fr, eps_yp, eps_u, eps_f, b_list=None, n_list=[1, 1, 1], strain_hist=None, type="eng"):
```

## Parameters
- **E (float):** Young's modulus of elasticity (initial slope of the stress-strain curve).
- **Fy (tuple):** Minimum and maximum yield stresses `(Fy_min, Fy_max)`.
- **Fu (float):** Ultimate tensile strength.
- **Fr (float):** Stress at the rupture point.
- **eps_yp (float):** Strain at the end of the yield plateau.
- **eps_u (float):** Strain at the ultimate tensile strength.
- **eps_f (float):** Strain at the final point.
- **b_list (list, optional):** List of rate parameters for exponential backstress in each region.
- **n_list (list, optional):** List of exponents for exponential backstress `[default: [1, 1, 1]]`.
- **strain_hist (array-like, optional):** Custom strain values for stress-strain history.
- **type (str, optional):** Defines the output type:
  - `'eng'`: Engineering stress-strain curve (default).
  - `'true'`: True (equivalent) stress-strain curve.

## Return Values
- **Tuple:** `(stress array, strain array)` containing computed stress and strain values.

## Special Considerations
- When using **true stress-strain** representation, it is sufficient to provide only two parameters for `b_list` and `n_list`. The post-yield branch slope is automatically computed by the function.
- For **force-deformation** curves, the stress values should be converted to corresponding forces, and strain values should be transformed into displacements.

## Example Usage
```python
import numpy as np
import matplotlib.pyplot as plt
from EMat import emat

# Define material properties
E = 191400  # MPa
Fy = (1075.5, 1075.5)  # Yield stress range (MPa)
Fu = 1158.9  # Ultimate tensile strength (MPa)
Fr = 1164.2  # Rupture stress (MPa)
eps_yp = 0.00562
eps_u = 0.04204
eps_f = 0.074
b_list = [1.0, 30.17]
n_list = [1, 1]

# Generate stress-strain curve
stress, strain = eMat(E, Fy, Fu, Fr, eps_yp, eps_u, eps_f, b_list, n_list, type="true")

plt.plot(strain, stress)
plt.show()
