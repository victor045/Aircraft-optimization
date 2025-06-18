
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, tf2zpk
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.termination import get_termination
from pymoo.optimize import minimize

class AircraftTransferFunction:
    def __init__(self, k, c, zeta, omega_n):
        self.k = k
        self.c = c
        self.zeta = zeta
        self.omega_n = omega_n

        self.num = [self.k, self.k * self.c]
        self.den = [1, 2 * self.zeta * self.omega_n, self.omega_n ** 2]
        self.system = TransferFunction(self.num, self.den)

    def analyze_poles_zeros(self):
        zeros, poles, _ = tf2zpk(self.num, self.den)
        analysis = []
        for p in poles:
            damping_ratio = -np.real(p) / np.abs(p)
            natural_freq = np.abs(p)
            analysis.append({
                "pole": p,
                "damping_ratio": damping_ratio,
                "natural_freq": natural_freq
            })
        return analysis

class AircraftOptimizationProblem(ElementwiseProblem):
    def __init__(self):
        super().__init__(n_var=4,
                         n_obj=2,
                         n_constr=0,
                         xl=np.array([0.5, 0.1, 0.01, 0.1]),
                         xu=np.array([5.0, 2.0, 1.0, 5.0]),
                         elementwise_evaluation=True)

    def _evaluate(self, x, out, *args, **kwargs):
        k, c, zeta, omega_n = x
        model = AircraftTransferFunction(k, c, zeta, omega_n)
        poles_data = model.analyze_poles_zeros()
        avg_freq = np.mean([p["natural_freq"] for p in poles_data])
        avg_damping = np.mean([p["damping_ratio"] for p in poles_data])
        out["F"] = [avg_freq, -avg_damping]

problem = AircraftOptimizationProblem()
algorithm = NSGA2(pop_size=40)
termination = get_termination("n_gen", 25)

res = minimize(problem,
               algorithm,
               termination,
               seed=1,
               verbose=True)

# Mostrar el frente de Pareto
F = res.F
plt.scatter(F[:, 0], -F[:, 1], c="blue")
plt.xlabel("Frecuencia Natural Promedio (a minimizar)")
plt.ylabel("Razón de Amortiguamiento Promedio (a maximizar)")
plt.title("Frente de Pareto: Estabilidad vs Maniobrabilidad")
plt.grid(True)
plt.tight_layout()
plt.show()
