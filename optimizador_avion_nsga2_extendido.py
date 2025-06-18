
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, tf2zpk, step
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.termination import get_termination
from pymoo.optimize import minimize
import os

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
            if natural_freq < 0.5:
                mode_type = "Fugoide"
            elif natural_freq > 1.5:
                mode_type = "Periodo corto"
            else:
                mode_type = "Indeterminado"

            analysis.append({
                "pole": p,
                "damping_ratio": damping_ratio,
                "natural_freq": natural_freq,
                "mode_type": mode_type
            })
        return analysis

    def plot_step_response(self, label):
        t, y = step(self.system)
        plt.figure()
        plt.plot(t, y, label=label)
        plt.title(f"Respuesta al Escalón - {label}")
        plt.xlabel("Tiempo [s]")
        plt.ylabel("Salida")
        plt.grid(True)
        plt.legend()
        output_dir = "respuestas_escalon"
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(f"{output_dir}/step_response_{label}.png")
        plt.close()

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

# Ejecutar optimización
problem = AircraftOptimizationProblem()
algorithm = NSGA2(pop_size=40)
termination = get_termination("n_gen", 25)

res = minimize(problem,
               algorithm,
               termination,
               seed=1,
               verbose=True)

# Mostrar frente de Pareto
F = res.F
plt.scatter(F[:, 0], -F[:, 1], c="blue")
plt.xlabel("Frecuencia Natural Promedio (a minimizar)")
plt.ylabel("Razón de Amortiguamiento Promedio (a maximizar)")
plt.title("Frente de Pareto: Estabilidad vs Maniobrabilidad")
plt.grid(True)
plt.tight_layout()
plt.savefig("frente_pareto.png")
plt.close()

# Clasificación y respuestas al escalón para las mejores 5 soluciones
top_indices = np.argsort(F[:, 0])[:5]
for i, idx in enumerate(top_indices):
    k, c, zeta, omega_n = res.X[idx]
    label = f"Sol_{i+1}_k{k:.2f}_c{c:.2f}_z{zeta:.2f}_wn{omega_n:.2f}"
    model = AircraftTransferFunction(k, c, zeta, omega_n)
    analysis = model.analyze_poles_zeros()
    print(f"\n== {label} ==")
    for p in analysis:
        print(f"Polo: {p['pole']}, ζ={p['damping_ratio']:.3f}, ωn={p['natural_freq']:.3f}, Tipo: {p['mode_type']}")
    model.plot_step_response(label)
