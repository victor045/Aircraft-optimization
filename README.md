
# Optimización de Dinámica Longitudinal de Aeronaves con NSGA-II ✈️

Este proyecto implementa un sistema de optimización multiobjetivo usando el algoritmo **NSGA-II** para encontrar configuraciones óptimas de un modelo dinámico longitudinal de una aeronave. El enfoque permite analizar la **estabilidad** y **maniobrabilidad** a través del análisis de polos y la respuesta al escalón.

---

## 📌 Objetivos

- Minimizar la **frecuencia natural promedio** del sistema.
- Maximizar la **razón de amortiguamiento promedio**.
- Clasificar cada modo como **Fugoide**, **Periodo corto** o **Indeterminado**.
- Visualizar gráficamente la **respuesta al escalón** para soluciones óptimas.

---

## 🧠 Estructura del Código

### 1. **Importación de librerías**
Usamos:
- `numpy`, `matplotlib`, `scipy.signal` para análisis y visualización del sistema dinámico.
- `pymoo` para la optimización evolutiva multiobjetivo.
- `os` para guardar archivos generados.

### 2. **Clase `AircraftTransferFunction`**
Define un modelo de segundo orden:
```
G(s) = (k·s + k·c) / (s² + 2ζωₙ·s + ωₙ²)
```
Incluye:
- Análisis de polos y ceros.
- Clasificación del tipo de modo según frecuencia natural:
    - `Fugoide` si ωn < 0.5 rad/s.
    - `Periodo corto` si ωn > 1.5 rad/s.
    - `Indeterminado` en el rango intermedio.
- Simulación de la respuesta al escalón y exportación como imagen PNG.

### 3. **Clase `AircraftOptimizationProblem`**
Implementa el problema de optimización con:
- 4 variables: `k`, `c`, `zeta`, `omega_n`.
- Función objetivo:
    - `f1 = promedio(ωn)`
    - `f2 = -promedio(ζ)`
- Búsqueda sobre los rangos definidos:
    ```
    k ∈ [0.5, 5.0]
    c ∈ [0.1, 2.0]
    zeta ∈ [0.01, 1.0]
    omega_n ∈ [0.1, 5.0]
    ```

### 4. **Ejecución de NSGA-II**
Se usa:
```python
NSGA2(pop_size=40)
n_gen = 25
```
El resultado es un conjunto de soluciones óptimas (frente de Pareto).

### 5. **Evaluación de las mejores soluciones**
Para las 5 mejores soluciones:
- Se imprimen los polos con su clasificación.
- Se grafican las respuestas al escalón y se almacenan en `/respuestas_escalon`.

---

## 📊 Resultados

- Se genera automáticamente un gráfico `frente_pareto.png`.
- Respuestas al escalón guardadas para cada solución óptima.
- Clasificación de polos para cada configuración encontrada.

---

## 🔧 Posibles Mejoras

- Penalizar soluciones que no tengan ambos modos dinámicos bien definidos.
- Exportar configuraciones y métricas a `.csv`.
- Incluir respuesta a impulso o rampa.
- Agregar animaciones para visualizar la evolución de la población.

---

## 📁 Estructura de Archivos Generados

```
├── optimizador_avion_nsga2_extendido.py
├── frente_pareto.png
├── respuestas_escalon/
│   ├── step_response_Sol_1_...
│   ├── ...
└── README.md
```

---

## 🚀 Requisitos

```bash
pip install numpy matplotlib scipy pymoo

python optimizador_avion_nsga2_extendido.py 
```

---

## 📬 Autor
Este proyecto fue desarrollado como extensión y análisis de tesis de dinámica longitudinal de aeronaves. Ideal para estudios de control, estabilidad y diseño aerodinámico.
