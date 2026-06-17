# Lógica de Navegación y Localización

## 1. Localización (¿Dónde estoy?)
El sistema utiliza una **Cámara Cenital** fija que supervisa el tablero completo.
*   **Referencia Global**: El marcador **ArUco ID 0** define el origen de coordenadas $(0,0)$.
*   **Posición del Robot**: La cámara detecta el marcador **ArUco ID 1** sobre el robot para determinar su ubicación absoluta en el campo.

    $$ \vec{P}_{robot} = \vec{P}_{ArUco1} - \vec{P}_{ArUco0} $$

    Es decir, el robot sabe que su posición $(x, y)$ es la distancia que lo separa del Origen.

## 2. Cálculo de Trayectoria (¿A dónde voy?)
Para navegar, el robot necesita conocer la ubicación del objetivo relativa a su propia posición. Esto se calcula mediante **Resta Vectorial**:

$$ \vec{V}_{relativo} = \vec{P}_{objetivo} - \vec{P}_{robot} $$

El resultado es un vector $(x, y)$ que representa la distancia longitudinal y lateral hacia el objetivo desde la perspectiva del robot.

## 3. Algoritmo de Control
A partir del vector relativo $(x, y)$, el robot determina sus movimientos:

1.  **Distancia ($d$):** Magnitud del vector ($\sqrt{x^2 + y^2}$).
2.  **Orientación ($\theta$):** Ángulo hacia el objetivo ($\arctan(y/x)$).

El robot ajusta sus velocidades lineal y angular para minimizar $\theta$ (girar hacia el objetivo) y reducir $d$ (avanzar), deteniéndose cuando la distancia es menor a **20 cm**.

## 4. Ejemplo Numérico
Supongamos la siguiente situación en el tablero:
*   **Posición Robot:** $(2.0, 1.0)$
*   **Posición Objetivo:** $(5.0, 5.0)$

**Paso 1: Resta Vectorial**
$$ (5.0, 5.0) - (2.0, 1.0) = (3.0, 4.0) $$
El robot "ve" que el objetivo está a **3 metros hacia adelante** y **4 metros a la izquierda**.

**Paso 2: Cálculo de Control**
*   **Distancia ($d$):** $\sqrt{3^2 + 4^2} = 5 \text{ metros}$
*   **Ángulo ($\theta$):** $\arctan(4/3) \approx 0.93 \text{ rad } (53^\circ)$

**Resultado:**
El robot girará **53º a la izquierda** y avanzará **5 metros** en línea recta hasta llegar al destino.
