# Carga Eléctrica Variable

Este proyecto consiste en el diseño y cálculo analítico de un sistema de calefacción basado en resistencias eléctricas, con el objetivo de determinar su viabilidad térmica y asegurar una distribución de calor eficiente y segura.

## Resumen del Proyecto y Logros

Se han completado los principales objetivos del proyecto, abarcando desde los cálculos térmicos iniciales hasta la documentación y el diseño CAD final:

1. **Selección de Componentes:**
   - Se seleccionaron **6 resistencias eléctricas tipo U** de **1000 W** y **120 V** cada una (capacidad total: 6000 W).
   - Para mejorar la disipación térmica, se emplearon resistencias con **disipadores de calor integrados** (aletas rectangulares), garantizando que operen de forma segura por debajo de su límite máximo de 200 °C.

2. **Modelado Analítico y Cálculo Térmico:**
   - Se desarrolló un script en Python (`Calculos/Calculos_Ventilador.py`) para analizar la transferencia de calor por convección forzada externa.
   - El sistema se modeló como un **banco de tubos escalonado**, teniendo en cuenta la geometría del disipador de calor (eficiencia de aletas, zonas desnudas y espaciado).
   - Se realizó un **estudio paramétrico** evaluando configuraciones de 2 y 4 tubos por fila, iterando sobre distintas velocidades del aire y evaluando caídas de presión y potencia del ventilador.
   - **Resultado:** Se determinó que la **Configuración B (1 resistencia por fila, 6 filas en total)** con un paso longitudinal ($S_L$) de **4 cm** es la óptima, minimizando el esfuerzo requerido por los ventiladores.

3. **Diseño Mecánico y Seguridad:**
   - Se completó el diseño del **modelo CAD de la caja** que contiene las resistencias y los ventiladores.
   - Se seleccionaron e integraron materiales de seguridad: **revestimiento térmico aislante** para el interior de la carcasa (para evitar quemaduras por contacto exterior) y **cables resistentes a altas temperaturas**.

4. **Documentación:**
   - Todo el proceso, cálculos, normativas y esquemas han sido documentados formalmente en la carpeta `Documentacion/`, facilitando su revisión y presentación.

## Estructura del Repositorio

- `Calculos/`: Contiene los scripts en Python (`Calculos_Ventilador.py`) utilizados para el cálculo del banco de tubos escalonado y las propiedades del aire evaluadas con *CoolProp*.
- `Documentacion/`: Contiene el informe técnico detallado del sistema en formato `.tex`, junto con las imágenes del diseño y resultados óptimos.
- `Fichas Tecnicas/`: Especificaciones del fabricante para las resistencias, ventiladores, cableado y material aislante térmico.
- `Modelos CAD/`: Archivos 3D y ensamblajes de la caja contenedora del sistema de resistencias.
- `Planos/`: Planos técnicos derivados del modelo CAD para la fabricación y ensamble de la caja.

---
*Para ver el informe detallado de las fórmulas empleadas y las conclusiones completas, revisa el archivo de documentación en la carpeta correspondiente.*
