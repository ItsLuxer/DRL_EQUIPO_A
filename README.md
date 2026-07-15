# 🚀 DRL Equipo A

¡Hola! 😁 Somos estudiantes del **Centro de Enseñanza Técnica Industrial (CETI)**, actualmente cursando la asignatura de **Control I**.

<p align="justify">
El objetivo principal de este proyecto es el diseño y construcción de un sistema aéreo no tripulado (dron) desde cero. Para fundamentar el desarrollo y comprender la integración de sus subsistemas, hemos iniciado este proceso mediante la ingeniería inversa de un dron comercial. Este análisis técnico nos ha permitido identificar, documentar y comprender la arquitectura, los componentes electrónicos y los protocolos de comunicación críticos necesarios para el funcionamiento y control de vuelo de un dron.
</p>

## 🛠 Equipo de Gestión y Desarrollo

Este proyecto es posible gracias al trabajo colaborativo del siguiente equipo de ingenieros:

* **Ingeniero Control:** José Eduardo

* **Ingeniero Mecánico:** Héctor Manuel

* **Ingeniero Eléctrico:** Héctor Cruz

* **Ingeniero Software:** José Alberto

* **Ingeniero Comunicaciones:** Darío Ibarra

---

# 🚁 Compilación de Betaflight 4.5.2 para STM32F411 (BlackPill)

**Objetivo:** Compilar un firmware funcional de Betaflight 4.5.2 para una placa de desarrollo genérica STM32F411 (BlackPill) en un entorno Windows utilizando MSYS2, evitando errores de estructura de directorios y dependencias de Python.

---

## 📋 Requisitos Previos
1. Sistema operativo **Windows**.
2. Entorno **MSYS2** instalado.
3. Consola **UCRT64** de MSYS2 abierta. Todo el proceso se realizará exclusivamente en esta terminal.

---

## 🛠️ Paso 1: Preparación del entorno y dependencias
Antes de descargar el código, es necesario asegurar que el sistema cuenta con todas las herramientas necesarias para la compilación y la ejecución de scripts internos.

```bash
# 1. Actualizar el sistema base de MSYS2
pacman -Syu

# 2. Instalar Git, Make, Unzip y Python3
pacman -S git make unzip python3
```

---

## 📥 Paso 2: Descarga del código fuente de Betaflight
Crearemos un directorio de trabajo limpio en la raíz del disco duro y descargaremos la versión exacta del código que vamos a compilar.

```bash
# 1. Navegar a la raíz y crear la carpeta de trabajo
cd /c/
mkdir betaflight
cd betaflight

# 2. Clonar el repositorio oficial (incluyendo submódulos)
git clone --recursive https://github.com/betaflight/betaflight.git betaflight-4.5.2

# 3. Entrar a la carpeta descargada
cd betaflight-4.5.2

# 4. Cambiar a la rama de la versión estable 4.5.2
git checkout -b release-4.5.2 tags/4.5.2
```

---

## ⚙️ Paso 3: Instalación de la cadena de herramientas (ARM SDK)
Betaflight requiere una versión específica del compilador GCC cruzado para arquitecturas ARM. Descargaremos esta herramienta y ajustaremos su directorio.

```bash
# 1. Descargar y extraer el compilador GCC ARM automáticamente
make arm_sdk_install

# 2. Corregir el error de anidación de carpetas (mover archivos un nivel arriba)
mv tools/gcc-arm-none-eabi-10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10/* tools/gcc-arm-none-eabi-10.3-2021.10/

# 3. Añadir el compilador a las variables de entorno (PATH)
export PATH=$PATH:/c/betaflight/betaflight-4.5.2/tools/gcc-arm-none-eabi-10.3-2021.10/bin
```

---

## 📂 Paso 4: Obtención de la configuración personalizada
Para que Betaflight reconozca la placa BlackPill, necesitamos un perfil de hardware (target) que defina los pines y puertos de la STM32F411.

```bash
# 1. Salir momentáneamente a la carpeta principal de trabajo
cd /c/betaflight

# 2. Clonar el repositorio comunitario con la configuración de la placa
git clone https://github.com/anlgncr/custom-betaflight-board.git
```

---

## 🧩 Paso 5: Integración en la estructura de Betaflight 4.5+
A partir de la versión 4.5, el sistema de compilación cambió. Las configuraciones de hardware ya no se ubican en `src/main/target`, sino que utilizan el sistema de configuración unificado.

```bash
# 1. Regresar a la carpeta del código fuente de Betaflight
cd betaflight-4.5.2

# 2. Crear el directorio para configuraciones personalizadas (si no existe)
mkdir -p src/config/configs

# 3. Copiar la configuración de la BlackPill a la nueva ruta
cp -r /c/betaflight/custom-betaflight-board/BLACKPILL src/config/configs/
```

---

## 🚀 Paso 6: Compilación del Firmware
Con el entorno limpio, el compilador configurado y la placa en la ruta correcta, procederemos a compilar.

> ⚠️ **Importante:** En la versión 4.5.2, para compilar configuraciones personalizadas, se debe utilizar el parámetro `CONFIG=` en lugar del antiguo `TARGET=`.

```bash
# 1. Limpiar el entorno de compilaciones o cachés anteriores
make clean

# 2. Iniciar la compilación final para la BlackPill
make CONFIG=BLACKPILL
```

---

## 🎉 Paso 7: Resultados y Flasheo
Si el proceso se realizó correctamente, la terminal finalizará sin errores y mostrará el uso de memoria (FLASH y RAM) del procesador. El archivo compilado listo para ser cargado en la placa se generará en la siguiente ruta:

```
/c/betaflight/betaflight-4.5.2/obj/betaflight_4.5.2_STM32F411_BLACKPILL.hex
```