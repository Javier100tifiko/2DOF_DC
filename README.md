# Robot Planar de 2 Grados de Libertad

Este proyecto implementa el control de un robot planar de 2 grados de libertad (GDL) utilizando un ESP32 y MATLAB. El sistema permite mover el robot a posiciones específicas en el plano XY, así como definir trayectorias de movimiento con un número determinado de pasos.

## Contenido del Repositorio

- Código para el ESP32.
- Código de MATLAB para la cinemática inversa y visualización.

## Requisitos

- **Hardware:**
  - ESP32
  - Motores con encoders
  - Robot planar de 2 GDL

- **Software:**
  - MATLAB
  - Arduino IDE (o cualquier otra herramienta compatible para programar el ESP32)

## Instrucciones

### Código Arduino

El código para el ESP32 se encuentra en este repositorio, se recomienta usar 'ESP32_Actual'. Este código se encarga de recibir los ángulos de los motores desde MATLAB y controlar los motores para mover el robot a las posiciones deseadas.

### Código MATLAB

Este script permite calcular la cinemática inversa del robot, visualizar el movimiento y enviar comandos al ESP32.

#### Uso del Script de MATLAB

1. **Configurar la comunicación serial:**

   Modifica la línea donde se especifica el puerto de comunicación serial para que coincida con el puerto utilizado por tu ESP32:

   ```matlab
   port = "COM3"; % Cambia "COM3" por el puerto correspondiente
   ```

2. **Ejecutar el script:**

   Abre MATLAB y ejecuta el script. Se abrirá una ventana de gráfico donde podrás visualizar el robot y seleccionar diferentes opciones para moverlo.

3. **Opciones del menú:**

   - **Mover a posición XY:** Permite ingresar coordenadas X e Y para mover el robot a la posición deseada.
   - **Definir posición inicial y final con pasos:** Permite definir un trayecto desde una posición inicial hasta una final en un número específico de pasos.
   - **Posición inicial:** Mueve el robot a su posición inicial.
   - **Desconectar:** Finaliza la comunicación y cierra el script.

## Funciones Principales

### MATLAB

- `cinematica_inversa_2gdl(x, y, L1, L2)`: Calcula los ángulos de las articulaciones para una posición dada.
- `graficar_robot(theta1, theta2, L1, L2, x, y)`: Grafica el estado actual del robot.
- `serialesp(theta1, theta2, esp)`: Envía los ángulos calculados al ESP32 a través de la comunicación serial.

### Arduino

- **Control de motores:** Recibe los comandos seriales y ajusta los motores para alcanzar los ángulos especificados.

## Notas

- Asegúrate de que el ESP32 esté correctamente conectado y configurado para comunicarse a través del puerto serial con MATLAB.
- Verifica que las longitudes de los eslabones `L1` y `L2` estén correctamente definidas en el código para asegurar un control preciso del robot.

## Contribuciones

Las contribuciones son bienvenidas. Siéntete libre de abrir un issue o enviar un pull request.

## Licencia

Este proyecto está bajo la licencia MIT. Para más detalles, consulta el archivo [LICENSE](LICENSE).
