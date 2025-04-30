# Proyecto final arquitectura software para robots: Escondite
### Carla García Alejandre, Elías Muñoz Taín, Miguel Ángel Piña Martínez y María Yagüe LLamas

## 1. Introducción
Este proyecto plantea el desarrollo de un juego interactivo de escondite utilizando un **Kobuki**. La propuesta consiste en mapear un aula como espacio de juego y delimitar seis escondites fijos empleando los corchos disponibles en el laboratorio. Durante cada partida, varios jugadores se ocultarán en esos puntos predefinidos mientras el Kobuki los busca.  

El robot dispondrá de un número de intentos igual al número de jugadores más uno para localizar a todos los participantes. Para ello, se integrará el modelo de visión por computador `YOLO`, que permitirá al Kobuki detectar la presencia de personas en cada escondite y llevar un registro del número de jugadores encontrados.  

Además, se prevé incorporar un sistema de diálogo para dotar al Kobuki de una interacción verbal básica: contará en voz alta al inicio del juego, reaccionará cuando detecte a un jugador y anunciará el resultado final. Si el robot logra encontrar a todos los jugadores antes de agotar sus intentos, ganará la partida; en caso contrario, la victoria será para los jugadores.  

## 2. Espacio de juego
Hemos realizado el mapeo del pasillo de los laboratorios de la universidad para delimitar seis escondites. Utilizamos un robot `Kobuki` ejecutando ROS 2, que recorrió el pasillo recogiendo datos con sus sensores.

### Creación del mapa
Para generar el mapa, utilizamos el sistema de navegación `nav2`. En terminales diferentes, se lanzan los isguientes comandos.

**Lanzar Kobuki**
Primero lanzaremos el robot en un punto concreto, para determinar nuestro origen de coordenadas.

```bash
ros2 launch kobuki kobuki.launch.py astra:=true lidar_a2:=true
```

En nuestro caso fue la puerta del laboratorio.

**RViz 2**
Lanzamos en otra terminal `rviz2` y añadimos el topic `Map` para poder ir visualizando el mapa que va generando.

**Nodo SLAM**
El mapeo se hizo usando un nodo de SLAM (como slam_toolbox) para generar un mapa 2D del entorno. 

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=src/kobuki/config/kobuki_nav.yaml
```

**Mapa**
Lanzamos un nodo que se suscribirá a `/map` el cual uardará en disco cuando se solicite el mapa que se está generando.

```bash
ros2 launch nav2_map_server map_saver_server.launch.py
```

Controlamos al robot mediante teleoperación mientras visualizamos el proceso en RViz2. De manera opcional podemos añadir un teclado o un mando para trabajar más cómodos.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Una vez obtenido el mapa, lo guardaremos:

```bash
ros2 run nav2_mao_server map_saver_cli
```

Este último comando nos generará un archivo `.yaml` que será el que lanzaremos más adelante para utilizar el mapa y otro archivo `.pgm` que podremos editar con herramientas como `gimp` para limpiar el ruido generado.

![Mapa universidad](./img/mapa.png)

## 3. Behaviour Tree

## 4. Navegación y WayPoints

## 5. Yolo

## 6. Sistema de diálogo

## 7. Demostración