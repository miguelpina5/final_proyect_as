# Proyecto final arquitectura software para robots: Escondite
### Carla García Alejandre, Elías Muñoz Taín, Miguel Ángel Piña Martínez y María Yagüe LLamas

## 1. Introducción
Este proyecto plantea el desarrollo de un juego interactivo de escondite utilizando un Kobuki. La propuesta consiste en mapear un aula como espacio de juego y delimitar seis escondites fijos empleando los corchos disponibles en el laboratorio. Durante cada partida, varios jugadores se ocultarán en esos puntos predefinidos mientras el Kobuki los busca.

El robot dispondrá de un número de intentos igual al número de jugadores más uno para localizar a todos los participantes. Para ello, se integrará el modelo de visión por computador `YOLO`, que permitirá al Kobuki detectar la presencia de personas en cada escondite y llevar un registro del número de jugadores encontrados.

Además, se prevé incorporar un sistema de diálogo para dotar al Kobuki de una interacción verbal básica: contará en voz alta al inicio del juego, reaccionará cuando detecte a un jugador y anunciará el resultado final. Si el robot logra encontrar a todos los jugadores antes de agotar sus intentos, ganará la partida; en caso contrario, la victoria será para los jugadores.

## 2. Espacio de juego
![Mapa universidad](./img/mapa.png)