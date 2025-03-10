- [Exportar Robot de SolidWorks a URDF](#exportar-robot-de-solidworks-a-urdf)
  - [SolidWorks: Eje Z vertical](#solidworks-eje-z-vertical)
  - [SolidWorks: Relación de posición](#solidworks-relación-de-posición)
  - [SolidWorks: Ejes de articulaciones](#solidworks-ejes-de-articulaciones)
  - [Solidworks: Puntos](#solidworks-puntos)
  - [SolidWorks: Sistemas de referencia](#solidworks-sistemas-de-referencia)

# Exportar Robot de SolidWorks a URDF
Para poder hacer nuestro robot en SolidWorks y poder usarlo en ROS con MoveIt, es necesario seguir unos pasos dentro de SolidWorks y no basta símplemente con exportarlo.

## SolidWorks: Eje Z vertical
Primero, se recomienda cambiar el eje vertical para que sea el eje Z en vez del eje Y. Esto nos evitará unos pasos adicionales más tarde ya que en gazebo, el eje Z es el eje vertical.

Para hacer esto, selecciona en `Orientación de vista` y en `Más opciones`.

![Orientación de vista, más opciones](assets/orientacion_mas_opciones.png)

Después en `Desplegable de eje superior` y `Aplicar vistas Z-arriba`.

![Aplicar vistas Z-arriba](assets/Z_arriba.png)

## SolidWorks: Relación de posición
Aquí simplemente se realizarán las relaciones de posición de toda la vida para ensamblar el robot de la forma inicial que queremos, con la diferencia de que al cambiar el eje Z, se tendrá que rotar el robot. Si ya lo tenías ensamblado pero ahora está volteado, puede que sea más fácil. 

Probablemente la primera pieza (mínimo una de ellas) se encuentra fija. Lo mejor es flotar la pieza para que se pueda mover libremente. Si no está completamente definido o solo estaban todas fijas, te deseo suerte para que no ocurra un desastre al voltearlo.

![Flotar pieza](assets/Flotar.png)

Luego, puedes rotarlo usando `Mover con el sistema de referencia`

![Menú sistema de referencia](assets/Sistema_referencia_1.png)

![Sistema de referencia](assets/Sistema_referencia_2.png)

O mejor aun, fijar los plano principales (frontal, superior y derecho) y asegurarte de que están alineados correctamente.

![Planos alineados](assets/alinear_planos.png)

En caso de que contenga muchas piezas, recomienda formar un subensamble y fijar las piezas dentro del subensamble, lo que permitirá que parezca un eslabón completo y causará menos problemas al manipular las piezas o seleccionar el eslabón (incluso puedes llamarlo como `eslabón_1` o algo así).

![Subensamble](assets/Subensamble.png)

Una vez completado todo lo anterior, asegúrate de que todos los eslabones tengan sus relaciones concéntricas, coincidentes y parlelas para que los ejes de cada eslabón coincidan con los de la articulación y no tengan juego. 

![Relación de posición](assets/relaciones_posicion.png)

De ser posible, las articulaciones de revoluta deberían de tener rotaciones de 90° y las articulacinoes prismáticas números enteros, pero eso puede esperar a que los ejes estén bien definidos en el siguiente paso.

## SolidWorks: Ejes de articulaciones
Lo siguiente ya se parece al sistema de Denavit-Hartenberg. De ser posible, recomiendo que, antes de seguir con SolidWorks, hagan un diagrama que simplifique cómo irán los sistemas de referencia y que lo prueben en matlab para ver si lo hicieron correctamente, ya que corregir cosas en SolidWorks es más complicado.

Para poner los ejes, ve  a `Geometría de referencia` para añadir un `Eje`.

![Geometría de referencia: Ejes](assets/Geometria_eje.png)

Usa las reglas de Denavit-Hartenberg para poner los ejes. Estos suelen ser concéntricos a alguna pieza donde se encuentra al motor en el caso de articulaciones de revoluta, mientras que en articulaciones prismáticas, solo importa que sean paralelas y no necesariamente que se encuentren en coincidencia con determinado punto.

![Eje concéntrico](assets/Eje_concentrico.png)

 A veces no está bien definido en articulaciones rotativas y tendrás que crear un croquis y hacerlo en base a puntos.

![Eje paralelo](assets/Eje_paralelo.png)

También cabe destacar que en la pinza, si el movimiento final es lineal pero utiliza un sistema de biela manivela con un servomotor (revoluta), como nos estamos enfocando en una pinza que se activa y desactiva sin un control preciso, es innecesario complicarse. Simplemente quita las partes que rotan y deja la parte que se mueve linealmente. No es necesario hacer tantos cálculos para un movimiento de abrir y cerrar. Solo toma nota de la abertura máxima que tiene para ponerla después en los límites.

![Pinza con manivela](assets/con_manivela.png)

![Pinza sin manivela](assets/sin_manivela.png)

El robot anterior se encuentra [Aquí](https://cults3d.com/en/3d-model/gadget/era-robotic-arm).

## Solidworks: Puntos
Para poner los puntos de los sitemas de referencia, se deben de tomar en cuenta los 3 casos distintos: ejes coincidentes, ejes paralelos y ejes no coplanares.

Cabe destacar que el `Punto 0` suele ir en lo más abajo del robot o en caso de tener una mesa de trabajo, a la altura de la mesa, pero siempre coincidente con el primer eje.
Ejemplo de ejes coincidentes:

![Ejes coincidentes](assets/Ejes_coincidentes.png)

Ejemplo de ejes paralelos:

![Ejes paralelos](assets/Ejes_paralelos.png)

En los ejes no coplanares, **siempre** tendrán que hacer un croquis para dibujar la linea perpendicular a los dos ejes. Hagan que todo con excepción de esa línea sea de construcción para que sea más fácil de seleccionar el punto y la dirección del eje X.

No olviden el punto y eje donde se sostendrá el objeto.

![Punto final pinza](assets/Punto_final_pinza.png)

## SolidWorks: Sistemas de referencia
Lo siguiente es definir los sistemas de referencia. Estos se posicionan en el punto que hemos definido y el eje Z va en dirección al eje. Para el eje X, seguimos los pasos mencionados en le sistema de Denavit hartenberg según uno de los tres casos y podemos hacer que el eje X vaya en dirección a una cara. A veces se voltea de forma extraña, así que asegúrate de que lo haga correctamente.

También recuerda visualizar los las piezas y componentes para que sea más fácil seleccionar los ejes. Aquí se muestra un ejemplo donde se puso el eje Z hacia abajo, por lo que se tiene que voltear con el botón de cambiar dirección del eje.

![Sistema de referencia](assets/Sistema_referencia.png)

