<h1>Exportar a ROS 2</h1>

A pesar de que sw2urdf genera varios archivos, es necesario hacer algunos cambios para simular nuestro robot.

Los paquetes de ROS 1 y ROS 2 tienen un estandar, donde se recomienda (aunque no es obligatorio) **empezar por una letra minúscula y contener SOLO letras minúsculas, dígitos, guiones bajos y guiones**. No recomiendo los guiones (solo los guines bajos), pero deben seguir esas instrucciones al exportar el robot en SolidWorks. 

<h2>Índice</h2>

<- Para exportar el robot de SolidWorks a URDF, ve a [SolidWorks2URDF](sw2urdf.md)

- [Importar proyecto en Ubuntu](#importar-proyecto-en-ubuntu)
  - [En resumen:](#en-resumen)
- [Copiar a Ubuntu el paquete del robot](#copiar-a-ubuntu-el-paquete-del-robot)
- [Editar CMakeLists.txt](#editar-cmakeliststxt)
- [Editar package.xml](#editar-packagexml)
- [Modificar el URDF](#modificar-el-urdf)
  - [El mundo como eslabón](#el-mundo-como-eslabón)
  - [Transmision](#transmision)
  - [Controlador de Gazebo](#controlador-de-gazebo)


## Importar proyecto en Ubuntu

Entren a Ubuntu e importen el repositorio del proyecto de robótica desde GitHub; recomiendo guardarlo en una carpeta llamada Robotica para que sea más fácil seguir el tutorial.
```bash
git clone https://github.com/UsuarioDeGitHub/RepositorioRobotica.git ~/Robotica
```
Una vez dentro, en Visual Studio Code abran la carpeta desde `Archivo` -> `Abrir carpeta...` para trabajar con GitHub desde ahí.

Ahora, convertirán el proyecto en un Workspace de ROS 2. Un Workspace permite crear paquetes o bibliotecas locales que no interfieran con otros Workspaces, por lo que es algo similar al proyecto de Matlab. Para convertirlo en un Workspace, abre un terminal y asegúrate de estar en la carpeta del proyecto, la pueden llamar como Robotica para no batallar en copiar y pegar (no tiene que ser el mismo de github), donde puedes ir ahí con `cd ~/Robotica` y ejecutar

```bash
colcon build --symlink-install
```

Esto creará las siguientes carpetas:
- `src/`: donde colocamos los **paquetes** (como `mi_robot`, `tutorial` o `sensores_genericos`). Yo les dije que por estandar, ya debíamos tener esa carpeta.
- `build/`: donde se guardan archivos temporales de compilación.
- `install/`: donde se instalan los paquetes ya compilados.
- `log/`: registros de la compilación.

Si está configurado bien `.gitignore`, deberían de verse de color gris y no deberían de aparecer como spam de cientos de modificaciones de git. 

Normalmente no se necesita usar la opción `--symlink-install`, pero la primera vez crea enlaces simbólicos en lugar de copiar los archivos de los paquetes en la carpeta `install`.

El comando `colcon build` se usa para compilar el código máquina de C++, por lo que si un paquete usa C++, es necesario usar el comando cada vez que se cambia el código. Python utiliza algo llamado intérprete que le permite que se lea diréctamente el código en lenguaje de alto nivel, por lo que no es necesario convertirlo a código máquina y, por lo tanto, no es necesairo usar `colcon build` cada vez que se modifique el código. Sin embargo, sí es necesario usarlo al crear un nuevo paquete para que ROS 2 los encuentre automáticamente.

Ahora, cada vez que se abra una terminal, para que use el Workspace, es necesario ejecutar

```bash
source install/setup.bash
```

Pero si solo usarás el workspace, se puede añadir al archivo `.bashrc` para que siempre se ejecute al abrir el terminal con
 ```bash
 echo 'source ~/Robotica/install/setup.bash' >> ~/.bashrc
 source ~/.bashrc
```

O puedes hacerlo desde el navegador de Windows entrando a `Linux\Ubuntu-24.02\home\$USUARIO\.bashrc`, aunque también puedes usar Visual Studio Code o abrir el editor de ubuntu con

```bash
gedit ~/.bashrc
```

Y añade `source install/setup.bash` para que asiempre se ejecute. Recuerda cerrar y abrir el terminal para que haga efecto.

### En resumen:
1. Abrir Ubuntu 24.04.
2. Clonar repositorio en la carpeta Robotica.
```bash
git clone https://github.com/UsuarioDeGitHub/RepositorioRobotica.git ~/Robotica
```

3. Abrir carpeta en Visual Studio Code.
4. Crear Workspace.
```bash
cd ~/Robotica
colcon build --symlink-install
```
5.  Abrirlo por defecto en terminal.
```
echo 'source ~/Robotica/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Copiar a Ubuntu el paquete del robot

Ahora, es necesario crear un paquete con el mismo nombre del que se creó en SolidWorks. Esto es porque no es compatible con la versión de ROS que tenemos. Para ello, ejecutan lo siguiente en la carpeta del proyecto. **Recuerden cambiar MI_ROBOT por el nombre del paquete del robot creado en SolidWorks**

```bash
cd ~/Robotica/src
ros2 pkg create --build-type ament_cmake MI_ROBOT
```

Lo siguiente es copiar todos los archivos y carpetas del paquete generado por `sw2urdf`, **con excepción de** `CMakeLists.txt` y `package.xml`, al generado en la carpeta `src/`. La excepción es porque esos dos archivos se generan automáticamente y la versión que usan está basada en la versión actual de ROS, por lo que recomiendo dejar las lineas que dicen la versión como están. 

## Editar CMakeLists.txt
El archivo `CMakeLists.txt` contiene la configuración de C++, por lo que los nodos programados en ese lenguaje deben añadirse ahí. Solo lo recomiendo si saben C++ o si el control es muy lento. Recuerden cambiar `MI_ROBOT`

```cmake
cmake_minimum_required(VERSION 3.8)
project(MI_ROBOT)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_publisher REQUIRED)

# ——— MoveIt 2 ———
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)             # planning componentes :contentReference[oaicite:0]{index=0}
find_package(moveit_ros_planning_interface REQUIRED)   # interfaz C++
# (si vas a usar MoveIt! visualización o ejecución de pipelines, añade también moveit_ros_planning_execution)

# Instalar carpetas de datos
foreach(dir config launch meshes urdf world rviz)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# Si tienes nodos en C++, decláralos aquí
# add_executable(my_moveit_node src/my_moveit_node.cpp)
# ament_target_dependencies(
#   my_moveit_node
#   rclcpp std_msgs geometry_msgs
#   moveit_ros_planning_interface
# )

ament_package()
```

## Editar package.xml
También tienen que cambiar el archivo `package.xml`. Cambien donde dice `MI_ROBOT`, el autor y correo. Si quieren poner múltiples autores, solo tienen que poner la linea de <author> y <maintainer> varias veces hasta que estén todos los integrantes.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>MI_ROBOT</name>
  <version>0.1.0</version>
  <description>URDF y paquete de configuración del robot MI_ROBOT.</description>
  <author>Ivan Medina</author>
  <maintainer email="jesus.medinag@hermosillo.tecnm.mx" />
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>xacro</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  
  <!-- MoveIt! 2 -->
  <depend>moveit_core</depend>
  <depend>moveit_ros_planning</depend>
  <depend>moveit_ros_planning_interface</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

El archivo `package.xml` contiene una lista de los paquetes necesarios para que este paquete funcione. Si intentan compilar de nuevo con `colcon build`, probablemente aparezca un error porque no tienen instalado un paquete. Es por ello que una práctica común es ejecutar
```bash
cd ~/Robotica
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
para instalar todos los paquetes que se encuentran en `package.xml` y compilarlo.

## Modificar el URDF
Ahora se debe modificar el URDF para que se pueda simular en gazebo. Este archivo describe todos los componentes del robot usando el lenguaje xml. 

En xml, una etiqueta empieza con `<` y termina con `/>`. Si queremos que una etiqueta contenga varias etiquetas dentro, se puede poner en una línea `<etiqueta>` y en otra línea `</etiqueta>` con el mismo nombre pero con `/`, de forma que todo lo que esté dentro de las dos forme parte de `etiqueta`. 

Para más información sobre URDF, pueden ver en la [página de ROS](https://wiki.ros.org/urdf/XML) en detalle. Su estructura básica se puede ver en el siguiente ejemplo:
```xml
<?xml version="1.0"?>
<robot name="robot_genérico">
  <!--
    <robot>
      Describe todas las propiedades de un robot.
  -->

  <!--
    <link>
      Describe las propiedades cinemáticas y dinámicas de un eslabón.
  -->
  <link name="base_link">
    <!-- 
      Aquí irían <inertial>, <visual> y <collision> si quisiéramos masa, geometría, etc.
    -->
  </link>
  <link name="link_1"/>

  <!--
    <joint>
      Describe las propiedades cinemáticas y dinámicas de una articulación.
  -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child  link="link_1"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis   xyz="0 0 1"/>
    <!-- Límites de torque y velocidad -->
    <limit effort="10" velocity="1.0" lower="-1.57" upper="1.57"/>
    <!--
      <dynamics>
        En URDF estándar se usa para damping y friction
    -->
    <dynamics damping="0.1" friction="0.2"/>
  </joint>

  <!--
    <transmission>
      Conecta actuadores (motores) con joints y define relación mecánica.
  -->
  <transmission name="joint1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1"/>
    <actuator name="motor1">
      <!-- gear ratio: torque_joint = torque_motor × reduction -->
      <mechanicalReduction>10</mechanicalReduction>
    </actuator>
  </transmission>

  <!--
    <gazebo>
      Bloque de configuración específico para simulación en Gazebo:
      aquí anidamos sensores, plugins, fricción/damping a nivel de link/joint, etc.
  -->
  <gazebo reference="link_1">
    <!--
      <sensor>
        Describe un sensor (ej. láser, cámara) dentro de Gazebo.
    -->
    <sensor type="ray" name="laser_sensor">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <min_angle>-1.57</min_angle>
            <max_angle> 1.57</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>5.0</max>
        </range>
      </ray>
    </sensor>
  </gazebo>

</robot>
```

### El mundo como eslabón
Lo primero es fijar el robot en el mundo para que no se tambalee ni se caiga. Para eso se añade el eslabón `world` y la articulación `base_joint`. Cabe destacar que si nombraron diferente a la primera articulación, entonces no debe ir `base_link` (y probablemente tampoco `base_joint`), sino algo así como `link_0` y `joint_0`.
```xml
<robot
  name="MI_ROBOT">
```
```xml
  <link name="world"/>
  <joint name="base_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0.0 0.0 0.17"/>
  </joint>
```
```xml
  <link
    name="base_link">
    <inertial>
```

### Transmision
La etiqueta de transmisión es necesaria para usar `ros_control`, pero permite simular una caja de engranajes o un reductor de velocidad.

Pueden agregarlas antes de `</robot>`, que es la última etiqueta. Debe ser una por cada articulación (si tienen 4 articulaciones, deben de copiar y pegar el código 4 veces) y solo deben cambiar `joint_n` y `link_n` con el número correspondiente.

```xml
  <transmission name="link_n_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_n">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="link_n_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```
```xml
</robot>
```
- `<mechanicalReduction>`: es el gear ratio, es decir, la relación entre el eje del motor y la articulación.
  - Si pones `1`, dices que no hay transmisión (velocidad y torque uno a uno).
  - Si tu caja de engranajes es, por ejemplo, 10:1, pones `10`.
- **Efecto sobre torque y velocidad**:
  - **Torque en el joint** = *torque del motor* × **mechanicalReduction**
  - **Velocidad del joint** = v*elocidad del motor* ÷ **mechanicalReduction**

Pueden poner que los motores tienen ya la fuerza total y que la transmisión solo tiene una reducción mecánica de `1`, pero si quieren hacerlo más realista, pueden poner la fuerza y velocidad máxima del motor que viene en la hoja de datos dentro de cada articulación en 
```xml
  <joint
    name="joint_n"
    type="revolute">
    <limit
      effort="999"
      velocity="999" />
  </joint>
```
y el gear ratio que diga la caja de engranajes que usen dentro de `<mechanicalReduction>`.

### Controlador de Gazebo
La etiqueta <gazebo> es para parámetros específicos del simulador homónimo. Pueden ser parámetros globales como el que está abajo o para algún eslabón o articulación específica, como el que viene en el ejemplo de [Modificar el URDF](#modificar-el-urdf) con `<gazebo reference="link_1">`.

Podemos hacer que el simulador Gazebo haga todas las cosas avanzadas que queramos usando plugins. Algunos ya están hechos en Gazebo como el que aparece aquí, pero podríamos programar los nuestros en python o C++. 

Solo tienen que añadir esto antes de `</robot>`.
```xml
  <gazebo>
    <plugin 
      name="control"
      filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>
```
```xml
</robot>
```
