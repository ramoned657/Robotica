# Crear el Workspace de Catkin
Si aun no tienes el workspace, puedes crearlo con
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
Luego, agrega el workspace a tu ~/.bashrc para que ROS lo reconozca automáticamente:
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Crear un Paquete para el Robot
Ahora, dentro de src/, creamos un paquete para nuestro robot:
```bash
cd ~/catkin_ws/src
catkin_create_pkg mi_robot gazebo_ros roscpp std_msgs
```
Este paquete `mi_robot` tendrá soporte para Gazebo, ROS y mensajes estándar.
Luego, compila el workspace para actualizar la estructura:
```bash
cd ~/catkin_ws
catkin_make
```

# 3. Crear la Carpeta del Modelo en el Paquete
Dentro de `mi_robot`, crea la estructura para tu robot en Gazebo:
```bash
cd ~/catkin_ws/src/mi_robot
mkdir -p models/mi_robot
mkdir -p worlds
mkdir -p launch
mkdir -p urdf
mkdir -p scripts
```
Esto organiza el proyecto en carpetas para modelos, mundos, archivos URDF y scripts.

# 4. Definir el Mundo en SDF
Entra a la carpeta `models/mi_robot` y crea el archivo `building_robot.sdf` como se muestra en el [tutorial para construir tu robot](https://gazebosim.org/docs/latest/building_robot/)
```xml
<?xml version="1.0" ?>
<sdf version="1.10">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```

Guarda el archivo y ve al directorio en el que está con `cd`  para ejecutarlo en el simulador con
```bash
gz sim building_robot.sdf
```
**Nota**: Puedes nombrar el archivo como quieras y donde quieras e igualmente podrás abrirlo, pero al hacerlo en el paquete, puedes ver un pcoo cómo se organiza.
Deberías ver un mundo vacío con sólo un plano de tierra y una luz solar. Echa un vistazo a la demo de Mundo para aprender a construir tu propio mundo.

Bajo la etiqueta </model> añadiremos nuestro modelo de robot de la siguiente forma: