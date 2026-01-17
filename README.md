# Drone Night Patrol — Symulacja autonomicznego drona w ROS 2 i Ignition Gazebo

**Projekt:** Symulacja i sterowanie autonomicznym dronem w środowisku nocnym
**Autor:** Mateusz
**Data:** 2025
**Technologie:** ROS 2 | Ignition Gazebo | Python | URDF/Xacro

---

## Funkcjonalności

* Symulacja autonomicznego drona w środowisku nocnym.
* Integracja z PX4 SITL.
* Kamera frontowa na dronie (`iris_opt_flow`) z pluginem ROS.
* Integracja kamery Realsense z pluginem ROS.
* Obsługa dodatkowych modeli środowiska (`dywan`, `intruz`).

---

## Struktura i zmiany w plikach

### 1. `empty.world`

Zmiana modeli w środowisku:

```xml
<include>
  <uri>model://dywan</uri>
  <pose>0 0 0.01 0 0 0</pose>
</include>
```

Zastąpiono oryginalne:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
<include>
  <uri>model://asphalt_plane</uri>
</include>
```

### 2. `iris_opt_flow.sdf`

Dodano kamerę frontową:

```xml
<link name="camera_link">
  <pose>0.1 0 0.05 0 0 0</pose>
  <sensor name="front_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    ...
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/iris_opt_flow/front_camera</namespace>
        <remapping>image:=image_raw</remapping>
      </ros>
      <update_rate>30.0</update_rate>
      <camera_name>front_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</link>

<joint name="camera_joint" type="fixed">
  <parent>iris::base_link</parent>
  <child>camera_link</child>
</joint>
```

### 3. Foldery modeli PX4

W katalogu `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` dodano:

* `dywan`
* `intruz`

Foldery umieszczone są w katalogu `resources` i będą dostępne w repozytorium GitHub.

### 4. Kamera Realsense

W pliku `realsense_camera/model.sdf` dodano plugin ROS:

```xml
<plugin name="realsense_ros_camera" filename="libgazebo_ros_camera.so">
  <ros2_namespace>/realsense</ros2_namespace>
  <image_topic_name>color/image_raw</image_topic_name>
  <camera_info_topic_name>color/camera_info</camera_info_topic_name>
  <frame_name>realsense_link</frame_name>
  <update_rate>60.0</update_rate>
</plugin>
```

---

## Instalacja ROS 2 i budowanie paczki

1. Sklonuj swoje repozytorium ROS:

```bash
git clone <TWÓJ_REPO_ROS>.git
cd <TWÓJ_REPO_ROS>
```

2. Zbuduj workspace ROS 2:

```bash
colcon build
```

3. Źródłuj środowisko:

```bash
source install/setup.bash
```

4. Uruchomienie symulacji:

```bash
ros2 launch drone_sim drone_sim_launch.launch.py
ros2 run gazebo_ros spawn_entity.py \
  -file <ścieżka>/intruz/moving_image.sdf \
  -entity my_image
ros2 run drone_sim offboard
```

---

## Instalacja PX4

Instrukcje instalacji PX4 SITL znajdziesz w oficjalnym repozytorium PX4:
[PX4 Autopilot — Gazebo Classic SITL Installation](https://docs.px4.io/master/en/simulation/gazebo.html)


---

## Uwagi

* Należy aby wskazać ścieżki do PX4 w launch file ROS.


Launch:
ros2 launch drone_sim drone_sim_launch.launch.py 
ros2 run gazebo_ros spawn_entity.py   -file /home/mateusz/Desktop/sem2_magisterka/drony/PX4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/intruz/moving_image.sdf   -entity my_image
ros2 run drone_sim offboard 
