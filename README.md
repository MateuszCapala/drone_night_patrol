# Drone Night Patrol — Symulacja autonomicznego drona w ROS 2 i Ignition Gazebo

**Projekt:** Symulacja i sterowanie dronem patrolującym granice w środowisku nocnym
**Autor:** Mateusz
**Data:** 2025
**Technologie:** ROS 2 | Gazebo Classic | Python | URDF/Xacro

---

## Funkcjonalności

* Symulacja autonomicznego drona w środowisku nocnym.
* Integracja z PX4 SITL.
* Obsługa dodatkowych modeli środowiska.
* Wytrenowanie i testowanie symulacyjne modelu wykrywania człowieka w symulowanej kamerze termowizyjnej.

---

## Struktura i zmiany w plikach

### 1. `empty.world`

W pliku `empty.world` dokonano następujących zmian w modelach środowiska:

- Usunięto domyślne modele `ground_plane` i `asphalt_plane`.
- Dodano modele `sun`, `dywan`, `intruz` oraz `bush_0` i `bush_1`.
- Dodano 22 modele budynków (`building_1` do `building_22`) o ciemnych teksturach, aby symulować warunki nocne.

Przykładowe inkluzje modeli w `empty.world`:
```xml
<include>
  <uri>model://sun</uri>
</include>

<include>
  <uri>model://dywan</uri>
  <pose>0 0 0.01 0 0 0</pose>
</include>

<include>
  <uri>model://intruz</uri>
  <pose>-10.91 80.44 0.03 0 0 0</pose>
</include>

<include>
  <uri>model://bush_0</uri>
  <name>bush_instance_1</name>
  <pose>-10.91 80.44 0.03 0 0 0</pose>
</include>
```

### 2. Foldery modeli PX4

W katalogu `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/` dodano:

* `dywan`
* `intruz`

Foldery umieszczone są w katalogu `resources` i są dostępne w repozytorium GitHub.

### 3. Kamera Realsense

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

### 4. Definiowanie trasy patrolu

Trasa patrolu drona jest zdefiniowana w pliku `src/drone_sim/resource/waypoints.json`. Plik ten zawiera współrzędne geograficzne (szerokość, długość) oraz wysokość dla punktu startowego (`origin`) i kolejnych punktów nawigacyjnych (`waypoints`).

Struktura pliku `waypoints.json`:
```json
{
  "origin": {
    "latitude": 51.507912,
    "longitude": 23.616133,
    "altitude": 10.0
  },
  "waypoints": [
    {
      "latitude": 51.507912,
      "longitude": 23.616133,
      "altitude": 22.0
    },
    ...
}
```

- **`origin`**: Punkt startowy drona.
- **`waypoints`**: Lista punktów, które dron będzie kolejno odwiedzał w trakcie patrolu. Ostatni punkt powinien być taki sam jak pierwszy, aby dron wrócił na pozycję początkową.

---

## Modele i trening

### Modele krzaków

Modele krzaków (`bush_0` i `bush_1`) zostały zapożyczone z repozytorium [gazebo-vegetation](https://github.com/kubja/gazebo-vegetation.git). Ich parametry widoczności zostały zmodyfikowane na potrzeby projektu.

### Trening modelu YOLO

Model `yolov8n.pt` został wytrenowany na bazie danych *S&R POP infrared Dataset*.

```
@misc{
    dataset,
    title = { S&R POP infrared Dataset },
    type = { Open Source Dataset },
    author = { thermalvisionUAV },
    howpublished = { \url{ https://universe.roboflow.com/thermalvisionuav/s-r-pop-infrared-qdzfc } },
    url = { https://universe.roboflow.com/thermalvisionuav/s-r-pop-infrared-qdzfc },
    journal = { Roboflow Universe },
    publisher = { Roboflow },
    year = { 2025 },
    month = { oct },
    note = { visited on 2026-01-17 },
}
```

---

## Instalacja ROS 2 i budowanie paczki

1. Sklonuj swoje repozytorium ROS:

```bash
git clone https://github.com/MateuszCapala/drone_night_patrol.git
cd https://github.com/MateuszCapala/drone_night_patrol.git
```

2. Zbuduj workspace ROS 2:

```bash
colcon build
```

3. Źródłuj środowisko:

```bash
source install/setup.bash
```



---

## Instalacja PX4

Instrukcje instalacji PX4 SITL znajdziesz w oficjalnym repozytorium PX4:
[PX4 Autopilot — Gazebo Classic SITL Installation](https://docs.px4.io/master/en/simulation/gazebo.html)


---

## Uwagi

* Należy aby wskazać ścieżki do PX4 w launch file ROS.



## Uruchomienie symulacji:
Po odpowiedniej konfiguracji całej paczki wraz z PX:

```bash
ros2 launch drone_sim drone_sim_launch.launch.py
ros2 run gazebo_ros spawn_entity.py \
  -file <ścieżka>/intruz/moving_image.sdf \
  -entity my_image
ros2 run drone_sim offboard
```
