#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import time
from mavros_msgs.msg import GlobalPositionTarget
from geometry_msgs.msg import Point
import json
import os
import math
from .submodules.geo_utils import geodetic_to_enu
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class OffboardFSM(Node):
    def __init__(self):
        super().__init__('offboard_fsm')

        self.current_state = "INIT"
        self.state_msg = State()
        self.target_pose = PoseStamped()
        self.rate_hz = 20.0
        self.publish_timer = None

        self.human_position = Point()
        self.human_position_sub = self.create_subscription(Point,'/human_position_xy',self.human_position_cb,10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.alarm_pub = self.create_publisher(String, '/alarm', 10)
        
        # Dodaj subskrypcję do rzeczywistej pozycji drona z poprawnym QoS
        # Ustawienia QoS kompatybilne z MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.current_position = PoseStamped()
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.position_cb, 
            qos_profile
        )
        # Subskrypcja wyników detekcji (JSON) do zliczania liczby osób
        self.last_detection_count = 0
        self.human_detection_sub = self.create_subscription(
            String,
            '/human_detection',
            self.detection_cb,
            10
        )
        

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Brak zależności od stabilności kamery — używamy tylko przechyłu drona

        # Nowe zmienne dla współrzędnych geograficznych
        try:
            package_share_directory = get_package_share_directory('drone_sim')
            self.waypoints_file_path = os.path.join(package_share_directory, 'resource', 'waypoints.json')
        except Exception:
            # Fallback dla developmentu - użyj ścieżki źródłowej
            self.waypoints_file_path = os.path.join(
                os.path.dirname(__file__), '..', '..', 'resource', 'waypoints.json'
            )
        self.local_waypoints = []
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0
        
        # Wczytaj współrzędne geograficzne z pliku
        self.load_geo_waypoints()

        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0

        # --- NOWE PARAMETRY KONTROLI PRZYSTANKÓW I STABILIZACJI ---
        self.segment_stop_interval_s = 10.0     # co ile sekund robić postój kontrolny (nieużywane po interpolacji)
        self.segment_length_m = 5.0             # odstęp punktów trasy po interpolacji
        self.stabilization_time_s = 2.0         # maksymalny czas oczekiwania na stabilizację
        self.orientation_stable_threshold = 0.2 # rad (ok. 0.2) — próg stabilnej orientacji
        self.min_detection_alt = 5.0            # minimalna wysokość do sprawdzania detekcji
        self.max_detection_alt = 50.0           # maksymalna wysokość do sprawdzania detekcji

        # Interpolacja trasy co 5 m
        self.route_spacing_m = 5.0
        self.route_points = []

        # Bieżący waypoint (do przywracania po sprawdzeniu)
        self.current_waypoint = None

        self.last_stop_time = time.time()
        self.last_stop_position = Point()  # zainicjalizowane na (0,0,0)

        self.get_logger().info("Node initialized. Starting FSM...")

        # Wygeneruj trasę z interpolacją
        self.generate_interpolated_route()
        self.get_logger().info(f"Po interpolacji: {len(self.route_points)} punktów trasy co ~{self.route_spacing_m} m")
        
        # Pola trybu alarmowego
        self.alert_active = False
        self.alert_target_x = 0.0
        self.alert_target_y = 0.0
        self.alert_target_z = 0.0
        self.alert_time_str = ""
        self.prev_state = None

        # Parametry korekcji pozycji nad celem
        self.alignment_timeout_s = 3.0         # maksymalny czas korekcji po wykryciu
        self.alignment_tolerance_m = 0.5
        self.alignment_deadzone_m = 10       # Martwa strefa w centrum obrazu (w metrach) - brak korekty w tym obszarze

        # Ostatnio widziana pozycja człowieka (na wypadek utraty detekcji)
        self.last_seen_human_x = 0.0
        self.last_seen_human_y = 0.0
        self.last_seen_human_z = 0.0
        self.last_seen_time = 0.0
        # Okno stabilizacji w ALERT (5s) i flaga pojedynczej korekty
        self.alert_window_start = 0.0
        self.alert_window_detected = False
        self.alert_correction_done = False

        # Zapamiętana poprzednia pozycja (do cofnięcia po braku detekcji 5s)
        self.alert_prev_x = 0.0
        self.alert_prev_y = 0.0
        self.alert_prev_z = 0.0


    def human_position_cb(self, msg: Point):
        self.human_position = msg

    def get_global_human_position(self):
        """
        Transformuje pozycję człowieka z układu względnego (z tematu human_position_xy) 
        na globalne współrzędne uwzględniając aktualną pozycję drona
        """
        if (self.human_position.x == 0.0 and self.human_position.y == 0.0):
            return 0.0, 0.0  # Brak wykrycia człowieka
            
        if (self.current_position.pose.position.x == 0.0 and 
            self.current_position.pose.position.y == 0.0 and 
            self.current_position.pose.position.z == 0.0):
            return 0.0, 0.0  # Pozycja drona jeszcze niedostępna
        
        # Aktualna pozycja drona
        drone_x = self.current_position.pose.position.x
        drone_y = self.current_position.pose.position.y
        
        # Pozycja człowieka względem drona (z tematu)
        relative_x = self.human_position.x
        relative_y = self.human_position.y

        # Martwa strefa - jeśli cel jest blisko centrum, nie rób korekty
        if abs(relative_x) < self.alignment_deadzone_m and abs(relative_y) < self.alignment_deadzone_m:
            self.get_logger().info(f"[get_global_human_position] Cel w martwej strefie ({relative_x:.2f}, {relative_y:.2f}). Brak korekty.")
            return 0.0, 0.0
        
        # Prosta transformacja - dodaj pozycję drona do pozycji względnej
        # (zakładając, że human_position_xy jest już we właściwym układzie współrzędnych)
        # Poprawka: odwracamy znak dla relative_y, aby skorygować kierunek osi Y
        global_x = drone_x - relative_x
        global_y = drone_y - relative_y
        
        return global_x, global_y

    def position_cb(self, msg: PoseStamped):
        self.current_position = msg
        # Debug - wypisz pozycję pierwszych kilka razy
        if not hasattr(self, '_position_count'):
            self._position_count = 0
        
        if self._position_count < 5:
            self.get_logger().info(f"[DEBUG] Pozycja drona #{self._position_count + 1}: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")
            self._position_count += 1

    def load_geo_waypoints(self):
        """Wczytaj współrzędne geograficzne z pliku JSON i konwertuj na lokalne"""
        self.get_logger().info(f"Próbuję wczytać waypoints z: {self.waypoints_file_path}")
        try:
            with open(self.waypoints_file_path, 'r') as f:
                data = json.load(f)
            
            # Pobierz punkt odniesienia (origin)
            origin = data['origin']
            self.origin_lat = origin['latitude']
            self.origin_lon = origin['longitude'] 
            self.origin_alt = origin['altitude']
            
            # Pobierz waypoints i konwertuj na lokalne współrzędne
            waypoints = data['waypoints']
            self.local_waypoints = []
            
            for wp in waypoints:
                lat = wp['latitude']
                lon = wp['longitude']
                alt = wp['altitude']
                
                # Konwertuj współrzędne geograficzne na lokalne ENU używając funkcji z geo_utils
                x, y, z = geodetic_to_enu(lat, lon, alt, 
                                        self.origin_lat, self.origin_lon, self.origin_alt)
                
                self.local_waypoints.append((x, y, z))
                self.get_logger().info(f"Waypoint: GPS({lat:.6f}, {lon:.6f}, {alt:.1f}) -> Local({x:.2f}, {y:.2f}, {z:.2f})")
                
            self.get_logger().info(f"Wczytano {len(self.local_waypoints)} punktów trasy z pliku {self.waypoints_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas wczytywania waypoints: {str(e)}")
            # Fallback - użyj domyślnych waypoints lokalnych
            self.local_waypoints = [(0.0, 0.0, 12.0), (10.0, 10.0, 15.0), (0.0, 0.0, 12.0)]
            self.get_logger().warn("Używam domyślnych waypoints lokalnych")

    # Usunięto zależność od stabilności kamery

    def detection_cb(self, msg: String):
        """Odbiera JSON z detektora i aktualizuje liczbę wykrytych osób."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('detected'):
                detections = data.get('detections', [])
                if isinstance(detections, list):
                    self.last_detection_count = len(detections)
                else:
                    self.last_detection_count = 1
            else:
                self.last_detection_count = 0
        except Exception:
            self.last_detection_count = 0

    def generate_interpolated_route(self):
        """Interpoluje trasę tak, aby kolejne punkty były oddalone o ok. route_spacing_m w poziomie."""
        self.route_points = []
        if not self.local_waypoints:
            return

        self.route_points.append(self.local_waypoints[0])
        for i in range(1, len(self.local_waypoints)):
            x0, y0, z0 = self.local_waypoints[i - 1]
            x1, y1, z1 = self.local_waypoints[i]
            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0
            horiz = math.sqrt(dx*dx + dy*dy)
            if horiz < 1e-6:
                # Punkty praktycznie w tym samym miejscu
                self.route_points.append((x1, y1, z1))
                continue
            n = max(1, int(math.ceil(horiz / self.route_spacing_m)))
            for k in range(1, n + 1):
                t = k / n
                self.route_points.append((x0 + dx * t, y0 + dy * t, z0 + dz * t))

    def state_cb(self, msg):
        self.state_msg = msg

    def _publish_setpoint(self):
        self.local_pos_pub.publish(self.target_pose)

    def start_publishing(self):
        if self.publish_timer is None:
            self.publish_timer = self.create_timer(1.0 / self.rate_hz, self._publish_setpoint)
            self.get_logger().info(f"Publikuję setpointy z częstotliwością {self.rate_hz} Hz")

    def stop_publishing(self):
        if self.publish_timer is not None:
            self.publish_timer.cancel()
            self.publish_timer = None
            self.get_logger().info("Zatrzymano publikację setpointów")

    def state_init(self):
        self.get_logger().info("[INIT] Czekam na połączenie z FCU...")
        while not self.state_msg.connected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("[INIT] Połączono. Start publikacji setpointów.")
        self.start_publishing()

        # 2 sekundy stabilizacji
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.current_state = "OFFBOARD"

    def state_offboard(self):
        self.get_logger().info("[OFFBOARD] Ustawiam tryb OFFBOARD i uzbrajam...")
        self.set_mode_client.wait_for_service()
        self.arming_client.wait_for_service()

        mode_req = SetMode.Request()
        mode_req.custom_mode = 'OFFBOARD'
        arm_req = CommandBool.Request()
        arm_req.value = True

        self.set_mode_client.call_async(mode_req)
        self.arming_client.call_async(arm_req)

        # Czekaj aż dron faktycznie przełączy się w OFFBOARD i się uzbroi
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state_msg.mode == "OFFBOARD" and self.state_msg.armed:
                self.get_logger().info("[OFFBOARD] Dron uzbrojony i w trybie OFFBOARD.")
                self.current_state = "CONTROL"
                return

        self.get_logger().warn("[OFFBOARD] Nie udało się uzbroić lub ustawić trybu OFFBOARD.")
        self.current_state = "INIT"

    # --- POMOCNICZE: konwersja kwaternionu na RPY ---
    def quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def is_orientation_stable(self, threshold=None):
        if threshold is None:
            threshold = self.orientation_stable_threshold
        q = self.current_position.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q)
        return (abs(roll) < threshold and abs(pitch) < threshold and abs(yaw) < threshold)

    def is_tilt_stable(self, threshold=None):
        """Stabilność przechyłu (roll/pitch) bez warunku na yaw."""
        if threshold is None:
            threshold = self.orientation_stable_threshold
        q = self.current_position.pose.orientation
        roll, pitch, _ = self.quaternion_to_euler(q)
        return (abs(roll) < threshold and abs(pitch) < threshold)

    def distance_from_last_stop(self):
        cx = self.current_position.pose.position.x
        cy = self.current_position.pose.position.y
        cz = self.current_position.pose.position.z
        dx = cx - self.last_stop_position.x
        dy = cy - self.last_stop_position.y
        dz = cz - self.last_stop_position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def should_stop_for_check(self):
        # Postój jeśli minął czas lub pokonano dystans
        time_ok = (time.time() - self.last_stop_time) >= self.segment_stop_interval_s
        dist_ok = self.distance_from_last_stop() >= self.segment_length_m
        return time_ok or dist_ok

    def stabilize_and_check(self):
        """Zatrzymaj, poczekaj na stabilizację i sprawdź detekcję człowieka."""
        # Zapamiętaj miejsce postoju
        self.last_stop_time = time.time()
        self.last_stop_position = Point(
            x=self.current_position.pose.position.x,
            y=self.current_position.pose.position.y,
            z=self.current_position.pose.position.z
        )

        # Utrzymuj bieżącą pozycję jako setpoint
        self.target_pose.pose.position.x = self.current_position.pose.position.x
        self.target_pose.pose.position.y = self.current_position.pose.position.y
        self.target_pose.pose.position.z = self.current_position.pose.position.z

        # Oczekiwanie na stabilizację orientacji
        start = time.time()
        stable = False
        while (time.time() - start) < self.stabilization_time_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.is_orientation_stable():
                stable = True
                break

        # Jeżeli dron nie jest stabilny (tilt) — nie sprawdzaj detekcji
        if not self.is_tilt_stable():
            self.get_logger().info("[CHECK] Orientacja niestabilna — pomijam detekcję, wracam do trasy.")
            if self.current_waypoint is not None:
                wx, wy, wz = self.current_waypoint
                self.target_pose.pose.position.x = wx
                self.target_pose.pose.position.y = wy
                self.target_pose.pose.position.z = wz
            return False

        # Uwzględnij wysokość i pozycję z /mavros/local_position/pose przy detekcji
        current_z = self.current_position.pose.position.z
        if self.min_detection_alt <= current_z <= self.max_detection_alt:
            # Jeśli mamy sygnał pozycji człowieka względny do drona
            if self.human_position.x != 0.0 or self.human_position.y != 0.0:
                gx, gy = self.get_global_human_position()
                if gx != 0.0 or gy != 0.0:
                    # Cel: pozycja człowieka, wysokość bieżąca
                    self.target_pose.pose.position.x = gx
                    self.target_pose.pose.position.y = gy
                    self.target_pose.pose.position.z = current_z
                    # Zapisz ostatnio widzianą pozycję człowieka
                    self.last_seen_human_x = gx
                    self.last_seen_human_y = gy
                    self.last_seen_human_z = current_z
                    self.last_seen_time = time.time()
                    # Brak precyzyjnej korekty pozycji — tylko zatrzymanie nad wykryciem
                    # Zainicjuj tryb alarmowy i wyślij wiadomość ALARM
                    self.alert_active = True
                    self.alert_target_x = gx
                    self.alert_target_y = gy
                    self.alert_target_z = current_z
                    # Czas i liczba osób
                    self.alert_time_str = time.strftime('%Y-%m-%d %H:%M:%S')
                    count = int(self.last_detection_count)
                    alarm_msg = (
                        f"ALARM: wykryty intruz; pozycja: x={gx:.2f}, y={gy:.2f}, z={current_z:.2f}; "
                        f"czas: {self.alert_time_str}; ilość osób: {count}"
                    )
                    self.alarm_pub.publish(String(data=alarm_msg))
                    self.get_logger().info(
                        f"[CHECK] Stabilizacja OK. Wykryto człowieka → lecę do ({gx:.2f}, {gy:.2f}, {current_z:.2f})"
                    )
                    return True
        self.get_logger().info("[CHECK] Brak detekcji po stabilizacji. Wracam do trasy.")
        return False

    

    def state_control(self):
        self.get_logger().info("[CONTROL] Start misji - lot według współrzędnych geograficznych.")
        
        # Zakomentowane - stare waypoints
        # waypoints = [
        #     (0.0, 0.0, 12.0),
        #     #(20.0, 0.0, 20.0),
        #     #(20.0, 20.0, 20.0),
        #     #(0.0, 20.0, 20.0),
        #     #(0.0, 0.0, 20.0),
        # ] * 20  # dwa okrążenia

        # Używaj punktów po interpolacji trasy co ~5 m
        for i, (x, y, z) in enumerate(self.route_points):
            # Normalny lot po trasie (ustaw cel waypointu)
            self.current_waypoint = (x, y, z)
            self.target_pose.pose.position.x = x
            self.target_pose.pose.position.y = y
            self.target_pose.pose.position.z = z
            self.get_logger().info(f"[CONTROL] Lecę do waypoint {i+1}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

            # Czekanie na osiągnięcie punktu - sprawdzaj odległość do celu
            start_time = time.time()
            tolerance = 1.0
            reached_waypoint = False
            found_human = False
            
            while time.time() - start_time < 50.0 and rclpy.ok():
                # Jeżeli pozycja niedostępna — kontynuuj
                if (self.current_position.pose.position.x == 0.0 and 
                    self.current_position.pose.position.y == 0.0 and 
                    self.current_position.pose.position.z == 0.0):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                # Sprawdź dystans do waypointu
                current_x = self.current_position.pose.position.x
                current_y = self.current_position.pose.position.y
                current_z = self.current_position.pose.position.z
                distance = ((current_x - x)**2 + (current_y - y)**2 + (current_z - z)**2)**0.5

                if distance < tolerance:
                    self.get_logger().info(f"[CONTROL] Osiągnięto waypoint {i+1} (odległość: {distance:.2f}m)")
                    reached_waypoint = True
                    # Po osiągnięciu każdego punktu co ~5 m: zatrzymaj, ustabilizuj i sprawdź detekcję
                    detected = self.stabilize_and_check()
                    if detected:
                        found_human = True
                    # Po osiągnięciu waypointu — punkt kontrolny również
                    # Po osiągnięciu waypointu — punkt kontrolny również
                    self.last_stop_time = time.time()
                    self.last_stop_position = Point(x=current_x, y=current_y, z=current_z)
                    break
                
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not reached_waypoint:
                self.get_logger().warn(f"[CONTROL] Timeout dla waypoint {i+1}! Przechodzę do następnego.")
            if found_human:
                # Po wykryciu człowieka, setpoint został ustawiony w stabilize_and_check.
                # Kończymy patrol i przechodzimy dalej (np. do lądowania po podejściu).
                break

        if self.alert_active:
            self.get_logger().info("[CONTROL] Wykryto intruza — przejście do ALERT i utrzymanie pozycji.")
            self.prev_state = "CONTROL"
            # zapamiętaj poprzedni setpoint, aby móc się cofnąć
            self.alert_prev_x = self.target_pose.pose.position.x
            self.alert_prev_y = self.target_pose.pose.position.y
            self.alert_prev_z = self.target_pose.pose.position.z
            # okno 5s do korekty
            self.alert_window_start = time.time()
            self.alert_window_detected = True
            self.alert_correction_done = False
            self.current_state = "ALERT"
        else:
            self.get_logger().info("[CONTROL] Misja zakończona. Przechodzę do LAND.")
            self.current_state = "LAND"

    def state_alert(self):
        """Utrzymuj pozycję nad intruzem; korekta co 5s jeśli jest detekcja. Brak detekcji 5s -> powrót do poprzedniej pozycji."""
        self.get_logger().info("[ALERT] Intruz wykryty — utrzymuję pozycję i publikuję ALARM.")
        # start od ostatnio wykrytego celu
        self.target_pose.pose.position.x = self.alert_target_x
        self.target_pose.pose.position.y = self.alert_target_y
        self.target_pose.pose.position.z = self.alert_target_z

        while rclpy.ok() and self.current_state == "ALERT":
            now = time.time()
            gx, gy = self.get_global_human_position()

            # detekcja w bieżącym oknie
            if gx != 0.0 or gy != 0.0:
                self.last_seen_human_x = gx
                self.last_seen_human_y = gy
                self.last_seen_human_z = self.target_pose.pose.position.z
                self.last_seen_time = now
                self.alert_window_detected = True

            # co 5 sekund: korekta jeśli była detekcja; inaczej powrót
            if (now - self.alert_window_start) >= 5.0:
                if self.alert_window_detected and self.is_tilt_stable():
                    # Użyj najnowszej pozycji, jeśli jest dostępna, inaczej ostatnio znanej
                    use_x = gx if (gx != 0.0 or gy != 0.0) else self.last_seen_human_x
                    use_y = gy if (gx != 0.0 or gy != 0.0) else self.last_seen_human_y
                    
                    # Aktualizuj cel drona o nową pozycję człowieka
                    if use_x != 0.0 or use_y != 0.0:
                        self.alert_target_x = use_x
                        self.alert_target_y = use_y
                        self.get_logger().info(f"[ALERT] Korekta pozycji co 5s: x={use_x:.2f}, y={use_y:.2f}")
                    
                    # reset okna
                    self.alert_window_start = now
                    self.alert_window_detected = False
                else:
                    # brak detekcji przez 5s -> cofnięcie do poprzedniej pozycji i wyjście z ALERT
                    self.get_logger().info("[ALERT] Brak detekcji przez 5s — wracam do poprzedniej pozycji i do CONTROL.")
                    self.target_pose.pose.position.x = self.alert_prev_x
                    self.target_pose.pose.position.y = self.alert_prev_y
                    self.target_pose.pose.position.z = self.alert_prev_z
                    self.alert_active = False
                    self.current_state = self.prev_state if self.prev_state else "CONTROL"
                    break
            
            # Ustawiaj cel na bieżąco, aby dron podążał za celem
            self.target_pose.pose.position.x = self.alert_target_x
            self.target_pose.pose.position.y = self.alert_target_y

            rclpy.spin_once(self, timeout_sec=0.1)

    def state_land(self):
        self.get_logger().info("[LAND] Lądowanie...")
        start_z = self.target_pose.pose.position.z
        start_time = time.time()
        duration = 5.0

        while time.time() - start_time < duration and rclpy.ok():
            elapsed = time.time() - start_time
            ratio = elapsed / duration
            self.target_pose.pose.position.z = start_z * (1 - ratio)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop_publishing()

        # Rozbrojenie
        disarm_req = CommandBool.Request()
        disarm_req.value = False
        self.arming_client.call_async(disarm_req)

        self.get_logger().info("[LAND] Wylądowano i rozbrojono. Misja zakończona.")
        self.current_state = "END"

    # ====== GŁÓWNA PĘTLA ======
    def run(self):
        while rclpy.ok() and self.current_state != "END":
            if self.current_state == "INIT":
                self.state_init()
            elif self.current_state == "OFFBOARD":
                self.state_offboard()
            elif self.current_state == "CONTROL":
                self.state_control()
            elif self.current_state == "LAND":
                self.state_land()
            elif self.current_state == "ALERT":
                self.state_alert()
            else:
                self.get_logger().error(f"Nieznany stan: {self.current_state}")
                break

        self.get_logger().info("FSM zakończony. Node kończy działanie.")


def main():
    rclpy.init()
    node = OffboardFSM()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
