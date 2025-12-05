import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import subprocess
import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# ============================================================
#  CONFIGURACIÓN: ¿QUÉ MOTORES ESTÁS USANDO?
# ============================================================
USE_XL430 = False

# ============================================================
#  LONGITUDES DEL MANIPULADOR (EN METROS)
# ============================================================
L1 = 44.0  / 1000.0
L2 = 107.5 / 1000.0
L3 = 107.5 / 1000.0
L4 = 75.3  / 1000.0

# Alcances aproximados derivados de las longitudes
PLANAR_REACH_MAX = L2 + L3 + L4        # radio máximo en el plano XY ≈ 0.2903 m
PLANAR_REACH_MIN = 0.04               # ~4 cm desde el eje de la base
Z_MAX = L1 + L2 + L3 + L4             # altura máxima teórica ≈ 0.3343 m
Z_MIN = 0.0                           # asumimos que no bajamos por debajo de la base


# ------------------------------------------------------------
# Direcciones de registro y parámetros según el tipo de motor
# ------------------------------------------------------------
if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_MOVING_SPEED     = 112  # Profile Velocity
    ADDR_TORQUE_LIMIT     = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023  # Velocidad máxima para XL430
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24
    ADDR_GOAL_POSITION    = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT     = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023  # Velocidad máxima para otros modelos

# ============================================================
#  FUNCIONES AUXILIARES
# ============================================================

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

# ============================================================
#  MODELO DEL ROBOT (DH) CON TUS LONGITUDES
# ============================================================

def build_pincher_robot():
    """
    Modelo DH del Pincher X100 (tus longitudes)
    """
    links = [
        rtb.RevoluteDH(d=L1, a=0.0,  alpha=np.pi/2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0,     offset=np.pi/2),
        rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0,     offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4, alpha=0.0,     offset=0.0),
    ]

    robot = rtb.DHRobot(links, name="Pincher")

    # Herramienta: T_tool = trotz(-pi/2)*trotx(-pi/2)
    T_tool = SE3.Rz(-np.pi/2) * SE3.Rx(-np.pi/2)
    robot.tool = T_tool

    return robot


# ============================================================
#  NODO ROS2 CON PUBLICACIÓN PARA RViz
# ============================================================

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')
        
        # Modelo cinemático
        self.robot_model = build_pincher_robot()

        # >>> NUEVO: límites geométricos del workspace (en m)
        self.planar_reach_max = PLANAR_REACH_MAX
        self.planar_reach_min = PLANAR_REACH_MIN
        self.z_min = Z_MIN
        self.z_max = Z_MAX

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 800)

        # Obtener parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        # Inicializar comunicación
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        self.packet = PacketHandler(PROTOCOL_VERSION)
        
        # Estado de emergencia
        self.emergency_stop_activated = False
        
        # Configuración inicial de los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
        
        # Publicador para Joint States (para RViz)
        from sensor_msgs.msg import JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer para publicar joint states
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        # Posiciones actuales de las articulaciones (en radianes)
        self.current_joint_positions = [0.0] * 5  # Para 5 articulaciones
        
        # Mapeo de IDs de motor a nombres de articulaciones
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']

        # Signo de cada motor (ajusta según montaje real)
        self.joint_sign = {
            1: 1,
            2: 1,
            3: 1,
            4: 1,
            5: 1,
        }

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """Configuración inicial de todos los motores"""
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                # Configurar velocidad
                self.update_speed_single_motor(dxl_id, moving_speed)
                
                # Mover a posición inicial
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
                
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def dxl_to_radians(self, dxl_value):
        """Convierte valor Dynamixel (0-1023) a radianes (~-2.618 a 2.618)"""
        center = 512          # punto medio
        half_range_rad = 2.618  # ±150° en radianes
        return (dxl_value - center) * (half_range_rad/center)

    def radians_to_dxl(self, radians):
        """Convierte radianes a valor Dynamixel (0-1023)"""
        center = 512
        half_range_rad = 2.618
        return int((radians * (center/half_range_rad)) + center)
        
    def radians_to_dxl_real(self, radians):
        """Convierte radianes a valor Dynamixel (0-1023) (para XL430 si lo usas)"""
        return int(radians * (2048.0 / 2.618) + 2048) / 4

    def publish_joint_states(self):
        """Publica el estado de las articulaciones para RViz"""
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        self.joint_state_pub.publish(joint_state)

    def move_motor(self, motor_id, position):
        """Mueve un motor a la posición especificada solo si no hay emergencia"""
        if self.emergency_stop_activated:
            self.get_logger().warning(f'No se puede mover motor {motor_id}: Parada de emergencia activada')
            return False
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                return True
            else:
                self.get_logger().error(f'Error moviendo motor {motor_id}: {error}')
                return False
        except Exception as e:
            self.get_logger().error(f'Excepción moviendo motor {motor_id}: {str(e)}')
            return False

    # ============================================================
    #  CINEMÁTICA INVERSA / DIRECTA
    # ============================================================

    def move_to_joint_angles(self, q_rad):
        """
        Mueve el robot a una configuración de articulaciones dada
        
        Args:
            q_rad: Lista de 4 ángulos en radianes [waist, shoulder, elbow, wrist]
        
        Returns:
            True si todos los motores se movieron correctamente, False en caso contrario
        """
        if len(q_rad) != 4:
            self.get_logger().error(f'Se esperaban 4 ángulos, se recibieron {len(q_rad)}')
            return False
        
        # rango real que estamos usando en la conversión Dynamixel <-> rad
        q_min = -2.618  # ≈ -150°
        q_max =  2.618  # ≈ +150°
        
        success = True
        for i, motor_id in enumerate(self.dxl_ids[:4]):  # Solo primeras 4 articulaciones
            sign = self.joint_sign.get(motor_id, 1)
            motor_angle = q_rad[i] * sign

            # Recortar a rango permitido por el mapeo a Dynamixel
            motor_angle_clamped = float(np.clip(motor_angle, q_min, q_max))

            if motor_angle != motor_angle_clamped:
                self.get_logger().warning(
                    f'Joint {i+1} saturado: {np.degrees(motor_angle):.1f}° '
                    f'-> {np.degrees(motor_angle_clamped):.1f}°'
                )

            goal_dxl = self.radians_to_dxl(motor_angle_clamped)
            
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success


    def move_to_xyz(self, x, y, z, orientation='down'):
        """
        Mueve el end-effector a una posición XYZ específica usando cinemática inversa.
        x, y, z en metros.
        Solo se impone la posición (no la orientación completa) para evitar
        que el solver falle por restricciones innecesarias.
        """
        try:
            # 1) Comprobar workspace geométrico según tus longitudes
            r = np.hypot(x, y)

            if r < self.planar_reach_min or r > self.planar_reach_max + 0.01:
                self.get_logger().error(
                    f'Objetivo fuera del alcance radial: r={r:.3f} m '
                    f'[{self.planar_reach_min:.3f}, {self.planar_reach_max:.3f}]'
                )
                return False

            if z < self.z_min - 0.01 or z > self.z_max + 0.01:
                self.get_logger().error(
                    f'Objetivo fuera del alcance en Z: z={z:.3f} m '
                    f'[{self.z_min:.3f}, {self.z_max:.3f}]'
                )
                return False

            # 2) Construir pose objetivo: solo nos interesa la posición, la R da igual
            #    (la orientación "down" queda sugerida, pero no se fuerza en la IK)
            if orientation == 'down':
                T_target = SE3(x, y, z) * SE3.Rx(np.pi)
            else:
                T_target = SE3(x, y, z)
            
            q_current = np.array(self.current_joint_positions[:4])

            self.get_logger().info(
                f'Intentando IK para ({x:.3f}, {y:.3f}, {z:.3f})  '
                f'con q0 = {np.degrees(q_current)}°'
            )
            
            # 3) Diferentes semillas, pero IMPORTANTÍSIMO: máscara de solo posición
            seeds = [
                q_current,
                np.array([0, 0, 0, 0]),
                np.array([0, 0.5, -0.5, 0]),
                np.array([0, 1.0, -1.0, 0]),
            ]
            
            best_sol = None
            best_error = float('inf')
            
            for seed in seeds:
                sol = self.robot_model.ikine_LM(
                    T_target,
                    q0=seed,
                    ilimit=1000,
                    slimit=100,
                    mask=[1, 1, 1, 0, 0, 0]  # SOLO posición XYZ
                )
                
                if sol.success:
                    T_check = self.robot_model.fkine(sol.q)
                    error = np.linalg.norm(T_check.t - T_target.t)
                    
                    if error < best_error:
                        best_error = error
                        best_sol = sol
                    
                    if error < 0.003:  # 3 mm
                        break
            
            if best_sol is None:
                self.get_logger().error(
                    f'No se encontró solución IK para ({x:.3f}, {y:.3f}, {z:.3f})'
                )
                return False

            q_solution = best_sol.q

            self.get_logger().info(
                f'✓ Solución IK encontrada: {np.degrees(q_solution)}°  '
                f'error ≈ {best_error*1000:.1f} mm'
            )

            # 4) Mover con saturación en move_to_joint_angles
            return self.move_to_joint_angles(q_solution)
                
        except Exception as e:
            self.get_logger().error(f'Error en cinemática inversa: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def get_current_xyz(self):
        """Devuelve (x, y, z) actual del end-effector en metros."""
        try:
            q_current = np.array(self.current_joint_positions[:4])
            T = self.robot_model.fkine(q_current)
            return (T.t[0], T.t[1], T.t[2])
        except Exception as e:
            self.get_logger().error(f'Error calculando cinemática directa: {str(e)}')
            return (0, 0, 0)

    # ============================================================
    #  RESTO DE FUNCIONES DE CONTROL
    # ============================================================

    def update_speed_single_motor(self, motor_id, speed):
        """Actualiza la velocidad de un motor individual"""
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        """Actualiza la velocidad de movimiento en todos los motores"""
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
            
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1
        
        if success_count == len(self.dxl_ids):
            self.get_logger().info(f'Velocidad actualizada a {speed} en todos los motores')
        else:
            self.get_logger().warning(
                f'Velocidad actualizada a {speed} en {success_count}/{len(self.dxl_ids)} motores'
            )

    def home_all_motors(self):
        """Mueve todos los motores a la posición home (DEFAULT_GOAL)"""
        if self.emergency_stop_activated:
            self.reactivate_torque()
            
        for motor_id in reversed(self.dxl_ids):
            self.move_motor(motor_id, DEFAULT_GOAL)
        self.get_logger().info('Todos los motores movidos a posición HOME')

    def home_all_motors_sec(self):
        """Home con pausas de 2 s entre motores (opcional)"""
        if self.emergency_stop_activated:
            self.reactivate_torque()
            
        for motor_id in reversed(self.dxl_ids):
            self.move_motor(motor_id, 512)
            time.sleep(2.0)
        self.get_logger().info('Todos los motores movidos a posición HOME')

    def emergency_stop(self):
        """Parada de emergencia - desactiva el torque de todos los motores"""
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id} (EMERGENCY STOP)')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        """Reactivar el torque después de una parada de emergencia"""
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')

    def close(self):
        """Apaga el torque y cierra el puerto"""
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()

# ============================================================
#  INTERFAZ GRÁFICA CON PESTAÑAS (SLIDERS, VALORES, XYZ, RViz)
# ============================================================

class PincherGUI:
    def __init__(self, controller):
        self.controller = controller
        self.window = tk.Tk()
        self.window.title("Control Pincher - Interfaz Completa con IK")
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Control de actualización
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.last_speed_update = 0
        self.update_interval = 0.05  # 50 ms
        
        # Proceso de RViz
        self.rviz_process = None
        
        # Notebook (pestañas)
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        self.tab1 = ttk.Frame(self.notebook)
        self.tab2 = ttk.Frame(self.notebook)
        self.tab3 = ttk.Frame(self.notebook)  # Control XYZ
        self.tab4 = ttk.Frame(self.notebook)  # RViz
        
        self.notebook.add(self.tab1, text='Control por Sliders')
        self.notebook.add(self.tab2, text='Control por Valores')
        self.notebook.add(self.tab3, text='Control XYZ (IK)')
        self.notebook.add(self.tab4, text='Visualización RViz')
        
        # Configurar pestañas
        self.setup_tab1()
        self.setup_tab2()
        self.setup_tab3_xyz()
        self.setup_tab4_rviz()
        
        # Botones comunes abajo
        self.setup_common_buttons()
        
        # Timer XYZ
        self.update_xyz_display_timer()

    # ---------------- PESTAÑA 1: SLIDERS ----------------

    def setup_tab1(self):
        title_label = tk.Label(self.tab1, text="Control por Sliders - TIEMPO REAL", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        motors_frame = tk.Frame(self.tab1)
        motors_frame.pack(fill='x', padx=20, pady=10)
        
        self.sliders = {}
        self.labels = {}
        
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=5)
            
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            slider = tk.Scale(motor_frame, from_=0, to=1023, orient=tk.HORIZONTAL, 
                             length=400, showvalue=True, resolution=1,
                             command=lambda value, mid=motor_id: self.on_motor_slider_change(mid))
            slider.set(DEFAULT_GOAL)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            label = tk.Label(motor_frame, text=f'Pos: {DEFAULT_GOAL}', 
                            font=("Arial", 9), width=10)
            label.pack(side='right', padx=5)
            
            self.sliders[motor_id] = slider
            self.labels[motor_id] = label
        
        ttk.Separator(self.tab1, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        speed_frame = tk.Frame(self.tab1)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab1 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab1.set(100)
        self.speed_slider_tab1.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab1 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab1.pack(side='right', padx=10)
        
        speed_note = tk.Label(speed_frame, text="Nota: Velocidad 0 = No movimiento", 
                             font=("Arial", 8), fg="gray")
        speed_note.pack(anchor='w')

    # ---------------- PESTAÑA 2: VALORES ----------------

    def setup_tab2(self):
        title_label = tk.Label(self.tab2, text="Control por Valores Manuales", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        speed_frame = tk.Frame(self.tab2)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab2 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab2.set(100)
        self.speed_slider_tab2.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab2 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab2.pack(side='right', padx=10)
        
        speed_note = tk.Label(speed_frame, text="Nota: Velocidad 0 = No movimiento", 
                             font=("Arial", 8), fg="gray")
        speed_note.pack(anchor='w')
        
        ttk.Separator(self.tab2, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        motors_frame = tk.Frame(self.tab2)
        motors_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        self.entries = {}
        self.entry_labels = {}
        
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=8)
            
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            entry_label = tk.Label(motor_frame, text="Valor (0-1023):", 
                                  font=("Arial", 9))
            entry_label.pack(side='left', padx=5)
            
            entry = tk.Entry(motor_frame, width=8, font=("Arial", 10))
            entry.insert(0, str(DEFAULT_GOAL))
            entry.pack(side='left', padx=5)
            
            move_btn = tk.Button(motor_frame, text="Mover Motor", 
                                font=("Arial", 9),
                                command=lambda mid=motor_id: self.move_single_motor_from_entry(mid))
            move_btn.pack(side='left', padx=5)
            
            status_label = tk.Label(motor_frame, text="Listo", 
                                   font=("Arial", 9), fg="green", width=10)
            status_label.pack(side='right', padx=5)
            
            self.entries[motor_id] = entry
            self.entry_labels[motor_id] = status_label
            
            entry.bind('<Return>', lambda event, mid=motor_id: self.move_single_motor_from_entry(mid))
        
        move_all_frame = tk.Frame(motors_frame)
        move_all_frame.pack(fill='x', pady=15)
        
        move_all_btn = tk.Button(move_all_frame, text="MOVER TODOS LOS MOTORES", 
                                font=("Arial", 10, "bold"), bg="#4CAF50", fg="white",
                                command=self.move_all_motors_from_entries)
        move_all_btn.pack(pady=10)

    # ---------------- PESTAÑA 3: XYZ (IK) ----------------

    def setup_tab3_xyz(self):
        title_label = tk.Label(self.tab3, text="Control XYZ - Cinemática Inversa", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        info_frame = tk.Frame(self.tab3)
        info_frame.pack(fill='x', padx=20, pady=10)
        
        info_text = """
Controla el robot especificando la posición XYZ del end-effector.
El sistema calcula automáticamente los ángulos de las articulaciones.
        """
        info_label = tk.Label(info_frame, text=info_text, justify=tk.LEFT, 
                             font=("Arial", 10), bg="#e3f2fd", relief="solid", padx=10, pady=10)
        info_label.pack(fill='x')
        
        current_frame = tk.LabelFrame(self.tab3, text="Posición Actual del End-Effector", 
                                      font=("Arial", 11, "bold"))
        current_frame.pack(fill='x', padx=20, pady=10)
        
        current_pos_frame = tk.Frame(current_frame)
        current_pos_frame.pack(fill='x', padx=10, pady=10)
        
        self.current_x_label = tk.Label(current_pos_frame, text="X: 0.000 m", 
                                       font=("Arial", 10, "bold"), width=15)
        self.current_x_label.pack(side='left', padx=10)
        
        self.current_y_label = tk.Label(current_pos_frame, text="Y: 0.000 m", 
                                       font=("Arial", 10, "bold"), width=15)
        self.current_y_label.pack(side='left', padx=10)
        
        self.current_z_label = tk.Label(current_pos_frame, text="Z: 0.000 m", 
                                       font=("Arial", 10, "bold"), width=15)
        self.current_z_label.pack(side='left', padx=10)
        
        ttk.Separator(self.tab3, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        target_frame = tk.LabelFrame(self.tab3, text="Posición Objetivo", 
                                     font=("Arial", 11, "bold"))
        target_frame.pack(fill='x', padx=20, pady=10)
        
        xyz_entries_frame = tk.Frame(target_frame)
        xyz_entries_frame.pack(fill='x', padx=10, pady=10)
        
        x_frame = tk.Frame(xyz_entries_frame)
        x_frame.pack(side='left', padx=20)
        tk.Label(x_frame, text="X (m):", font=("Arial", 10)).pack()
        self.x_entry = tk.Entry(x_frame, width=10, font=("Arial", 11))
        self.x_entry.insert(0, "0.200")
        self.x_entry.pack()
        
        y_frame = tk.Frame(xyz_entries_frame)
        y_frame.pack(side='left', padx=20)
        tk.Label(y_frame, text="Y (m):", font=("Arial", 10)).pack()
        self.y_entry = tk.Entry(y_frame, width=10, font=("Arial", 11))
        self.y_entry.insert(0, "0.000")
        self.y_entry.pack()
        
        z_frame = tk.Frame(xyz_entries_frame)
        z_frame.pack(side='left', padx=20)
        tk.Label(z_frame, text="Z (m):", font=("Arial", 10)).pack()
        self.z_entry = tk.Entry(z_frame, width=10, font=("Arial", 11))
        self.z_entry.insert(0, "0.100")
        self.z_entry.pack()
        
        move_xyz_btn = tk.Button(target_frame, text="MOVER A POSICIÓN XYZ", 
                                font=("Arial", 12, "bold"), bg="#4CAF50", fg="white",
                                command=self.move_to_xyz_target, height=2)
        move_xyz_btn.pack(pady=15)
        
        ttk.Separator(self.tab3, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        presets_frame = tk.LabelFrame(self.tab3, text="Posiciones Predefinidas", 
                                      font=("Arial", 11, "bold"))
        presets_frame.pack(fill='x', padx=20, pady=10)
        
        presets_buttons_frame = tk.Frame(presets_frame)
        presets_buttons_frame.pack(fill='x', padx=10, pady=10)
        
        presets = [
            ("Home XYZ",       0.20,  0.00, 0.10),
            ("Alto Centro",    0.15,  0.00, 0.20),
            ("Frente Derecha", 0.18,  0.08, 0.05),
            ("Frente Izquierda", 0.18, -0.08, 0.05),
        ]
        
        for name, x, y, z in presets:
            btn = tk.Button(
                presets_buttons_frame,
                text=f"{name}\n({x:.2f}, {y:.2f}, {z:.2f})", 
                font=("Arial", 9), width=15, height=3,
                command=lambda px=x, py=y, pz=z: self.move_to_preset_xyz(px, py, pz)
            )
            btn.pack(side='left', padx=5)
        
        self.ik_status_label = tk.Label(self.tab3, text="Listo para mover", 
                                       font=("Arial", 10), fg="green")
        self.ik_status_label.pack(pady=10)

    # ---------------- PESTAÑA 4: RViz ----------------

    def setup_tab4_rviz(self):
        title_label = tk.Label(self.tab4, text="Visualización en RViz - PhantomX Pincher X100", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        info_frame = tk.Frame(self.tab4)
        info_frame.pack(fill='x', padx=20, pady=10)
        
        info_text = """
Esta pestaña permite visualizar el robot PhantomX Pincher X100 en RViz.

Características:
• Modelo 3D del PhantomX Pincher X100
• Sincronización en tiempo real con el robot físico
• Visualización de todas las articulaciones
• Feedback visual de los movimientos
        """
        info_label = tk.Label(info_frame, text=info_text, justify=tk.LEFT, 
                             font=("Arial", 10), bg="#f0f0f0", relief="solid", padx=10, pady=10)
        info_label.pack(fill='x')
        
        controls_frame = tk.Frame(self.tab4)
        controls_frame.pack(fill='x', padx=20, pady=20)
        
        self.rviz_btn = tk.Button(controls_frame, text="LANZAR RViz", 
                                 font=("Arial", 12, "bold"), bg="#2196F3", fg="white",
                                 command=self.launch_rviz, height=2)
        self.rviz_btn.pack(fill='x', pady=5)
        
        self.stop_rviz_btn = tk.Button(controls_frame, text="DETENER RViz", 
                                      font=("Arial", 10), bg="#f44336", fg="white",
                                      command=self.stop_rviz, state=tk.DISABLED)
        self.stop_rviz_btn.pack(fill='x', pady=5)
        
        self.rviz_status_label = tk.Label(controls_frame, text="RViz no iniciado", 
                                         font=("Arial", 10), fg="red")
        self.rviz_status_label.pack(pady=10)
        
        joints_frame = tk.Frame(self.tab4)
        joints_frame.pack(fill='x', padx=20, pady=10)
        
        joints_label = tk.Label(joints_frame, text="Posiciones de Articulaciones (radianes):", 
                               font=("Arial", 10, "bold"))
        joints_label.pack(anchor='w')
        
        self.joint_labels = {}
        for i, joint_name in enumerate(self.controller.joint_names):
            joint_frame = tk.Frame(joints_frame)
            joint_frame.pack(fill='x', pady=2)
            
            label = tk.Label(joint_frame, text=f"{joint_name}:", 
                            font=("Arial", 9), width=10, anchor='w')
            label.pack(side='left')
            
            value_label = tk.Label(joint_frame, text="0.000", 
                                  font=("Arial", 9), width=10)
            value_label.pack(side='left')
            
            self.joint_labels[joint_name] = value_label
        
        self.update_joints_timer()

    # ---------------- BOTONES COMUNES ----------------

    def setup_common_buttons(self):
        common_buttons_frame = tk.Frame(self.window)
        common_buttons_frame.pack(fill='x', padx=20, pady=10)
        
        home_btn = tk.Button(common_buttons_frame, text="HOME", 
                            font=("Arial", 10, "bold"), bg="#2196F3", fg="white",
                            command=self.home_all)
        home_btn.pack(side='left', padx=10)
        
        emergency_btn = tk.Button(common_buttons_frame, text="PARADA DE EMERGENCIA", 
                                 font=("Arial", 10, "bold"), bg="#f44336", fg="white",
                                 command=self.emergency_stop)
        emergency_btn.pack(side='right', padx=10)
        
        self.status_label = tk.Label(common_buttons_frame, text="Sistema Listo", 
                                    font=("Arial", 9), fg="green")
        self.status_label.pack(side='bottom', pady=5)

    # ---------------- FUNCIONES XYZ GUI ----------------

    def move_to_xyz_target(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            
            # Verificar límites sigiendo las longitudes reales
            r = (x**2 + y**2) ** 0.5

            if not (self.controller.planar_reach_min <= r <= self.controller.planar_reach_max + 0.01):
                msg = (
                    "El punto está fuera del alcance radial del brazo.\n\n"
                    f"Radio r = {r:.3f} m\n"
                    f"Rango permitido ≈ [{self.controller.planar_reach_min:.3f}, "
                    f"{self.controller.planar_reach_max:.3f}] m"
                )
                messagebox.showerror("Error", msg)
                return

            if not (self.controller.z_min - 0.01 <= z <= self.controller.z_max + 0.01):
                msg = (
                    "El punto está fuera del alcance en Z.\n\n"
                    f"z = {z:.3f} m\n"
                    f"Rango permitido ≈ [{self.controller.z_min:.3f}, "
                    f"{self.controller.z_max:.3f}] m"
                )
                messagebox.showerror("Error", msg)
                return

            
            self.ik_status_label.config(
                text=f"Calculando IK para ({x:.3f}, {y:.3f}, {z:.3f})...", 
                fg="orange"
            )
            self.window.update()
            
            success = self.controller.move_to_xyz(x, y, z)
            
            if success:
                self.ik_status_label.config(
                    text=f"✓ Movido a ({x:.3f}, {y:.3f}, {z:.3f})", 
                    fg="green"
                )
                self.status_label.config(text="Robot movido a XYZ objetivo")
            else:
                self.ik_status_label.config(
                    text="✗ No se encontró solución IK válida", 
                    fg="red"
                )
                messagebox.showerror(
                    "Error",
                    "No se pudo alcanzar la posición solicitada.\n"
                    "Puede estar fuera del workspace o en singularidad."
                )
                
        except ValueError:
            messagebox.showerror("Error", "Por favor ingresa valores numéricos válidos")
            self.ik_status_label.config(text="Error en valores ingresados", fg="red")

    def move_to_preset_xyz(self, x, y, z):
        self.x_entry.delete(0, tk.END)
        self.x_entry.insert(0, f"{x:.3f}")
        self.y_entry.delete(0, tk.END)
        self.y_entry.insert(0, f"{y:.3f}")
        self.z_entry.delete(0, tk.END)
        self.z_entry.insert(0, f"{z:.3f}")
        self.move_to_xyz_target()

    def update_xyz_display_timer(self):
        try:
            x, y, z = self.controller.get_current_xyz()
            self.current_x_label.config(text=f"X: {x:.3f} m")
            self.current_y_label.config(text=f"Y: {y:.3f} m")
            self.current_z_label.config(text=f"Z: {z:.3f} m")
        except:
            pass
        self.window.after(200, self.update_xyz_display_timer)

    # ---------------- RViz ----------------

    def launch_rviz(self):
        try:
            cmd = ["ros2", "launch", "pincher_description", "display.launch.py"]

            def run_rviz():
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                self.window.after(0, self.on_rviz_closed)

            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()

            self.rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
            self.stop_rviz_btn.config(state=tk.NORMAL, bg="#f44336")
            self.rviz_status_label.config(
                text="RViz + robot_state_publisher ejecutándose", fg="green"
            )
            self.status_label.config(text="Lanzado display.launch.py (RViz + modelo)")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo lanzar RViz: {str(e)}")
            self.rviz_status_label.config(text=f"Error: {str(e)}", fg="red")

    def stop_rviz(self):
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        self.on_rviz_closed()

    def on_rviz_closed(self):
        self.rviz_btn.config(state=tk.NORMAL, bg="#2196F3")
        self.stop_rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
        self.rviz_status_label.config(text="RViz no iniciado", fg="red")
        self.status_label.config(text="RViz cerrado")

    def get_rviz_config_path(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_path = get_package_share_directory('phantomx_pincher_description')
            return os.path.join(pkg_path, 'rviz', 'pincher.rviz')
        except:
            return ""

    def update_joints_timer(self):
        for i, joint_name in enumerate(self.controller.joint_names):
            if i < len(self.controller.current_joint_positions):
                position = self.controller.current_joint_positions[i]
                self.joint_labels[joint_name].config(text=f"{position:.3f}")
        self.window.after(100, self.update_joints_timer)

    # ---------------- CALLBACKS DE GUI ----------------

    def on_motor_slider_change(self, motor_id):
        current_time = time.time()
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            position = self.sliders[motor_id].get()
            speed = self.speed_slider_tab1.get()
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, position)
                self.labels[motor_id].config(text=f'Pos: {position}')
                self.last_motor_update[motor_id] = current_time
                self.status_label.config(text=f"Motor {motor_id} moviéndose a {position}")
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo"))
            elif self.controller.emergency_stop_activated:
                self.status_label.config(
                    text=f"EMERGENCIA: No se puede mover motor {motor_id}", fg="red"
                )
            else:
                self.status_label.config(
                    text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange"
                )

    def on_speed_slider_change(self, value):
        current_time = time.time()
        if current_time - self.last_speed_update >= self.update_interval:
            speed = int(value)
            if not self.controller.emergency_stop_activated:
                self.controller.update_speed(speed)
                self.speed_value_label_tab1.config(text=str(speed))
                self.speed_value_label_tab2.config(text=str(speed))
                self.speed_slider_tab1.set(speed)
                self.speed_slider_tab2.set(speed)
                self.last_speed_update = current_time
                if speed == 0:
                    self.status_label.config(
                        text="Velocidad 0: Los motores no se moverán", fg="orange"
                    )
                else:
                    self.status_label.config(text=f"Velocidad actualizada: {speed}")
                self.window.after(
                    2000, lambda: self.status_label.config(text="Sistema Listo", fg="green")
                )
            else:
                self.status_label.config(
                    text="EMERGENCIA: No se puede cambiar velocidad", fg="red"
                )

    def move_single_motor_from_entry(self, motor_id):
        try:
            value = self.entries[motor_id].get()
            position = int(value)
            if 0 <= position <= 1023:
                speed = self.speed_slider_tab2.get()
                if speed == 0:
                    self.entry_labels[motor_id].config(text="Velocidad 0", fg="orange")
                    self.status_label.config(
                        text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange"
                    )
                elif self.controller.emergency_stop_activated:
                    self.entry_labels[motor_id].config(text="EMERGENCIA", fg="red")
                    self.status_label.config(
                        text="EMERGENCIA: No se puede mover motores", fg="red"
                    )
                else:
                    self.controller.move_motor(motor_id, position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    self.status_label.config(
                        text=f"Motor {motor_id} moviéndose a {position}"
                    )
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(position)
                    self.window.after(
                        2000,
                        lambda: self.entry_labels[motor_id].config(text="Listo", fg="green"),
                    )
                    self.window.after(
                        2000, lambda: self.status_label.config(text="Sistema Listo")
                    )
            else:
                self.entry_labels[motor_id].config(text="Error: 0-1023", fg="red")
        except ValueError:
            self.entry_labels[motor_id].config(text="Error: Número", fg="red")

    def move_all_motors_from_entries(self):
        speed = self.speed_slider_tab2.get()
        if speed == 0:
            self.status_label.config(
                text="Velocidad 0: Los motores no se moverán", fg="orange"
            )
            return
        if self.controller.emergency_stop_activated:
            self.status_label.config(
                text="EMERGENCIA: No se puede mover motores", fg="red"
            )
            return
            
        success_count = 0
        for motor_id in self.controller.dxl_ids:
            try:
                value = self.entries[motor_id].get()
                position = int(value)
                if 0 <= position <= 1023:
                    self.controller.move_motor(motor_id, position)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    success_count += 1
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(position)
                else:
                    self.entry_labels[motor_id].config(text="Error: 0-1023", fg="red")
            except ValueError:
                self.entry_labels[motor_id].config(text="Error: Número", fg="red")
        
        self.status_label.config(
            text=f"Comando enviado a {success_count}/{len(self.controller.dxl_ids)} motores"
        )
        self.window.after(
            3000,
            lambda: [label.config(text="Listo", fg="green") for label in self.entry_labels.values()],
        )
        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo"))

    def home_all(self):
        if self.controller.emergency_stop_activated:
            if not messagebox.askokcancel(
                "Reactivar Sistema",
                "La parada de emergencia está activada.\n\n"
                "¿Desea reactivar el sistema y mover los motores a HOME?",
            ):
                return
        self.controller.home_all_motors()
        self.status_label.config(text="Todos los motores movidos a HOME", fg="blue")
        
        for motor_id in self.controller.dxl_ids:
            if motor_id in self.sliders:
                self.sliders[motor_id].set(DEFAULT_GOAL)
                self.labels[motor_id].config(text=f'Pos: {DEFAULT_GOAL}')
            if motor_id in self.entries:
                self.entries[motor_id].delete(0, tk.END)
                self.entries[motor_id].insert(0, str(DEFAULT_GOAL))
                self.entry_labels[motor_id].config(text="HOME", fg="green")
        
        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))

    def emergency_stop(self):
        self.controller.emergency_stop()
        self.status_label.config(text="PARADA DE EMERGENCIA ACTIVADA", fg="red")
        for label in self.entry_labels.values():
            label.config(text="EMERGENCIA", fg="red")

    def on_close(self):
        if self.rviz_process:
            self.stop_rviz()
        if messagebox.askokcancel(
            "Salir",
            "¿Estás seguro de que quieres salir?\n"
            "Se desactivará el torque de los motores.",
        ):
            self.controller.close()
            self.window.destroy()
            rclpy.shutdown()

    def run(self):
        try:
            self.window.mainloop()
        except KeyboardInterrupt:
            self.on_close()

# ============================================================
#  MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    controller = PincherController()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    try:
        gui = PincherGUI(controller)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
