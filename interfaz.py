import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import random
import datetime

class ControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("HJP LOGÍSTICA - Control de Flota")
        # Aumentamos un poco la altura para que quepa la consola de logs
        self.root.geometry("535x750")
        
        # --- VOLVEMOS AL TEMA OSCURO ---
        self.COLOR_BG = "#1e293b"       # Fondo Slate 900 (Oscuro)
        self.COLOR_TEXT = "#ffffff"     # Texto blanco
        self.COLOR_ACCENT = "#38bdf8"   # Azul claro (Sky 400)
        self.COLOR_STATUS = "#fbbf24"   # Amarillo/Ámbar para estado
        self.COLOR_BTN_TEXT = "#000000" # Texto negro para botones de acción
        
        self.root.configure(bg=self.COLOR_BG) 

        # Variable de control para la simulación
        self.is_simulating = False
        self.step_index = 0  # Índice para saber en qué paso vamos
        self.last_topic = None # Para controlar el cierre de tareas anteriores

        # --- CONFIGURACIÓN DE FUENTES ---
        font_title = ("Helvetica", 22, "bold")
        font_label = ("Arial", 12, "bold") 
        font_input = ("Arial", 14) # Mantenemos tamaño grande
        font_btn_start = ("Arial", 14, "bold")
        font_status_title = ("Arial", 12)
        font_status_val = ("Courier", 26, "bold")   
        font_btn_stop = ("Arial", 14, "bold")
        font_console = ("Consolas", 10) 

        self.root.option_add('*TCombobox*Listbox.font', font_input)

        # --- LISTA MAESTRA DE UBICACIONES ---
        self.posibles_ubicaciones = [f"ESTANTERIA_{i}" for i in range(1, 7)]

        # --- DEFINICIÓN DE LA SECUENCIA DE ESTADOS (FLUJO DE TAREA) ---
        self.secuencia_estados = [
            "NAVEGACIÓN ORIGEN",    # Paso 2
            "VISUALIZANDO ARUCO",   # Paso 3
            "GRASPING",             # Paso 4
            "NAVEGACIÓN A DESTINO", # Paso 5
            "VISUALIZANDO ARUCO 2", # Paso 6
            "DEPOSITANDO",          # Paso 7
            "NAVEGACIÓN HOME",      # Paso 8
            "REPOSO"                # Vuelta al Paso 1 (Fin)
        ]

        # --- TÍTULO ---
        lbl_title = tk.Label(root, text="CONTROL DE MISIÓN", font=font_title, bg=self.COLOR_BG, fg=self.COLOR_ACCENT)
        lbl_title.pack(pady=(30, 20)) 

        # --- SELECCIÓN DE TAREA ---
        frame_inputs = tk.Frame(root, bg=self.COLOR_BG)
        frame_inputs.pack(pady=10) 

        # CONFIGURACIÓN DE ESTILO ESPECÍFICA PARA COMBOBOX
        style = ttk.Style()
        style.theme_use('clam')
        
        # AQUÍ ESTÁ EL CAMBIO: Fondo claro (#f1f5f9) con texto oscuro (#0f172a)
        # Esto hace que resalte sobre el fondo oscuro de la app
        style.configure("TCombobox", 
                        fieldbackground="#f1f5f9",  # Fondo de la caja de texto (Claro)
                        background="#94a3b8",       # Fondo de la flechita (Gris medio)
                        foreground="#0f172a",       # Color de la letra (Oscuro)
                        arrowsize=20)

        # Origen
        tk.Label(frame_inputs, text="Origen (ID_LOC1):", font=font_label, bg=self.COLOR_BG, fg=self.COLOR_TEXT).grid(row=0, column=0, padx=10, pady=10, sticky="e")
        self.combo_origin = ttk.Combobox(frame_inputs, values=self.posibles_ubicaciones, font=font_input, width=20, state="readonly")
        self.combo_origin.grid(row=0, column=1, padx=10, pady=10)
        self.combo_origin.current(0) 

        # Destino
        tk.Label(frame_inputs, text="Destino (ID_LOC2):", font=font_label, bg=self.COLOR_BG, fg=self.COLOR_TEXT).grid(row=1, column=0, padx=10, pady=10, sticky="e")
        self.combo_target = ttk.Combobox(frame_inputs, values=self.posibles_ubicaciones, font=font_input, width=20, state="readonly")
        self.combo_target.grid(row=1, column=1, padx=10, pady=10)
        self.combo_target.current(1) 

        # --- BOTÓN DE ACCIÓN ---
        self.btn_start = tk.Button(root, text="INICIAR TAREA", command=self.send_order, 
                                   bg=self.COLOR_ACCENT, fg="black", font=font_btn_start, 
                                   height=2, width=18, cursor="hand2")
        self.btn_start.pack(pady=20)

        # --- MONITOR DE ESTADO ---
        self.lbl_status_title = tk.Label(root, text="ESTADO DEL ROBOT:", font=font_status_title, bg=self.COLOR_BG, fg="#94a3b8")
        self.lbl_status_title.pack(pady=(10, 5))
        
        self.lbl_current_state = tk.Label(root, text="REPOSO", font=font_status_val, bg=self.COLOR_BG, fg=self.COLOR_STATUS)
        self.lbl_current_state.pack(pady=10)

        # --- CONSOLA DE COMUNICACIONES ---
        lbl_log = tk.Label(root, text="REGISTRO DE COMUNICACIONES (TOPICS):", font=("Arial", 10, "bold"), bg=self.COLOR_BG, fg="#cbd5e1")
        lbl_log.pack(pady=(20, 5), anchor="w", padx=20)

        # Consola estilo Matrix/Terminal: Fondo oscuro, letra verde
        self.console_log = scrolledtext.ScrolledText(root, height=8, bg="#0f172a", fg="#22c55e", font=font_console, state='disabled', borderwidth=1, relief="solid")
        self.console_log.pack(padx=20, pady=5, fill="x")

        # --- PARADA EMERGENCIA ---
        btn_stop = tk.Button(root, text="PARADA DE EMERGENCIA", command=self.emergency_stop, 
                             bg="#ef4444", fg="white", font=font_btn_stop, 
                             height=2, width=25, cursor="hand2")
        btn_stop.pack(side=tk.BOTTOM, pady=30)

    def log_message(self, message):
        """Escribe en la consola visual de la interfaz"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] {message}\n"
        
        self.console_log.config(state='normal') # Habilitar escritura
        self.console_log.insert(tk.END, full_msg)
        self.console_log.see(tk.END) # Scroll automático al final
        self.console_log.config(state='disabled') # Bloquear escritura

    def publicar_mensaje(self, topic, payload):
        """Simula la publicación MQTT/ROS"""
        msg = f"PUB [{topic}]: {payload}"
        print(msg) # Salida terminal
        self.log_message(msg) # Salida GUI

    def get_topic_from_state(self, state):
        """Devuelve el topic asociado a un estado"""
        # MODIFICACIÓN: Quitamos "REPOSO" de aquí para que cambie de topic y cierre la navegación
        if "NAVEGACIÓN" in state:
            return "navegacion"
        elif "VISUALIZANDO" in state:
            return "orientacion"
        elif "GRASPING" in state:
            return "agarre"
        elif "DEPOSITANDO" in state:
            return "deposicion"
        return "tarea"

    def send_order(self):
        origin = self.combo_origin.get()
        target = self.combo_target.get()
        
        if origin == target:
            self.log_message("⚠️ WARN: Origen y Destino idénticos")
        
        self.log_message(f"--> ORDEN ENVIADA: {origin} -> {target}")
        
        if not self.is_simulating:
            # PUBLICACIÓN INICIAL DE TAREA - AHORA SOLO ON
            self.publicar_mensaje("tarea", "STATUS: ON")
            
            self.is_simulating = True
            self.step_index = 0
            self.last_topic = None # Resetear el último topic
            self.lbl_current_state.config(fg=self.COLOR_STATUS) 
            self.update_dummy_status()

    def emergency_stop(self):
        # CAMBIO SOLICITADO: Solo informa de STATUS: OFF, sin distinguir emergencia
        self.publicar_mensaje("tarea", "STATUS: OFF")
        self.is_simulating = False
        self.lbl_current_state.config(text="STOPPED", fg="#ef4444")

    def update_dummy_status(self):
        if not self.is_simulating:
            return

        if self.step_index < len(self.secuencia_estados):
            nuevo_estado = self.secuencia_estados[self.step_index]
            self.lbl_current_state.config(text=nuevo_estado)
            
            # --- LÓGICA DE PUBLICACIÓN REORDENADA ---
            # 1. Determinar el topic actual
            current_topic = self.get_topic_from_state(nuevo_estado)
            
            # 2. PRIMERO: Si cambiamos de tarea (topic), cerramos la anterior con OFF
            # Al ser REPOSO un topic diferente ("tarea"), esto cerrará "navegacion"
            if self.last_topic and self.last_topic != current_topic:
                self.publicar_mensaje(self.last_topic, "STATUS: OFF")

            # 3. SEGUNDO: Publicar estado general para monitorización
            self.publicar_mensaje("estado", f"ID: {self.step_index + 1} | {nuevo_estado}")

            # 4. TERCERO: Indicamos que la tarea actual está activa con ON (EXCEPTO EN REPOSO)
            if nuevo_estado != "REPOSO":
                self.publicar_mensaje(current_topic, "STATUS: ON")

            # Guardamos referencia para la siguiente vuelta
            self.last_topic = current_topic
            # --- FIN LÓGICA ---

            if nuevo_estado == "REPOSO":
                # PUBLICACIÓN FINAL DE TAREA - AHORA SOLO OFF
                self.publicar_mensaje("tarea", "STATUS: OFF")
                self.is_simulating = False
                return 

            self.step_index += 1
            self.root.after(3000, self.update_dummy_status) # 3 segs para dar tiempo a leer

if __name__ == "__main__":
    root = tk.Tk()
    app = ControlPanel(root)
    root.mainloop()