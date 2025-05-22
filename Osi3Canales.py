import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import serial
import threading
import time
import serial.tools.list_ports
from collections import deque
from PIL import Image, ImageTk

# Configuración inicial
ctk.set_appearance_mode("System")
ctk.set_default_color_theme("dark-blue")

root = ctk.CTk()
root.title("Osciloscopio con ESP32")
root.geometry("900x750")

# Parámetros de señal
amplitude_scale = 0.2
time_scale = 5.0
center_voltage = 1.8

# Variables de adquisición
data = deque()
data_ch2 = deque()
data_ch3 = deque()
esp = None
running = False
paused = False
start_time = time.time()
show_ch2 = ctk.BooleanVar(value=True)
show_ch3 = ctk.BooleanVar(value=True)

# Matplotlib
fig, ax = plt.subplots(figsize=(10, 4), dpi=150)
line, = ax.plot([], [], color="blue", linewidth=1.5, antialiased=True, label="Canal 1")
line2, = ax.plot([], [], color="green", linewidth=1.0, antialiased=True, label="Canal 2")
line3, = ax.plot([], [], color="red", linewidth=1.0, antialiased=True, label="Canal 3")
ax.set_title("Señal del Osciloscopio")
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Voltaje (V)")
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_xlim(0, time_scale)
ax.set_ylim(center_voltage - amplitude_scale, center_voltage + amplitude_scale)
ax.legend(loc="upper right")

# Cargar imágenes
def cargar_imagen(ruta, ancho, alto):
    imagen = Image.open(ruta)
    imagen = imagen.resize((ancho, alto), Image.LANCZOS)
    return ImageTk.PhotoImage(imagen)

start_image = cargar_imagen("/home/mario/Documentos/Spyder/start.png", 60, 60)
stop_image = cargar_imagen("/home/mario/Documentos/Spyder/stop.png", 60, 60)
pause_image = cargar_imagen("/home/mario/Documentos/Spyder/pause.png", 60, 60)
resume_image = cargar_imagen("/home/mario/Documentos/Spyder/resume.png", 60, 60)

# UI Callbacks
def update_amplitude(value):
    global amplitude_scale
    amplitude_scale = float(value)
    amplitude_value_label.configure(text=f"Amplitud: {amplitude_scale:.2f}")

def update_time(value):
    global time_scale
    time_scale = float(value)
    time_value_label.configure(text=f"Tiempo: {time_scale:.2f}")

def select_com():
    global esp
    com_port = com_combobox.get()
    try:
        esp = serial.Serial(com_port, 115200, timeout=1)
        time.sleep(2)
        status_label.configure(text=f"Conectado a {com_port}")
    except Exception as e:
        status_label.configure(text=f"Error: {e}")

def toggle_acquisition():
    global running, data, start_time
    if running:
        running = False
        data.clear()
        line.set_data([], [])
        fig.canvas.draw()
        start_stop_button.configure(image=start_image)
    else:
        if esp is None:
            status_label.configure(text="Selecciona un puerto COM primero")
            return
        running = True
        data.clear()
        start_time = time.time()
        start_stop_button.configure(image=stop_image)
        threading.Thread(target=acquire_data, daemon=True).start()

def toggle_pause():
    global paused
    paused = not paused
    pause_resume_button.configure(image=resume_image if paused else pause_image)

def acquire_data():
    global data, data_ch2, data_ch3, running, paused
    buffer = ""
    while running:
        if not paused and esp.in_waiting:
            try:
                buffer += esp.read(esp.in_waiting).decode(errors="ignore")
                lines = buffer.split("\n")
                buffer = lines[-1]

                for line_data in lines[:-1]:
                    line_data = line_data.strip()
                    if "," in line_data:
                        parts = line_data.split(",")
                        if len(parts) >= 2:
                            try:
                                voltage1 = float(parts[0])
                                bpm = int(parts[1])
                                bpm_label.configure(text=f"BPM: {bpm}")
                                voltage2 = float(parts[2]) if len(parts) > 2 else None
                                voltage3 = float(parts[3]) if len(parts) > 3 else None

                                current_time = time.time() - start_time
                                data.append((current_time, voltage1))
                                if voltage2 is not None:
                                    data_ch2.append((current_time, voltage2))
                                if voltage3 is not None:
                                    data_ch3.append((current_time, voltage3))

                                # Limitar tamaño
                                for d in (data, data_ch2, data_ch3):
                                    while d and d[0][0] < current_time - time_scale:
                                        d.popleft()
                            except ValueError:
                                continue
            except Exception as e:
                print(f"[Error lectura]: {e}")
        time.sleep(0.001 if not paused else 0.05)


# --- dentro de update_plot()
def update_plot(frame):
    if running and data:
        times1, voltages1 = zip(*data)
        line.set_data(times1, voltages1)
        ax.set_xlim(max(0, times1[-1] - time_scale), times1[-1])
        v_mean = sum(voltages1) / len(voltages1)
        ax.set_ylim(v_mean - amplitude_scale, v_mean + amplitude_scale)

        if show_ch2.get() and data_ch2:
            times2, voltages2 = zip(*data_ch2)
            line2.set_data(times2, voltages2)
        else:
            line2.set_data([], [])

        if show_ch3.get() and data_ch3:
            times3, voltages3 = zip(*data_ch3)
            line3.set_data(times3, voltages3)
        else:
            line3.set_data([], [])

        fig.canvas.draw_idle()

# UI
def build_ui():
    main_frame = ctk.CTkFrame(root)
    main_frame.pack(pady=20, padx=20, fill="x")

    frame_left = ctk.CTkFrame(main_frame)
    frame_left.pack(side="left", padx=20)

    global com_combobox, status_label
    com_combobox = ctk.CTkComboBox(frame_left, values=[port.device for port in serial.tools.list_ports.comports()])
    com_combobox.pack(pady=5)

    ctk.CTkButton(frame_left, text="Seleccionar COM", command=select_com).pack(pady=5)
    status_label = ctk.CTkLabel(frame_left, text="Selecciona un puerto COM")
    status_label.pack()

    frame_center = ctk.CTkFrame(main_frame)
    frame_center.pack(side="left", padx=20)

    global amplitude_slider, amplitude_value_label, time_slider, time_value_label
    amplitude_slider = ctk.CTkSlider(frame_center, from_=0.05, to=6.0, number_of_steps=100, command=update_amplitude)
    amplitude_slider.set(amplitude_scale)
    amplitude_slider.pack(pady=5)
    amplitude_value_label = ctk.CTkLabel(frame_center, text=f"Amplitud: {amplitude_scale:.2f}")
    amplitude_value_label.pack()

    time_slider = ctk.CTkSlider(frame_center, from_=1.0, to=15.0, command=update_time)
    time_slider.set(time_scale)
    time_slider.pack(pady=5)
    time_value_label = ctk.CTkLabel(frame_center, text=f"Tiempo: {time_scale:.2f}")
    time_value_label.pack()

    frame_right = ctk.CTkFrame(main_frame)
    frame_right.pack(side="right", padx=20)

    global start_stop_button, pause_resume_button
    start_stop_button = ctk.CTkButton(frame_right, image=start_image, text="", command=toggle_acquisition, width=80, height=80)
    start_stop_button.pack(pady=5)

    pause_resume_button = ctk.CTkButton(frame_right, image=pause_image, text="", command=toggle_pause, width=80, height=80)
    pause_resume_button.pack(pady=5)

    global bpm_label
    bpm_label = ctk.CTkLabel(root, text="BPM: --")
    bpm_label.pack(pady=5)
    
    toggle_frame = ctk.CTkFrame(root)
    toggle_frame.pack(pady=10)

    ctk.CTkCheckBox(toggle_frame, text="Ver Canal 2", variable=show_ch2).pack(side="left", padx=10)
    ctk.CTkCheckBox(toggle_frame, text="Ver Canal 3", variable=show_ch3).pack(side="left", padx=10)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill="both", expand=True)

build_ui()
ani = animation.FuncAnimation(fig, update_plot, interval=50)
root.mainloop()

if esp:
    esp.close()
