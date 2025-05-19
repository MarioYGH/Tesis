import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import serial
import threading
import time
import serial.tools.list_ports
from PIL import Image, ImageTk

# Configuración de la ventana principal
ctk.set_appearance_mode("System")
ctk.set_default_color_theme("dark-blue")

root = ctk.CTk()
root.title("Osciloscopio con ESP32")
root.geometry("900x750")

# Variables para la adquisición de señales
running = False
paused = False
amplitude_scale = 0.2
time_scale = 5.0
center_voltage = 1.8
data = []
esp = None
start_time = time.time()

# Crear la figura de Matplotlib con mejor resolución y tamaño
fig, ax = plt.subplots(figsize=(10, 4), dpi=150)
ax.set_title("Señal del Osciloscopio")
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("Voltaje (V)")
line, = ax.plot([], [], color="blue", linewidth=1.5, antialiased=True)
ax.set_xlim(0, time_scale)
ax.set_ylim(center_voltage - amplitude_scale, center_voltage + amplitude_scale)
ax.grid(True, linestyle='--', alpha=0.5)

# Función para cargar y redimensionar imágenes
def cargar_imagen(ruta, ancho, alto):
    imagen = Image.open(ruta)
    imagen = imagen.resize((ancho, alto), Image.LANCZOS)
    return ImageTk.PhotoImage(imagen)

# Cargar imágenes
start_image = cargar_imagen("/home/mario/Documentos/Spyder/start.png", 60, 60)
stop_image = cargar_imagen("/home/mario/Documentos/Spyder/stop.png", 60, 60)
pause_image = cargar_imagen("/home/mario/Documentos/Spyder/pause.png", 60, 60)
resume_image = cargar_imagen("/home/mario/Documentos/Spyder/resume.png", 60, 60)

def update_amplitude(value):
    global amplitude_scale
    amplitude_scale = float(value)
    fig.canvas.draw()
    amplitude_value_label.configure(text=f"Amplitud: {amplitude_scale:.2f}")

def update_time(value):
    global time_scale
    time_scale = float(value)
    fig.canvas.draw()
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
    global running, data
    if running:
        running = False
        data = []
        line.set_data([], [])
        fig.canvas.draw()
        start_stop_button.configure(image=start_image)
    else:
        if esp is None:
            status_label.configure(text="Selecciona un puerto COM primero")
            return
        running = True
        data = []
        start_stop_button.configure(image=stop_image)
        threading.Thread(target=acquire_data, daemon=True).start()

def toggle_pause():
    global paused
    paused = not paused
    pause_resume_button.configure(image=resume_image if paused else pause_image)

def acquire_data():
    global data, running, paused, start_time
    start_time = time.time()
    while running:
        if not paused:
            try:
                line_data = esp.readline().decode(errors="ignore").strip()
                if "," in line_data:
                    parts = line_data.split(",")
                    if len(parts) == 2:
                        voltage = float(parts[0])
                        bpm = int(parts[1])
                        bpm_label.configure(text=f"BPM: {bpm}")
                        current_time = time.time() - start_time
                        data.append((current_time, voltage))
                        data = [(t, v) for t, v in data if t > current_time - time_scale]
                else:
                    print(f"[Línea ignorada] {line_data}")
            except Exception as e:
                print(f"Error: {e}")
        else:# Crear la figura de Matplotlib

            time.sleep(0.1)

def update_plot(frame):# Crear la figura de Matplotlib

    if running and not paused and data:
        times, voltages = zip(*data)
        line.set_data(times, voltages)
        ax.set_xlim(max(0, times[-1] - time_scale), times[-1])
        
        # Mejor centrado dinámico del eje Y
        v_mean = sum(voltages) / len(voltages)
        ax.set_ylim(v_mean - amplitude_scale, v_mean + amplitude_scale)

        fig.canvas.draw()

# Interfaz
def build_ui():
    main_frame = ctk.CTkFrame(root)
    main_frame.pack(pady=20, padx=20, fill="x")

    frame_left = ctk.CTkFrame(main_frame)
    frame_left.pack(side="left", padx=20)

    global com_combobox, status_label
    com_combobox = ctk.CTkComboBox(frame_left, values=[port.device for port in serial.tools.list_ports.comports()])
    com_combobox.pack(pady=5)

    select_com_button = ctk.CTkButton(frame_left, text="Seleccionar COM", command=select_com)
    select_com_button.pack(pady=5)

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

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill="both", expand=True)

build_ui()
ani = animation.FuncAnimation(fig, update_plot, interval=50)
root.mainloop()

if esp:
    esp.close()
