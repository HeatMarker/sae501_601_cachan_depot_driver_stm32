##
# @file serial_app.py
# @brief Interface de contrôle Robot STM32 via Série
# @author SCHWAGER Jérôme
# @date 2025
#

import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time
import struct

##
# @brief Calcule le CRC8 (Polynôme 0x07, Init 0x00)
# @param data Liste des octets à traiter
# @return L'octet de CRC calculé
def crc8_atm(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = (crc << 1)
            crc &= 0xFF
    return crc

##
# @class SerialApp
# @brief Classe principale de l'application graphique
class SerialApp(ctk.CTk):
    
    ##
    # @brief Constructeur de l'application
    # Initialise la fenêtre, les variables et l'interface
    def __init__(self):
        super().__init__()

        self.title("STM32 Robot Controller")
        self.geometry("1100x720")
        
        self.ser = None
        self.is_connected = False
        self.read_thread = None
        self.stop_thread = False
        self.is_auto_sending = False
        
        self.rx_buffer = bytearray()
        self.last_imu_update = 0

        self._init_ui()
        self._refresh_ports()

    ##
    # @brief Initialisation des composants graphiques
    def _init_ui(self):
        
        # --- Section Connexion ---
        self.frame_conn = ctk.CTkFrame(self)
        self.frame_conn.pack(pady=5, padx=10, fill="x")

        self.combo_ports = ctk.CTkComboBox(self.frame_conn, values=["Aucun port"])
        self.combo_ports.pack(side="left", padx=5)

        self.btn_refresh = ctk.CTkButton(self.frame_conn, text="↻", width=30, command=self._refresh_ports)
        self.btn_refresh.pack(side="left", padx=5)

        self.btn_connect = ctk.CTkButton(self.frame_conn, text="Connexion", fg_color="green", command=self._toggle_connection)
        self.btn_connect.pack(side="left", padx=5)

        self.lbl_status = ctk.CTkLabel(self.frame_conn, text="Déconnecté", text_color="red")
        self.lbl_status.pack(side="left", padx=10)

        # --- Section Commande Manuelle ---
        self.frame_cmd = ctk.CTkFrame(self)
        self.frame_cmd.pack(pady=5, padx=10, fill="x")

        # Ligne 0 : Registre et Mode RW
        ctk.CTkLabel(self.frame_cmd, text="Reg (Hex): 0x").grid(row=0, column=0, padx=5, pady=5)
        self.entry_reg = ctk.CTkEntry(self.frame_cmd, width=60, placeholder_text="REG")
        self.entry_reg.grid(row=0, column=1, padx=5, pady=5)

        self.switch_rw = ctk.CTkSwitch(self.frame_cmd, text="WRITE", command=self._update_rw_mode)
        self.switch_rw.grid(row=0, column=2, padx=10, pady=5)

        # Ligne 1 : Data et Boutons
        ctk.CTkLabel(self.frame_cmd, text="Data (Int):").grid(row=1, column=0, padx=5, pady=5)
        self.entry_data = ctk.CTkEntry(self.frame_cmd, width=100, placeholder_text="DATA")
        self.entry_data.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="ew")

        self.btn_send = ctk.CTkButton(self.frame_cmd, text="ENVOYER", command=self._send_frame)
        self.btn_send.grid(row=1, column=3, padx=5, pady=5)

        self.btn_auto = ctk.CTkButton(self.frame_cmd, text="Heart Beat (100ms)", fg_color="orange", command=self._toggle_auto_send)
        self.btn_auto.grid(row=1, column=4, padx=5, pady=5)

        self.btn_clear = ctk.CTkButton(self.frame_cmd, text="Clear Logs", fg_color="gray", width=80, command=self._clear_terminal)
        self.btn_clear.grid(row=1, column=5, padx=20, pady=5)

        # Ligne 2 : Info bulle
        self.lbl_rw_info = ctk.CTkLabel(self.frame_cmd, text="", text_color="gray", font=("Arial", 11))
        self.lbl_rw_info.grid(row=2, column=0, columnspan=6, padx=5, pady=(0, 5), sticky="w")

        # --- Section Pilotage Direct ---
        self.frame_pilot = ctk.CTkFrame(self)
        self.frame_pilot.pack(pady=5, padx=10, fill="x")
        
        ctk.CTkLabel(self.frame_pilot, text="PILOTAGE DIRECT", font=("Arial", 12, "bold")).pack(pady=2)

        self.lbl_servo = ctk.CTkLabel(self.frame_pilot, text="Servo: 0°")
        self.lbl_servo.pack()
        self.slider_servo = ctk.CTkSlider(self.frame_pilot, from_=-20, to=20, number_of_steps=40, command=self._on_servo_slide)
        self.slider_servo.set(0) 
        self.slider_servo.pack(fill="x", padx=20, pady=2)

        self.lbl_motor = ctk.CTkLabel(self.frame_pilot, text="Moteur: 0 mm/s")
        self.lbl_motor.pack()
        self.slider_motor = ctk.CTkSlider(self.frame_pilot, from_=-1000, to=1000, number_of_steps=200, command=self._on_motor_slide)
        self.slider_motor.set(0)
        self.slider_motor.pack(fill="x", padx=20, pady=2)
        
        self.btn_stop = ctk.CTkButton(self.frame_pilot, text="ARU", fg_color="red", height=40, font=("Arial", 14, "bold"), command=self._emergency_stop)
        self.btn_stop.pack(pady=10)

        # --- Section Logs et Télémétrie ---
        self.frame_split = ctk.CTkFrame(self)
        self.frame_split.pack(pady=5, padx=10, fill="both", expand=True)
        self.frame_split.grid_columnconfigure(0, weight=1)
        self.frame_split.grid_columnconfigure(1, weight=1)
        self.frame_split.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(self.frame_split, text="LOGS COMMANDES & DEBUG", font=("Arial", 14, "bold")).grid(row=0, column=0, pady=5)
        ctk.CTkLabel(self.frame_split, text="TELEMETRIE (Temps Réel)", font=("Arial", 14, "bold")).grid(row=0, column=1, pady=5)

        self.txt_log = ctk.CTkTextbox(self.frame_split, width=400)
        self.txt_log.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.txt_log.configure(state="disabled")

        self.txt_imu = ctk.CTkTextbox(self.frame_split, width=400)
        self.txt_imu.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")
        self.txt_imu.configure(state="disabled", font=("Consolas", 14))

    ##
    # @brief Met à jour l'affichage selon le mode Read/Write sélectionné
    def _update_rw_mode(self):
        if self.switch_rw.get() == 1:
            self.switch_rw.configure(text="READ")
            self.lbl_rw_info.configure(text="Mode READ : 'Data' indique le nombre de registres à lire (Burst)")
        else:
            self.switch_rw.configure(text="WRITE")
            self.lbl_rw_info.configure(text="")

    ##
    # @brief Rafraîchit la liste des ports COM disponibles
    def _refresh_ports(self):
        ports_list = [p.device for p in serial.tools.list_ports.comports()]
        if not ports_list:
            self.combo_ports.configure(values=["Aucun port"])
        else:
            self.combo_ports.configure(values=ports_list)
            # Sélection automatique de ttyACM0 si présent
            if "/dev/ttyACM0" in ports_list:
                self.combo_ports.set("/dev/ttyACM0")
            else:
                self.combo_ports.set(ports_list[0])

    ##
    # @brief Gère la connexion et déconnexion au port série
    def _toggle_connection(self):
        if not self.is_connected:
            port = self.combo_ports.get()
            if port == "Aucun port": return
            
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.configure(text="Déconnexion", fg_color="red")
                self.lbl_status.configure(text=f"Connecté à {port}", text_color="green")
                
                self.rx_buffer = bytearray()
                self.stop_thread = False
                self.read_thread = threading.Thread(target=self._read_serial_loop)
                self.read_thread.start()
                
            except Exception as e:
                self._log_cmd(f"Erreur connexion: {e}")
        else:
            self.is_auto_sending = False
            self.btn_auto.configure(text="Heart Beat (100ms)", fg_color="orange")
            self.stop_thread = True
            if self.ser:
                self.ser.close()
            self.is_connected = False
            self.btn_connect.configure(text="Connexion", fg_color="green")
            self.lbl_status.configure(text="Déconnecté", text_color="red")

    ##
    # @brief Active ou désactive l'envoi automatique (Heart Beat)
    def _toggle_auto_send(self):
        if not self.is_connected:
            self._log_cmd("Erreur: Non connecté")
            return

        if not self.is_auto_sending:
            self.is_auto_sending = True
            self.btn_auto.configure(text="STOP Auto", fg_color="red")
            self._auto_send_loop()
        else:
            self.is_auto_sending = False
            self.btn_auto.configure(text="Heart Beat (100ms)", fg_color="orange")

    ##
    # @brief Boucle d'envoi automatique (Thread non-bloquant via .after)
    def _auto_send_loop(self):
        if self.is_auto_sending and self.is_connected:
            self._send_frame()
            self.after(100, self._auto_send_loop)
        else:
            self.is_auto_sending = False
            self.btn_auto.configure(text="Heart Beat (100ms)", fg_color="orange")

    ##
    # @brief Construit et envoie la trame série basée sur les champs de saisie
    # Gère également la mise à jour visuelle des sliders si en mode WRITE
    def _send_frame(self):
        if not self.is_connected or not self.ser:
            self._log_cmd("Erreur: Non connecté")
            return

        try:
            addr_str = self.entry_reg.get().strip()
            addr = int(addr_str, 16) & 0x7F
            
            is_read = (self.switch_rw.get() == 1)
            hdr = (0x80 if is_read else 0x00) | addr

            data_str = self.entry_data.get().strip()
            data_val = int(data_str) if data_str else 0
            
            # Synchronisation inverse : Entry -> Slider (si mode Write)
            if not is_read:
                if addr == 0x00:
                    self.slider_servo.set(data_val)
                    self.lbl_servo.configure(text=f"Servo: {data_val}°")
                elif addr == 0x01:
                    self.slider_motor.set(data_val)
                    self.lbl_motor.configure(text=f"Moteur: {data_val} mm/s")

            d0 = 0
            d1 = 0

            if is_read:
                d0 = data_val & 0xFF 
                d1 = 0             
            else:
                d0 = data_val & 0xFF        
                d1 = (data_val >> 8) & 0xFF 

            payload = [hdr, d0, d1]
            crc = crc8_atm(payload)
            frame = bytearray(payload + [crc])

            self.ser.write(frame)
            
            hex_frame = " ".join([f"{b:02X}" for b in frame])
            mode_str = "READ" if is_read else "WRITE"
            # Log uniquement si pas en mode auto pour éviter le spam, ou optionnel
            if not self.is_auto_sending:
                self._log_cmd(f"TX [{mode_str} Reg:0x{addr:02X}]: {hex_frame}")

        except ValueError:
            self._log_cmd("Erreur: Format invalide")
        except Exception as e:
            self._log_cmd(f"Erreur envoi: {e}")

    ##
    # @brief Callback du slider Servo
    # Met à jour les champs manuels et force l'activation du Heart Beat
    # @param value Valeur du slider (float)
    def _on_servo_slide(self, value):
        angle = int(value)
        self.lbl_servo.configure(text=f"Servo: {angle}°")
        
        # Mise à jour des champs de texte pour que le Heart Beat envoie la bonne valeur
        self.entry_reg.delete(0, "end")
        self.entry_reg.insert(0, "00") # Reg Servo
        self.entry_data.delete(0, "end")
        self.entry_data.insert(0, str(angle))

        # Force le mode WRITE si on était en READ
        if self.switch_rw.get() == 1:
            self.switch_rw.toggle()
            self._update_rw_mode()
        
        # Active le Heart Beat si éteint
        if not self.is_auto_sending:
            self._toggle_auto_send()

    ##
    # @brief Callback du slider Moteur
    # Met à jour les champs manuels et force l'activation du Heart Beat
    # @param value Valeur du slider (float)
    def _on_motor_slide(self, value):
        speed = int(value)
        self.lbl_motor.configure(text=f"Moteur: {speed} mm/s")
        
        # 1. Mise à jour des champs de texte
        self.entry_reg.delete(0, "end")
        self.entry_reg.insert(0, "01") # Reg Moteur
        self.entry_data.delete(0, "end")
        self.entry_data.insert(0, str(speed))

        # 2. Force le mode WRITE
        if self.switch_rw.get() == 1:
            self.switch_rw.toggle()
            self._update_rw_mode()

        # 3. Active le Heart Beat si éteint
        if not self.is_auto_sending:
            self._toggle_auto_send()

    ##
    # @brief Procédure d'arrêt d'urgence
    # Remet tout à zéro et coupe les moteurs
    def _emergency_stop(self):
        # Reset Sliders
        self.slider_motor.set(0)
        self.slider_servo.set(0) 
        
        # Mise à jour des labels
        self.lbl_motor.configure(text="Moteur: 0 mm/s")
        self.lbl_servo.configure(text="Servo: 0°")

        # Mise à jour des champs pour le Heart Beat
        self.entry_reg.delete(0, "end")
        self.entry_reg.insert(0, "01") # Priorité coupe moteur
        self.entry_data.delete(0, "end")
        self.entry_data.insert(0, "0")
        
        # On force le Heart Beat pour envoyer l'arrêt en boucle
        if not self.is_auto_sending:
            self._toggle_auto_send()
            
        self._log_cmd("!!! ARU SEND !!!")

    ##
    # @brief Thread de lecture du port série
    # Décode les trames IMU (37 octets) et les réponses Commandes (4 octets)
    def _read_serial_loop(self):
        IMU_SIZE = 37 
        CMD_SIZE = 4

        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.rx_buffer.extend(data)
                    
                    while len(self.rx_buffer) > 0:
                        # CAS 1 : Trame IMU (Start 0xAA 0x55)
                        if self.rx_buffer[0] == 0xAA:
                            if len(self.rx_buffer) < 2: break
                            if self.rx_buffer[1] == 0x55:
                                if len(self.rx_buffer) < IMU_SIZE: break
                                packet = self.rx_buffer[:IMU_SIZE]
                                if crc8_atm(packet[:-1]) == packet[-1]:
                                    self._decode_and_show_imu(packet)
                                    del self.rx_buffer[:IMU_SIZE]
                                    continue
                                else:
                                    del self.rx_buffer[:1]
                                    continue
                            else:
                                del self.rx_buffer[:1]
                                continue
                        
                        # CAS 2 : Réponse Commande (Header bit7=0)
                        elif (self.rx_buffer[0] & 0x80) == 0x00:
                            if len(self.rx_buffer) < CMD_SIZE: break
                            packet = self.rx_buffer[:CMD_SIZE]
                            if crc8_atm(packet[:-1]) == packet[-1]:
                                self._decode_and_log_cmd(packet)
                                del self.rx_buffer[:CMD_SIZE]
                                continue
                            else:
                                del self.rx_buffer[:1]
                                continue

                        # CAS 3 : Octet inconnu
                        else:
                            del self.rx_buffer[:1]
                else:
                    time.sleep(0.005)
            except Exception:
                break

    ##
    # @brief Décode et affiche les données IMU
    # @param packet Le paquet brut de 33 octets
    def _decode_and_show_imu(self, packet):
        try:
            now = time.time()
            # Limite le rafraichissement UI à 10Hz
            if (now - self.last_imu_update) < 0.1:
                return
            self.last_imu_update = now
            
            unpacked = struct.unpack('<BBBBIf f f f f f f B', packet)
            timestamp = unpacked[4]
            ax, ay, az = unpacked[5], unpacked[6], unpacked[7]
            gx, gy, gz = unpacked[8], unpacked[9], unpacked[10]
            speed = unpacked[11]

            display_text = (
                f"--- IMU DATA UPDATE ---\n"
                f"TIMESTAMP : {timestamp} ms\n\n"
                f"ACCEL (mm/s²)\n"
                f"  X: {ax:>8.2f}\n"
                f"  Y: {ay:>8.2f}\n"
                f"  Z: {az:>8.2f}\n\n"
                f"GYRO (rad/s)\n"
                f"  X: {gx:>8.2f}\n"
                f"  Y: {gy:>8.2f}\n"
                f"  Z: {gz:>8.2f}\n"
                f"SPEED (m/s)\n"
                f"  {speed:>8.2f}\n"
            )
            self.txt_imu.configure(state="normal")
            self.txt_imu.delete("1.0", "end")
            self.txt_imu.insert("end", display_text)
            self.txt_imu.configure(state="disabled")
        except Exception:
            pass

    ##
    # @brief Décode et log les réponses aux commandes READ
    # @param packet Le paquet brut de 4 octets
    def _decode_and_log_cmd(self, packet):
        try:
            header = packet[0]
            d0 = packet[1]
            d1 = packet[2]
            crc = packet[3]
            
            addr = header & 0x7F
            value = d0 | (d1 << 8)
            if value > 32767: value -= 65536
            
            hex_str = f"{header:02X} {d0:02X} {d1:02X} {crc:02X}"
            
            self._log_cmd(f"RX [READ Reg:0x{addr:02X}]: {hex_str} (Value_decimal={value})")
        except Exception as e:
            self._log_cmd(f"Erreur Decode CMD: {e}")

    ##
    # @brief Ajoute un message dans la console de logs
    # @param message Le texte à afficher
    def _log_cmd(self, message):
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", message + "\n")
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")
    
    ##
    # @brief Efface la console de logs
    def _clear_terminal(self):
        self.txt_log.configure(state="normal")
        self.txt_log.delete("1.0", "end")
        self.txt_log.configure(state="disabled")

    ##
    # @brief Gestion de la fermeture de la fenêtre
    # Arrête les threads et ferme le port série
    def on_closing(self):
        self.stop_thread = True
        self.is_auto_sending = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.destroy()

if __name__ == "__main__":
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("blue")
    
    app = SerialApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
