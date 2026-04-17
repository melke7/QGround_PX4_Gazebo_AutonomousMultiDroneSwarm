import time
from pymavlink import mavutil

class DroneWorker:
    def __init__(self, sys_id, port, target):
        self.sys_id = sys_id
        self.port = port
        self.target = target  # (lat, lon, alt)
        self.master = None

    def log(self, msg):
        """Drone özelinde log mesajı yazdırır."""
        print(f"[Drone {self.sys_id}] {msg}", flush=True)

    def connect(self, timeout=30):
        """UDP portu üzerinden MAVLink bağlantısını kurar."""
        self.log(f"Port {self.port} üzerinden bağlanılıyor...")
        self.master = mavutil.mavlink_connection(f'udp:127.0.0.1:{self.port}')
        
        hb = self.master.wait_heartbeat(timeout=timeout)
        if not hb:
            raise RuntimeError(f"Hata: Port {self.port} üzerinde Heartbeat alınamadı!")
            
        self.log(f"Bağlantı Başarılı — sysid={self.master.target_system} compid={self.master.target_component}")

    def send_setpoint(self, lat, lon, alt):
        """Global koordinatlarla hedef noktası (setpoint) gönderir."""
        self.master.mav.set_position_target_global_int_send(
            0,                                  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,                 # Maske: Sadece Pozisyon aktif
            int(lat * 1e7),                     # Lat (e7 format)
            int(lon * 1e7),                     # Lon (e7 format)
            alt,                                # Alt (m)
            0, 0, 0,                            # Velocity
            0, 0, 0,                            # Acceleration
            0, 0                                # Yaw, Yaw Rate
        )

    def start_offboard(self):
        """Setpoint akışını başlatır ve PX4'ten OFFBOARD moduna geçmesini ister."""
        self.log("OFFBOARD için setpoint akışı başlatılıyor...")
        # PX4 mod değiştirmek için en az 2 saniye setpoint görmelidir
        for _ in range(30):
            self.send_setpoint(*self.target)
            time.sleep(0.1)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,      # Base Mode: 1 (Custom Mode aktif)
            6,      # Custom Mode: 6 (PX4 OFFBOARD)
            0, 0, 0, 0, 0
        )
        self.log("OFFBOARD modu talep edildi.")
        time.sleep(2)

    def arm(self, retries=5):
        """Motorları aktif eder (ARM). Başarısız olursa setpoint göndererek tekrar dener."""
        for attempt in range(1, retries + 1):
            self.log(f"ARM denemesi {attempt}/{retries}...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log("Motorlar aktif (ARMED).")
                return
            
            res = str(ack.result) if ack else "Zaman aşımı"
            self.log(f"ARM başarısız ({res}), tekrar deneniyor...")
            
            # Bağlantıyı canlı tutmak için setpoint göndermeye devam et
            for _ in range(20):
                self.send_setpoint(*self.target)
                time.sleep(0.1)
        
        self.log("UYARI: ARM onayı alınamadı, göreve devam ediliyor.")

    def get_position(self):
        """Drone'un anlık global koordinatlarını ve yüksekliğini alır."""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
        return None

    def distance_to_target(self, tlat, tlon, talt):
        """Hedef koordinatlara olan 3D mesafeyi (metre) hesaplar."""
        pos = self.get_position()
        if not pos:
            return float('inf')
        
        # Basit düzlem mesafe projeksiyonu
        dlat = (pos[0] - tlat) * 111320
        dlon = (pos[1] - tlon) * 111320 * 0.7  # Yaklaşık İzmir/Türkiye enlemi ölçeği
        dalt = pos[2] - talt
        return (dlat**2 + dlon**2 + dalt**2) ** 0.5

    def goto_waypoint(self, lat, lon, alt, tolerance=2.0, max_iter=300):
        """Belirlenen noktaya gidene kadar setpoint gönderir."""
        self.log(f"Hedefe gidiliyor: ({lat:.7f}, {lon:.7f}, {alt}m)")
        for i in range(max_iter):
            self.send_setpoint(lat, lon, alt)
            time.sleep(0.1)
            
            dist = self.distance_to_target(lat, lon, alt)
            if dist < tolerance:
                self.log(f"Hedefe varıldı (Mesafe: {dist:.2f}m)")
                return
            
            if i % 50 == 0:
                self.log(f"  Kalan Mesafe: {dist:.2f}m")
        
        self.log(f"Zaman aşımı! Hedefe tam varılamadı: {dist:.2f}m")

    def return_to_launch(self):
        """RTL (Eve Dön) komutu gönderir ve inişi bekler."""
        self.log("RTL (Eve Dön) komutu verildi.")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        self.log("İniş bekleniyor...")
        for _ in range(120):
            msg = self.master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
            if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.log("Drone yere indi.")
                return
            time.sleep(1)
        self.log("RTL Zaman aşımı, drone hala iniyor olabilir.")

    def run(self, waypoints=None):
        """Ana görev akışı."""
        self.connect()
        self.start_offboard()
        self.arm()

        self.log(f"Kalkış yüksekliğine çıkılıyor: {self.target[2]}m")
        self.goto_waypoint(*self.target)

        if waypoints:
            self.log(f"Görev başlatılıyor: {len(waypoints)} Waypoint")
            for idx, wp in enumerate(waypoints):
                self.log(f"WP {idx+1}/{len(waypoints)} gidiliyor...")
                self.goto_waypoint(*wp)
                time.sleep(1)
        else:
            self.log("Waypoint listesi boş, kısa süre havada kalınıyor.")
            time.sleep(2)

        self.return_to_launch()
        self.log("Görev tamamlandı.")
