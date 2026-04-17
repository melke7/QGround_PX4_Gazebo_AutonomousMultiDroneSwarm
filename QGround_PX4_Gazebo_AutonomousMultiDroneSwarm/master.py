import multiprocessing
import time
from drone_worker import DroneWorker

def drone_process(sys_id, port, target, waypoints):
    """
    Her bir drone için ayrı bir process içinde çalışacak fonksiyon.
    """
    try:
        drone = DroneWorker(sys_id, port, target)
        drone.run(waypoints)
    except RuntimeError as e:
        print(f"[Drone sys_id={sys_id}] BAĞLANTI HATASI: {e}", flush=True)
    except Exception as e:
        print(f"[Drone sys_id={sys_id}] BEKLENMEDİK HATA: {e}", flush=True)

if __name__ == "__main__":
    # Drone konfigürasyonları: Port 14541'den başlıyor (droneN_wp.plan -> port 14540+N)
    drone_configs = [
        {
            "label": 1, "port": 14541,
            "target": (37.4135598, -121.9965541, 30),
            "waypoints": [(37.4115995, -121.9984912, 25), (37.4113532, -121.9991161, 20), (37.4115774, -121.9983899, 30)]
        },
        {
            "label": 2, "port": 14542,
            "target": (37.4135887, -121.9965613, 30),
            "waypoints": [(37.4117264, -121.9986419, 30), (37.4114151, -121.9994757, 20), (37.4118546, -121.9982871, 30)]
        },
        {
            "label": 3, "port": 14543,
            "target": (37.4136158, -121.9965621, 30),
            "waypoints": [(37.4118938, -121.9987500, 30), (37.4115433, -121.9996041, 20), (37.4118632, -121.9987334, 30)]
        },
        {
            "label": 4, "port": 14544,
            "target": (37.4136422, -121.9965608, 30),
            "waypoints": [(37.4119337, -121.9991556, 30), (37.4117172, -121.9997011, 20), (37.4119215, -121.9991519, 30)]
        },
        {
            "label": 5, "port": 14545,
            "target": (37.4136666, -121.9965597, 30),
            "waypoints": [(37.4127145, -121.9996661, 30), (37.4125001, -122.0002039, 20), (37.4126905, -121.9996580, 30)]
        },
        {
            "label": 6, "port": 14546,
            "target": (37.4136963, -121.9965610, 30),
            "waypoints": [(37.4129857, -121.9994738, 30), (37.4126677, -122.0002763, 20), (37.4129660, -121.9994791, 30)]
        },
        {
            "label": 7, "port": 14547,
            "target": (37.4137258, -121.9965617, 30),
            "waypoints": [(37.4131513, -121.9995559, 30), (37.4128239, -122.0003972, 20), (37.4131329, -121.9995510, 30)]
        }
    ]

    processes = []

    # Drone'ları başlatma
    for cfg in drone_configs:
        print(f"[MASTER] Drone {cfg['label']} başlatılıyor... Port: {cfg['port']}", flush=True)
        p = multiprocessing.Process(
            target=drone_process,
            args=(cfg["label"], cfg["port"], cfg["target"], cfg["waypoints"]),
            name=f"drone-{cfg['label']}"
        )
        p.start()
        processes.append((cfg["label"], p))
        time.sleep(1)  # MAVLink çakışmalarını önlemek için kısa bekleme

    print(f"[MASTER] Toplam {len(processes)} drone havada.", flush=True)

    try:
        # İzleme döngüsü
        while True:
            all_done = True
            for label, p in processes:
                if p.is_alive():
                    all_done = False
                elif p.exitcode not in (None, 0):
                    print(f"[MASTER] Drone {label} hata ile sonlandı (Exit Code: {p.exitcode})", flush=True)
            
            if all_done:
                print("[MASTER] Tüm drone'lar görevlerini tamamladı.", flush=True)
                break
            time.sleep(5)

    except KeyboardInterrupt:
        print("\n[MASTER] Kapatma sinyali alındı, tüm drone'lar durduruluyor...", flush=True)
        for _, p in processes:
            p.terminate()
        for _, p in processes:
            p.join(timeout=5)
        print("[MASTER] Tüm süreçler kapatıldı.", flush=True)
