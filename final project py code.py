"""
YOLO-based vehicle counting + UART density transmission to PIC16F1719 microcontroller.

UART protocol: C:<nn>\n  (nn = 00..99)
PIC microcontroller reads this value every 1 second and can extend red light duration accordingly.
"""

import os
# MPS'in desteklemediği işlemleri otomatik olarak CPU'da yapmasını sağlar
# IMPORTANT: Enables automatic CPU fallback for operations not supported by MPS (Apple Silicon GPU)
# Without this, the program would crash when MPS encounters unsupported PyTorch operations
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

import cv2
from ultralytics import YOLO
import torch
import serial
import serial.tools.list_ports
import time

# --- 1. AYARLAR VE PARAMETRELER ---
# --- 1. CONFIGURATION AND PARAMETERS ---
# CRITICAL: In top-down videos, vehicles appear small; therefore confidence threshold is lowered and image size is increased.
# Tepeden (top-down) videolarda araçlar küçük görünür; bu nedenle conf düşürülür ve imgsz yükseltilir.
CONF_THRESHOLD = 0.25  # Lower threshold: to avoid missing small objects (default is 0.5, we use 0.25 for better small vehicle detection)
IMG_SIZE = 960         # Higher resolution (1280 can also be tried based on MPS speed)
                       # Larger input size improves detection accuracy for small vehicles but increases processing time

# Model selection: prefer stronger model over smaller model for better accuracy.
# Model seçimi: küçük model yerine daha güçlü model tercih edilir.
# If file not found, continues with yolo11n.pt as fallback.
# Dosya yoksa yolo11n.pt ile devam eder.
MODEL_PATH = "yolo12m.pt"  # Medium-sized YOLO model - balance between accuracy and speed

# Frame upscaling to make small vehicles visible (pre-processing step)
# Küçük araçları görünür kılmak için frame büyütme
UPSCALE_IF_WIDTH_LT = 1280  # If video width is less than 1280 pixels, upscale the frame
UPSCALE_FACTOR = 1.5        # Upscaling multiplier - makes vehicles 50% larger, easier for YOLO to detect

# Minimum area (pixels^2) to filter out very small bounding boxes (likely false positives)
# Çok küçük kutuları filtrelemek için minimum alan (piksel^2)
MIN_BOX_AREA = 600  # Any detection box smaller than 600 pixels^2 is discarded as noise

# COCO classes: 2=car, 3=motorcycle, 5=bus, 7=truck
# COCO sınıfları: 2=car, 3=motorcycle, 5=bus, 7=truck
VEHICLE_CLASSES = {2, 3, 5, 7}  # Set of vehicle class IDs from COCO dataset - we only count motorized vehicles

# --- AYARLAR ---
# --- DETECTION LINE SETTINGS ---
# If video is 720p, 600-650 is ideal. 
# Eğer video 720p ise 600-650 idealdir. 
# If video is 1080p, you can set it to 900-950 levels.
# Eğer video 1080p ise 900-950 seviyelerine çekebilirsiniz.
LINE_Y = 600  # Line moved much lower by increasing from 450 to 600 - vehicles counted when crossing this Y-coordinate
              # CRITICAL: This is the virtual "counting line" - vehicles crossing this line are counted

# Detection region can be narrowed or expanded accordingly.
# Algılama bölgesini de buna göre daraltabilir veya genişletebilirsiniz.
DETECTION_MARGIN = 40  # Vertical margin around LINE_Y that defines detection window - prevents double counting

def find_mcp2221_port():
    """MCP2221 USB-UART köprüsünü otomatik bulur (Explorer 8)."""
    # Automatically finds MCP2221 USB-UART bridge (used on Explorer 8 board)
    # This is IMPORTANT: enables automatic hardware detection without manual port configuration
    ports = list(serial.tools.list_ports.comports())  # Get list of all available serial ports
    for p in ports:
        # Check if port description contains "MCP2221" - this is how Windows/Mac identifies the chip
        if "MCP2221" in (p.description or ""):
            return p.device  # Return the device path (e.g., "/dev/cu.usbmodem..." or "COM3")
    # fallback: first usbmodem
    # Fallback: if MCP2221 not found, try any USB modem port (common on Mac systems)
    for p in ports:
        if "usbmodem" in p.device:
            return p.device
    return None  # No serial port found - program continues in simulation mode

try:
    # Try to find port automatically, fallback to default path if auto-detection fails
    port = find_mcp2221_port() or '/dev/cu.usbserial-110'
    # Open serial connection: 9600 baud rate (matches PIC configuration), 0.1s timeout (non-blocking read)
    ser = serial.Serial(port, 9600, timeout=0.1)
    print(f">>> DONANIM MODU: PIC bağlantısı başarılı. Port: {port}")
    # >>> HARDWARE MODE: PIC connection successful. Port: {port}
except Exception as e:
    # If serial port not found or connection fails, continue in simulation mode (no hardware required)
    print(f">>> SİMÜLASYON MODU: Seri port bulunamadı. ({e})")
    # >>> SIMULATION MODE: Serial port not found. ({e})
    ser = None  # Set to None - all UART writes will be skipped (graceful degradation)

# --- 2. MODEL VE CİHAZ YAPILANDIRMASI ---
# --- 2. MODEL AND DEVICE CONFIGURATION ---
# Load YOLO neural network model - this is the CORE of our vehicle detection system
try:
    # Try to load medium model (better accuracy for small vehicles)
    model = YOLO(MODEL_PATH)
except Exception:
    # Fallback to nano model if medium model file not found (ensures program always runs)
    model = YOLO('yolo11n.pt')  # Nano model is always available in ultralytics package
# IMPORTANT: Use GPU acceleration if available (Apple Silicon MPS) - speeds up inference significantly
if torch.backends.mps.is_available():
    model.to('mps')  # Move model to MPS device - enables GPU acceleration on Apple Silicon (M1/M2/M3)
    print(">>> CİHAZ: Apple Silicon GPU (MPS) aktif.")
    # >>> DEVICE: Apple Silicon GPU (MPS) active.

video_path = "demo.mp4"  # Demo video path - can be changed to 0 for webcam or RTSP URL for IP camera
cap = cv2.VideoCapture(video_path)  # Open video file for processing

# Persistent vehicle IDs and counters for counting (tracking data structures)
# Sayım için kalıcı araç kimlikleri ve sayaçlar
vehicle_ids = set()  # Set of vehicle IDs that have already been counted - prevents double counting
counter = 0  # Total vehicle count - increments when new vehicle crosses counting line

# Instantaneous density to be sent to PIC (number of vehicles currently tracked on screen)
# PIC'e gönderilecek anlık yoğunluk (ekrandaki takip edilen araç sayısı)
current_density = 0  # Current number of vehicles visible in frame - different from counter (counter = passed, density = visible)

# 1Hz UART transmission timer (sends data every second)
# 1Hz UART gönderim zamanlayıcısı
last_send = time.time()  # Timestamp of last UART transmission - used to control 1Hz rate

print(f"Sistem başlatıldı. Güven Eşiği: {CONF_THRESHOLD}")
# System initialized. Confidence Threshold: {CONF_THRESHOLD}

while cap.isOpened():
    success, frame = cap.read()
    if not success: break

    # Upscale frame for small objects in top-down videos (CRITICAL for detection accuracy)
    # Top-down videoda küçük nesneler için frame'i büyüt
    if frame.shape[1] < UPSCALE_IF_WIDTH_LT:  # Check if frame width is less than threshold (1280 pixels)
        # Resize frame by UPSCALE_FACTOR in both dimensions - makes vehicles larger and easier to detect
        frame = cv2.resize(
            frame,
            None,  # None means calculate output size from fx/fy factors
            fx=UPSCALE_FACTOR,  # Horizontal scale factor: 1.5 = 150% width
            fy=UPSCALE_FACTOR,  # Vertical scale factor: 1.5 = 150% height
            interpolation=cv2.INTER_CUBIC  # Cubic interpolation - best quality for upscaling (but slower)
        )

    # --- 3. FİLTRELENMİŞ TAKİP (TRACKING) ---
    # --- 3. FILTERED TRACKING ---
    # This is the CORE: YOLO object detection + tracking in one call
    # conf: Filters out low-confidence detections (removes uncertain detections)
    # iou: Counts overlapping boxes as single vehicle (Non-Maximum Suppression threshold)
    results = model.track(
        frame,  # Input image - numpy array in BGR format
        persist=True,  # Enable tracking persistence - vehicle IDs maintained across frames (CRITICAL for counting)
        classes=list(VEHICLE_CLASSES),  # Only vehicle classes - filters out pedestrians, bicycles, etc.
        conf=CONF_THRESHOLD,  # Confidence threshold - detections below 0.25 are discarded
        iou=0.5,  # IoU threshold for NMS - higher value allows more overlapping boxes (standard is 0.5)
        imgsz=IMG_SIZE,  # Input image size for YOLO - frame resized to 960x960 before inference
        verbose=False  # Suppress YOLO progress output - reduces console spam
    )

    # Extract detection results - convert from PyTorch tensor to NumPy array for processing
    if results[0].boxes.id is not None:  # Check if tracking found any vehicles (ids exist)
        # CRITICAL: Convert tensors from GPU (MPS) to CPU, then to NumPy (required for OpenCV operations)
        boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding box coordinates: [x1, y1, x2, y2] format
        ids = results[0].boxes.id.cpu().numpy().astype(int)  # Tracking IDs: unique integer for each tracked vehicle
        confs = results[0].boxes.conf.cpu().numpy() # Güven oranlarını al - Get confidence scores (0.0 to 1.0)
        clss = results[0].boxes.cls.cpu().numpy().astype(int)  # Class IDs: vehicle type (2=car, 3=motorcycle, 5=bus, 7=truck)

        # Instantaneous density: number of vehicles on screen (only vehicle classes)
        # Anlık yoğunluk: ekrandaki araç sayısı (sadece vehicle classes)
        # IMPORTANT: This counts vehicles currently VISIBLE, not vehicles that PASSED (different from counter)
        current_density = int(sum(1 for c in clss if c in VEHICLE_CLASSES))  # Count vehicles by checking class membership

        for box, id, conf, cls_id in zip(boxes, ids, confs, clss):
            if cls_id not in VEHICLE_CLASSES:
                continue
            x1, y1, x2, y2 = box
            center_y = (y1 + y2) / 2
            box_area = (x2 - x1) * (y2 - y1)
            if box_area < MIN_BOX_AREA:
                continue
            
            # --- 4. AKILLI SAYIM MANTIĞI ---
            # --- 4. SMART COUNTING LOGIC ---
            # This is CRITICAL: prevents double counting by checking three conditions
            # 1. Koşul: Araç daha önce sayılmamış olmalı - Condition 1: Vehicle must not have been counted before
            # 2. Koşul: Aracın merkezi sayım çizgisi (LINE_Y) üzerinde olmalı - Condition 2: Vehicle center must be at counting line
            # 3. Koşul: Araç belirli bir dikey bölge içinde olmalı (ROI kısıtlaması) - Condition 3: Vehicle must be in detection window
            if id not in vehicle_ids:  # Check if this vehicle ID has been counted before (set lookup is O(1) fast)
                # Only count vehicle if its center is in the detection zone (between LINE_Y and LINE_Y + margin)
                if center_y > LINE_Y and center_y < (LINE_Y + DETECTION_MARGIN):  # Vehicle center must be in detection window
                    vehicle_ids.add(id)  # Mark this vehicle ID as counted - prevents double counting in next frames
                    counter += 1  # Increment total vehicle count - this is our main output metric
                    print(f"[OK] Araç Sayıldı! ID: {id} | Güven: {conf:.2f} | Toplam: {counter}")
                    # [OK] Vehicle Counted! ID: {id} | Confidence: {conf:.2f} | Total: {counter}
                    
                    # We no longer send individual '1' messages; we send 1Hz density messages instead.
                    # Artık tek tek '1' göndermiyoruz; 1Hz yoğunluk mesajı gönderiyoruz.
                    pass  # Placeholder - UART transmission happens separately every second (see below)

    # --- 4.5 UART: Her 1 saniyede bir yoğunluk gönder ---
    # --- 4.5 UART: Send density every 1 second ---
    # This is CRITICAL: send vehicle density to PIC microcontroller every second for adaptive traffic control
    # Format: C:07\n - Protocol format where XX is 2-digit density (00-99)
    now = time.time()  # Get current timestamp - used to check if 1 second has elapsed
    if ser and (now - last_send) >= 1.0:  # Check if serial connection exists AND 1 second has passed
        last_send = now  # Update timestamp - mark that we just sent (start counting next second)
        # Clamp density value to valid range: 0 to 99 (PIC protocol limitation - 2-digit format)
        value = max(0, min(current_density, 99))  # Ensures value is in [0, 99] range
        # Format message according to PIC protocol: "C:XX\n" where XX is zero-padded 2-digit number
        msg = f"C:{value:02d}\n".encode()  # 02d means zero-padded 2-digit integer, encode() converts string to bytes
        ser.write(msg)  # Send bytes over serial port to PIC microcontroller
        # Debug log
        print(f"[UART] sent C:{value:02d}")  # Console output confirms data was sent

    # --- 5. GÖRSELLEŞTİRME ---
    # --- 5. VISUALIZATION ---
    # Draw detection boxes and labels on frame - makes it easy to see what system is detecting
    annotated_frame = results[0].plot()  # YOLO built-in function: draws bounding boxes, labels, confidence scores
    
    # --- Görselleştirme (Demo Sunumu İçin) ---
    # --- Visualization (For Demo Presentation) ---
    # These visual elements are IMPORTANT for presentations - shows how the system works visually
    
    # 1. Ana Sayım Çizgisi (Kırmızı - Kalın) - Main Counting Line (Red - Thick)
    # Draw main counting line - vehicles crossing this line are counted
    cv2.line(annotated_frame, (0, LINE_Y), (int(frame.shape[1]), LINE_Y), (0, 0, 255), 3)
            # Parameters: image, start_point, end_point, color(B,G,R), thickness
            # (0, 0, 255) = red in BGR format, thickness 3 = 3 pixels wide (thick, visible line)

    # 2. Aktif Algılama Bölgesi Sınırı (Sarı - İnce) - Active Detection Zone Boundary (Yellow - Thin)
    # Draw detection zone boundary - vehicles must be between this line and main line to be counted
    # Araç bu iki çizgi arasına girdiğinde '1' sinyali tetiklenir.
    # Vehicle signal is triggered when it enters between these two lines.
    cv2.line(annotated_frame, (0, LINE_Y + DETECTION_MARGIN), (int(frame.shape[1]), LINE_Y + DETECTION_MARGIN), (0, 255, 255), 1)
            # Yellow line (0, 255, 255 in BGR) marks bottom of detection zone, thickness 1 = thin line

    # 3. Bölgeyi görsel olarak vurgulamak için (Opsiyonel: Şeffaf Katman)
    # To visually highlight the region (Optional: Transparent Layer)
    # Bu kısım hocanıza sunum yaparken "bakın bu bölgeyi tarıyorum" demek için iyidir.
    # This part is good for presentations: "look, I'm scanning this region"
    overlay = annotated_frame.copy()  # Create copy for overlay manipulation
    # Draw filled rectangle covering detection zone (yellow color, fully opaque in overlay)
    cv2.rectangle(overlay, (0, LINE_Y), (int(frame.shape[1]), LINE_Y + DETECTION_MARGIN), (0, 255, 255), -1)
            # -1 thickness = filled rectangle
    # Blend overlay with original: 20% overlay + 80% original = semi-transparent yellow highlight
    cv2.addWeighted(overlay, 0.2, annotated_frame, 0.8, 0, annotated_frame)
            # Creates subtle yellow tint over detection zone (professional visualization)

    # Display statistics text on frame - shows current count and configuration
    cv2.putText(annotated_frame, f"Arac Sayisi: {counter}", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # Vehicle Count: {counter} - large green text at top-left
    cv2.putText(annotated_frame, f"Güven Esigi: {CONF_THRESHOLD}", (20, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            # Confidence Threshold: {CONF_THRESHOLD} - smaller white text below count

    cv2.imshow("Vision-Enhanced Traffic Controller (YOLO11)", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"): break

cap.release()
cv2.destroyAllWindows()