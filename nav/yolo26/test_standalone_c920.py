#!/usr/bin/env python3
# test_standalone_c920.py - Benchmark YOLO26 NCNN cu Logitech C920

import ncnn
import cv2
import numpy as np
import time
from collections import deque

class C920Camera:
    """Wrapper optimizat pentru Logitech C920 pe RPi5"""
    
    def __init__(self, device_id=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            # Fallback fără V4L2 explicit
            self.cap = cv2.VideoCapture(device_id)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Nu pot deschide camera {device_id}")
        
        # Forțează MJPEG (hardware decode în cameră, reduce CPU load)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        
        # Rezoluție - folosim 640x360 (16:9 nativ) sau 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Reduce buffer-ul pentru latență minimă
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Verifică setările reale
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((actual_fourcc >> 8*i) & 0xFF) for i in range(4)])
        
        print(f"Camera configurată:")
        print(f"  Rezoluție: {actual_w}x{actual_h}")
        print(f"  FPS: {actual_fps}")
        print(f"  Format: {fourcc_str}")
    
    def read(self):
        return self.cap.read()
    
    def release(self):
        self.cap.release()


def letterbox(img, new_shape=(416, 416), color=(114, 114, 114)):
    """
    Resize păstrând aspect ratio + padding (letterbox).
    C920 e 16:9, YOLO vrea pătrat - asta evită distorsiunea.
    """
    shape = img.shape[:2]  # [height, width]
    
    # Calculează ratio
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    
    # Dimensiuni noi
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    
    # Padding necesar
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    
    # Împarte padding-ul egal sus/jos, stânga/dreapta
    dw /= 2
    dh /= 2
    
    # Resize
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    
    # Adaugă padding
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                              cv2.BORDER_CONSTANT, value=color)
    
    return img, r, (dw, dh)


def load_ncnn_model(model_path, num_threads=4):
    """Încarcă modelul NCNN optimizat pentru RPi5"""
    net = ncnn.Net()
    
    # Opțiuni pentru RPi5 (fără GPU, doar CPU)
    net.opt.use_vulkan_compute = False
    net.opt.num_threads = num_threads
    net.opt.use_fp16_packed = False  # FP32 e mai stabil pe RPi5
    net.opt.use_fp16_storage = False
    net.opt.use_fp16_arithmetic = False
    net.opt.use_packing_layout = True
    net.opt.lightmode = True
    
    # Încarcă model
    param_path = f"{model_path}/model.ncnn.param"
    bin_path = f"{model_path}/model.ncnn.bin"
    
    ret_param = net.load_param(param_path)
    ret_bin = net.load_model(bin_path)
    
    if ret_param != 0 or ret_bin != 0:
        raise RuntimeError(f"Eroare la încărcare model: param={ret_param}, bin={ret_bin}")
    
    print(f"Model încărcat: {model_path}")
    
    return net


def inference_ncnn(net, img_preprocessed, input_size):
    """Rulează inferența NCNN"""
    
    # Conversie la NCNN Mat
    mat_in = ncnn.Mat.from_pixels(
        img_preprocessed,
        ncnn.Mat.PixelType.PIXEL_BGR2RGB,
        input_size, input_size
    )
    
    # Normalizare YOLO standard (0-255 → 0-1)
    mat_in.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    
    # Crează extractor și rulează
    ex = net.create_extractor()
    
    # Numele input-ului - verifică în model.ncnn.param prima linie "Input"
    ex.input("in0", mat_in)
    
    # Extrage output - verifică ultima linie din .param
    ret, mat_out = ex.extract("out0")
    
    if ret != 0:
        print(f"Warning: extract returned {ret}")
        return None
    
    return np.array(mat_out)


def benchmark(model_path, input_size=416, duration=30, camera_res=(640, 480)):
    """
    Benchmark complet YOLO26 NCNN cu C920
    """
    print("="*60)
    print("YOLO26 NCNN Benchmark - Logitech C920 pe RPi5")
    print("="*60)
    
    # Inițializare
    try:
        camera = C920Camera(device_id=0, width=camera_res[0], height=camera_res[1])
    except RuntimeError as e:
        print(f"EROARE cameră: {e}")
        print("Verifică: ls /dev/video*")
        return
    
    try:
        net = load_ncnn_model(model_path)
    except RuntimeError as e:
        print(f"EROARE model: {e}")
        camera.release()
        return
    
    print(f"Input YOLO: {input_size}x{input_size}")
    print(f"Durată test: {duration}s")
    print("-"*60)
    
    # Metrici
    inference_times = []
    total_times = []  # Include capture + preprocess + inference
    fps_window = deque(maxlen=30)  # Pentru FPS smoothed
    
    frame_count = 0
    start_time = time.time()
    
    # Warmup - primele inferențe sunt mai lente
    print("Warmup (10 frames)...")
    for _ in range(10):
        ret, frame = camera.read()
        if ret:
            img_lb, _, _ = letterbox(frame, (input_size, input_size))
            _ = inference_ncnn(net, img_lb, input_size)
    print("Warmup complet.\n")
    
    print("Benchmark în curs... (apasă 'q' pentru stop)")
    
    while time.time() - start_time < duration:
        t_total_start = time.perf_counter()
        
        # Capture
        ret, frame = camera.read()
        if not ret:
            print("Frame drop!")
            continue
        
        # Preprocess (letterbox pentru aspect ratio corect)
        img_letterbox, ratio, pad = letterbox(frame, (input_size, input_size))
        
        # Inferență
        t_inf_start = time.perf_counter()
        output = inference_ncnn(net, img_letterbox, input_size)
        t_inf_end = time.perf_counter()
        
        t_total_end = time.perf_counter()
        
        # Metrici
        inf_time = t_inf_end - t_inf_start
        total_time = t_total_end - t_total_start
        
        inference_times.append(inf_time)
        total_times.append(total_time)
        
        fps_instant = 1.0 / total_time
        fps_window.append(fps_instant)
        fps_smooth = sum(fps_window) / len(fps_window)
        
        frame_count += 1
        
        # Vizualizare
        display_frame = frame.copy()
        
        # Info overlay
        cv2.putText(display_frame, f"FPS: {fps_smooth:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Inf: {inf_time*1000:.1f}ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Total: {total_time*1000:.1f}ms", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        if output is not None:
            cv2.putText(display_frame, f"Output shape: {output.shape}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        
        cv2.imshow("YOLO26 Benchmark", display_frame)
        
        # Afișează și imaginea letterbox (debug)
        cv2.imshow("Letterbox Input", img_letterbox)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\nOprit de utilizator.")
            break
    
    # Cleanup
    camera.release()
    cv2.destroyAllWindows()
    
    # Raport final
    elapsed = time.time() - start_time
    inf_times = np.array(inference_times)
    tot_times = np.array(total_times)
    
    print("\n" + "="*60)
    print("REZULTATE FINALE")
    print("="*60)
    print(f"Frames procesate:     {frame_count}")
    print(f"Timp total:           {elapsed:.1f}s")
    print(f"FPS efectiv:          {frame_count/elapsed:.1f}")
    print("-"*60)
    print("INFERENȚĂ (doar NCNN):")
    print(f"  Medie:              {inf_times.mean()*1000:.1f} ms")
    print(f"  Std dev:            {inf_times.std()*1000:.1f} ms")
    print(f"  Min/Max:            {inf_times.min()*1000:.1f} / {inf_times.max()*1000:.1f} ms")
    print(f"  FPS teoretic:       {1/inf_times.mean():.1f}")
    print("-"*60)
    print("PIPELINE COMPLET (capture + preprocess + inference):")
    print(f"  Medie:              {tot_times.mean()*1000:.1f} ms")
    print(f"  Std dev:            {tot_times.std()*1000:.1f} ms")
    print(f"  Min/Max:            {tot_times.min()*1000:.1f} / {tot_times.max()*1000:.1f} ms")
    print(f"  FPS real:           {1/tot_times.mean():.1f}")
    print("="*60)
    
    # Verdict
    fps_real = 1/tot_times.mean()
    print("\nVERDICT:")
    if fps_real >= 8:
        print(f"  ✓ EXCELENT ({fps_real:.1f} FPS) - Gata pentru integrare ROS2")
    elif fps_real >= 5:
        print(f"  ✓ ACCEPTABIL ({fps_real:.1f} FPS) - Merge pentru navigație")
    elif fps_real >= 3:
        print(f"  ~ MARGINAL ({fps_real:.1f} FPS) - Consideră reducerea rezoluției")
    else:
        print(f"  ✗ INSUFICIENT ({fps_real:.1f} FPS) - Nu recomand integrare")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Benchmark YOLO26 NCNN')
    parser.add_argument('--model', type=str, default='yolo26n_ncnn_model',
                        help='Calea către modelul NCNN exportat')
    parser.add_argument('--size', type=int, default=416,
                        help='Dimensiune input YOLO (default: 416)')
    parser.add_argument('--duration', type=int, default=30,
                        help='Durată benchmark în secunde (default: 30)')
    parser.add_argument('--cam-width', type=int, default=640,
                        help='Lățime captură cameră (default: 640)')
    parser.add_argument('--cam-height', type=int, default=480,
                        help='Înălțime captură cameră (default: 480)')
    
    args = parser.parse_args()
    
    benchmark(
        model_path=args.model,
        input_size=args.size,
        duration=args.duration,
        camera_res=(args.cam_width, args.cam_height)
    )