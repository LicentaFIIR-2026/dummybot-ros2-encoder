#!/usr/bin/env python3
# test_headless.py - Benchmark FĂRĂ GUI (pentru SSH)

import ncnn
import cv2
import numpy as np
import time
from collections import deque

class C920Camera:
    def __init__(self, device_id=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(device_id)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Nu pot deschide camera {device_id}")
        
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera: {actual_w}x{actual_h} @ {actual_fps}fps")
    
    def read(self):
        return self.cap.read()
    
    def release(self):
        self.cap.release()


def letterbox(img, new_shape=(416, 416), color=(114, 114, 114)):
    shape = img.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                              cv2.BORDER_CONSTANT, value=color)
    return img


def load_ncnn_model(model_path, num_threads=4):
    net = ncnn.Net()
    net.opt.use_vulkan_compute = False
    net.opt.num_threads = num_threads
    net.opt.lightmode = True
    
    net.load_param(f"{model_path}/model.ncnn.param")
    net.load_model(f"{model_path}/model.ncnn.bin")
    
    print(f"Model: {model_path}")
    return net


def inference_ncnn(net, img_preprocessed, input_size):
    mat_in = ncnn.Mat.from_pixels(
        img_preprocessed,
        ncnn.Mat.PixelType.PIXEL_BGR2RGB,
        input_size, input_size
    )
    mat_in.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
    
    ex = net.create_extractor()
    ex.input("in0", mat_in)
    ret, mat_out = ex.extract("out0")
    
    return np.array(mat_out) if ret == 0 else None


def benchmark(model_path, input_size=416, duration=30):
    print("="*60)
    print("YOLO26 NCNN Benchmark (Headless)")
    print("="*60)
    
    camera = C920Camera()
    net = load_ncnn_model(model_path)
    print(f"Input size: {input_size}x{input_size}")
    print(f"Durată: {duration}s")
    print("-"*60)
    
    # Warmup
    print("Warmup...", end=" ", flush=True)
    for _ in range(10):
        ret, frame = camera.read()
        if ret:
            img_lb = letterbox(frame, (input_size, input_size))
            _ = inference_ncnn(net, img_lb, input_size)
    print("OK")
    
    # Benchmark
    inference_times = []
    total_times = []
    frame_count = 0
    start_time = time.time()
    last_print = start_time
    
    print("\nBenchmark rulează...")
    print("Frame | Inf(ms) | Total(ms) | FPS")
    print("-"*40)
    
    while time.time() - start_time < duration:
        t_total_start = time.perf_counter()
        
        ret, frame = camera.read()
        if not ret:
            continue
        
        img_lb = letterbox(frame, (input_size, input_size))
        
        t_inf_start = time.perf_counter()
        output = inference_ncnn(net, img_lb, input_size)
        t_inf_end = time.perf_counter()
        
        t_total_end = time.perf_counter()
        
        inf_time = t_inf_end - t_inf_start
        total_time = t_total_end - t_total_start
        
        inference_times.append(inf_time)
        total_times.append(total_time)
        frame_count += 1
        
        # Print progress la fiecare 2 secunde
        if time.time() - last_print >= 2.0:
            fps = 1.0 / total_time
            print(f"{frame_count:5d} | {inf_time*1000:7.1f} | {total_time*1000:9.1f} | {fps:.1f}")
            last_print = time.time()
    
    camera.release()
    
    # Rezultate
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
    print("PIPELINE COMPLET:")
    print(f"  Medie:              {tot_times.mean()*1000:.1f} ms")
    print(f"  FPS real:           {1/tot_times.mean():.1f}")
    print("="*60)
    
    fps_real = 1/tot_times.mean()
    print("\nVERDICT:")
    if fps_real >= 8:
        print(f"  ✓ EXCELENT ({fps_real:.1f} FPS)")
    elif fps_real >= 5:
        print(f"  ✓ ACCEPTABIL ({fps_real:.1f} FPS)")
    elif fps_real >= 3:
        print(f"  ~ MARGINAL ({fps_real:.1f} FPS)")
    else:
        print(f"  ✗ INSUFICIENT ({fps_real:.1f} FPS)")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='yolo26n_ncnn_model')
    parser.add_argument('--size', type=int, default=416)
    parser.add_argument('--duration', type=int, default=30)
    args = parser.parse_args()
    
    benchmark(args.model, args.size, args.duration)
