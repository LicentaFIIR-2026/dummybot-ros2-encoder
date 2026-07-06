# NAV RPi backup

Snapshot cod si configs de pe RPi NAV (compute node YOLO).

## Rol in sistem
- Hostname: nav-desktop
- IP eth (ethernet catre SAIM): 10.2.0.1
- Rol: doar YOLO inference (yolo26 / yolo_ros)
- Fara internet direct, doar prin ethernet catre SAIM (10.2.0.2)

## Structura
- `ros2_ws/src/` = workspace ROS2 principal
- `amrtcm_nav/src/` = workspace secundar de navigatie
- `yolo26/` = setup YOLO26
- `yolo_ros/` = wrapper YOLO ROS2
- `ros2_web/` = interfata web
- `fastdds_profile.xml` = config DDS pentru reducere latenta WiFi

## Excluse din backup (reproductibile)
- `build/`, `install/`, `log/` = artifacts colcon
- `deany/` (depth-anything-v2) = fork upstream nefolosit, cloneaza din https://github.com/DepthAnything/Depth-Anything-V2
- `ncnn/` = librarie, se compileaza local
- `depth_anything/` = weights nefolosite
- `snap/`, `.cache/`, `.config/` = irelevante
- `*.pt` weights = YOLO models, se descarca separat
