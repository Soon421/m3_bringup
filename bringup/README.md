# M3 Bringup

ISR-M3 로봇의 최상위 실행(bringup) ROS2 패키지. 센서 드라이버와 로봇 베이스(주행 모터)를 한 번에 띄워 자율주행 파이프라인의 하드웨어 인터페이스를 구성합니다.

## 레포 구성

```
m3_bringup/
├── bringup/    # 이 패키지 (m3_bringup): 최상위 launch 모음
├── robot/
│   └── m3_driver/                       # M3 베이스 드라이버 (시리얼 ↔ Arduino)
└── sensors/    # 소스로 포함된 센서 드라이버 (다운받은 원본 그대로)
    ├── velodyne/                            # Velodyne VLP16 LiDAR
    ├── Xsens_MTi_ROS_Driver_and_Ntrip_Client/  # Xsens MTi IMU (+ ntrip)
    └── novatel_oem7_driver/                 # Novatel OEM7 GPS
```

## 구성 요소

- **`bringup.launch.py`**: 센서 + 로봇 베이스 통합 실행
- **`sensors.launch.py`**: Velodyne VLP16 (driver/transform/laserscan), Xsens MTi IMU, Novatel GPS(`/gps/fix`로 remap), base→lidar/imu 정적 TF. 어떤 센서를 띄울지는 `config/sensors.yaml`의 `SENSORS:` 섹션에서 선택 (현재 lidar/imu on, gps off)
- **`robot.launch.py`**: `m3_driver` 베이스 드라이버 (`/dev/ttyACM0`, odometry 발행)

센서 파라미터(포트, LiDAR IP 등)는 `config/sensors.yaml`, 로봇 베이스 파라미터는 `robot/m3_driver/config/params.yaml`에서 수정합니다.

## 설치

### 사전 요구사항
- Ubuntu 22.04, ROS2 Humble

### 빌드

```bash
cd <your_workspace>
git clone <this-repo> src/m3_bringup

# 외부 의존성 설치 (libsdl2, libpcap, libpcl, mavros_msgs 등)
rosdep update
rosdep install --from-paths src --ignore-src -y

colcon build --packages-up-to m3_bringup
source install/setup.bash
```

### 장치 권한

```bash
sudo usermod -aG dialout $USER   # /dev/ttyACM0 (베이스), /dev/ttyUSB* (IMU/GPS), 재로그인 필요
```

## 사용법

```bash
ros2 launch m3_bringup bringup.launch.py    # 센서 + 로봇 베이스
ros2 launch m3_bringup sensors.launch.py    # 센서만
ros2 launch m3_bringup robot.launch.py      # 로봇 베이스만

ros2 launch m3_driver manual_launch.py  # 수동 조종 (조이스틱/키보드)
```
