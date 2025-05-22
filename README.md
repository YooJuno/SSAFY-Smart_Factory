# SSAFY 스마트팩토리 최종 프로젝트

## 📌 주제  
**웹 인터페이스 기반 DOBOT 로봇팔 제어 자동화 시스템 구축**
- 웹 페이지에 로봇 상태 확인 및 조작을 위한 인터페이스를 구현한다.
- 로봇의 상태창에는 Coordinate Board 실시간 촬영 영상을 송출한다.
- 또한 로봇의 현재 상태(TCP 좌표, 관절값)를 실시간으로 나타낸다.
- 사용자는 X, Y, Z bar를 조작하거나 Coordinate Board 이미지에서 target의 좌표를 클릭함으로써 로봇의 움직임을 제어한다.
- 또한 Suction option toggle을 통해 suction을 제어한다.
- panel의 위치를 입력으로 넣어주면 DOBOT이 panel을 흡착하여 컨베이어 벨트 위에 안착시킨다.
- 컨베이어 벨트 위에 올려진 panel을 YOLO를 이용해 객체 인식 및 색 검출을 진행한다. 
- 인식 결과에 따라 빨간색 panel은 1번으로, 나머지는 2번으로 servo motor를 이용해 분류한다.


## ⚙️ 프로젝트 구성 요소

### 1️⃣ Web Interface


**실시간 로봇 상태 및 영상 표시**  
  - **관절 각도 (float):** Joint1, Joint2, Joint3, Joint4  
  - **말단 Tool 좌표 (int):** X, Y, Z (TCP 기준)  
  - **흡착기 상태 (bool):** Suction 작동 여부  
  - **카메라 이미지 (jpeg):** Realsense로부터 수신한 영상. **Size : ???**

**DOBOT 조작**
- 사용자의 조작 입력이 감지되는 순간, 로봇 상태 수신을 일시 중지
- **이미지 클릭 조작:**  
  - 웹에 표시되는 **Coordinate Board** 이미지를 클릭해 목적지를 지정
  - 클릭된 위치에 Point로 위치를 부각시킨다.
- **슬라이더 바 조작:**  
  - 하단의 X, Y, Z 바를 이용하여 말단 좌표를 미세 조정

---

### 2️⃣ ROS2  
DOBOT, RealSense D435i, Server를 ROS2 System에서 관리

**구현 환경 : Ubuntu 22.04 ROS2 Humble**

**[ yolo_pkg ]**
  - **/yolo_node**
    - **/conv_space_image** topic을 구독하여 컨베이어 벨트 위의 객체를 인식 및 색 검출

**[ vision_pkg ]**
  - **/perspective_transformer_node**
    - realsense camera의 /camera/camera/color/image_raw 토픽을 구독하여 OpenCV Mat으로 변환
    - **Perspective Transformation**을 수행하여 원근을 제거한 Coordinate Board 이미지를 생성. 

**[ server_pkg ]**
  - **/server_node**

**[ dobot_pkg ]**
  - **/target_follower_node**
    - DOBOT의 초기화(Homing) 및 움직임 제어

**[ 외부 종속 ]**
- realsense2_camera **(package)**
  - /camera/camera **(node)**
- dobot_bringup **(package)**
  - /dobot_PTP_server **(node)**
  - /dobot_homing_srv **(node)**
  - /dobot_state_publisher **(node)**
  - /dobot_suction_cup_srv **(node)**

---

### 3️⃣ RoboDK
로봇 모션 시뮬레이션
**구현 환경** : Windows OS
- ROS2 Server_node와 TCP 통신 수행
- 컨베이어에서 panel이 검출되면 객체의 색 정보를 수신 
- 양품의 색인 경우 사전에 정의된 공정을 수행


---

### 4️⃣ Conveyor Belt
물체의 위치에 따라 DOBOT이 작업을 수행할 수 있도록 연동
**구현 환경** : Raspberry Pi5, Servo Motor, Step Motor, Switch
- TCP통신을 통해 YOLO node로부터 검출 결과를 수신
스텝 모터와 서보 모터를 결과에 따라 구동함으로써 panel 분류 수행
- 빨간색 : 1번으로 분류 (비정상)
- 나머지 : 2번으로 분류 (양품)

---
