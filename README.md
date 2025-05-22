# SSAFY 스마트팩토리 최종 프로젝트

## 📌 주제  
**웹 인터페이스 기반 Dobot 로봇팔 제어 자동화 시스템 구축**

---

## ⚙️ 프로젝트 구성 요소

### 1️⃣ Web Interface

#### Server
- *(내용 미작성: 서버 기능 및 기술스택 입력 가능)*

#### Client

- **[수신] 실시간 로봇 상태 및 영상 표시**  
  - 다음 4가지 항목을 실시간으로 수신하여 웹에 표시:
    - **관절 각도 (float):** Joint1, Joint2, Joint3, Joint4  
    - **말단 Tool 좌표 (int):** X, Y, Z (TCP 기준)  
    - **흡착기 상태 (bool):** Suction 작동 여부  
    - **카메라 이미지 (jpeg):** Realsense로부터 수신한 영상

- **[송신] 로봇 조작 기능**
  - 사용자의 조작 입력이 감지되는 순간, 로봇 상태 수신을 일시 중지
  - **이미지 클릭 조작:**  
    - 웹에 표시되는 **Coordinate Board** 이미지를 클릭해 목적지를 지정
  - **슬라이더 바 조작:**  
    - 하단의 X, Y, Z 바를 이용하여 말단 좌표를 미세 조정



### 2️⃣ ROS2  
Dobot, RealSense D435i, Server를 ROS2 System에서 관리
- **구현 환경 : Ubuntu 22.04 ROS2 Humble**

**Node info**
- YOLO_node (yolo_pkg)
- Perspective_Transformer_node (rs_pkg)
- Server_node (pjt_pkg)
 

**외부 launch 종속**
- realsense2_camera rs_launch.py
- dobot_bringup dobot_magician_control_system

### 3️⃣ RoboDK
로봇 모션 시뮬레이션
- **구현 환경 : Windows OS**
- 사용자의 클릭 위치를 로봇 기준 좌표계로 변환
- 경로 시각화 및 동작 예측



### 4️⃣ Conveyor Belt
물체의 위치에 따라 Dobot이 작업을 수행할 수 있도록 연동
- **구현 환경 : Raspberry Pi5**

---
