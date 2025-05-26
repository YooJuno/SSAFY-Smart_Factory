// static/app.js

document.addEventListener('DOMContentLoaded', () => {
  // ─── 1) 도넛 차트 인스턴스 배열 ───
  const charts = [];

  for (let i = 0; i < 4; i++) {
    const chartElement = document.getElementById(`chart-${i}`);
    chartElement.innerHTML = '';

    const canvas = document.createElement('canvas');
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    chartElement.appendChild(canvas);

    const ctx = canvas.getContext('2d');

    // 초기: 모두 0 (0, 100)
    const data = {
      labels: ['Value', 'Remaining'],
      datasets: [{
        data: [0, 100],
        backgroundColor: [
          'rgba(54, 162, 235, 0.7)',
          'rgba(200, 200, 200, 0.3)'
        ],
        borderColor: [
          'rgba(54, 162, 235, 1)',
          'rgba(200, 200, 200, 1)'
        ],
        borderWidth: 1
      }]
    };

    const options = {
      responsive: true,
      cutout: '70%',
      plugins: {
        legend: { display: false },
        title: { display: true, text: `Joint ${i}` }
      }
    };

    const chart = new Chart(ctx, {
      type: 'doughnut',
      data: data,
      options: options
    });
    charts.push(chart);
  }

  // ─── 2) Socket.IO 연결 ───
  // 서버가 같은 도메인/포트로 socketio.run(app) 했다고 가정 → io() 만 써도 됨
  const socket = io('http://192.168.110.114:65432');

  socket.on('connect', () => {
    console.log('Socket.IO connected, id =', socket.id);
  });

  socket.on('disconnect', () => {
    console.log('Socket.IO disconnected');
  });

  // 서버가 'joints_update' 이벤트와 함께 최신 joint 배열을 보냄
  socket.on('joints_update', (msg) => {
    if (!msg.joints || !Array.isArray(msg.joints) || msg.joints.length !== 4) {
      console.warn('잘못된 joints_update 메시지:', msg);
      return;
    }
    // msg.joints = [f0, f1, f2, f3]
    msg.joints.forEach((jointValue, i) => {
      let v = Number(jointValue);
      if (isNaN(v)) v = 0;
      if (v < 0) v = 0;
      if (v > 100) v = 100;

      charts[i].data.datasets[0].data = [v, 100 - v];
      charts[i].update();
    });
  });

  
  const sliderX = document.getElementById('slider-x');
  const valueX  = document.getElementById('value-x');

  const sliderY = document.getElementById('slider-y');
  const valueY  = document.getElementById('value-y');

  const sliderZ = document.getElementById('slider-z');
  const valueZ  = document.getElementById('value-z');
  const selectZ = document.getElementById('z-select');

  const checkboxSuction = document.getElementById('toggle');
  const btnSend = document.getElementById('btn-send');
  const btnHoming = document.getElementById('btn-homing');

  const modalOverlay = document.querySelector('.modal-overlay');
  const modalWrapper = document.querySelector('.modal-wrapper');
  const modalText    = document.querySelector('.modal-txt');

  const chatInput = document.getElementById('chat-txt');

  const imgElem = document.getElementById('video-section');
  const overlay = document.getElementById('click-overlay');

  const canvas = document.getElementById('video-canvas');
  const ctx = canvas.getContext("2d")

  socket.on("video_frame", (b64data) => {
    const img = new Image();
    img.src = "data:image/jpeg;base64," + b64data;
    img.onload = () => {
      ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    };
  });

  // const DOT_SIZE = 10;
  // const DOT_RADIUS = DOT_SIZE / 2;

  // ─── 모달을 화면에 띄우는 함수 ───
  function showModal(text) {
    modalText.textContent = text;
    modalOverlay.style.opacity = '1';
    modalOverlay.style.visibility = 'visible';

    modalWrapper.style.opacity = '1';
    modalWrapper.style.visibility = 'visible';
  }
  
  // ─── 모달을 숨기는 함수 ───
  function hideModal() {
    modalOverlay.style.opacity = '0';
    modalOverlay.style.transition = "opacity 1.2s ease";

    modalWrapper.style.opacity = '0';
    modalWrapper.style.transition = "opacity 1.2s ease";

    setTimeout(() => {
      modalWrapper.style.visibility = "hidden";
      modalOverlay.style.visibility = "hidden";
    }, 1200);
  }

  socket.on('homing_done', (msg) => {
    console.log('Received homing_done:', msg);
    hideModal();
  });

  // ─── 서버가 move 완료를 알리면 모달 숨기기 ───
  socket.on('move_done', (msg) => {
    console.log('Received move_done:', msg);
    hideModal();
  });


  sliderX.addEventListener('input', () => {
    valueX.textContent = sliderX.value;
  });
  sliderY.addEventListener('input', () => {
    valueY.textContent = sliderY.value;
  });
  sliderZ.addEventListener('input', () => {
    valueZ.textContent = sliderZ.value;
  });

  window.addEventListener('DOMContentLoaded', () => {
    valueX.textContent = sliderX.value;
    valueY.textContent = sliderY.value;
    valueZ.textContent = sliderZ.value;
  });

  selectZ.addEventListener('change', () => {
    const choice = selectZ.value;
    if (choice === 'box') {
      sliderZ.value = 45;
    } else if (choice === 'panel') {
      sliderZ.value = 25;
    } else {
      sliderZ.value = 50;
    }
    valueZ.textContent = sliderZ.value;
  });

  // send 버튼 클릭 이벤트
  btnSend.addEventListener('click', () => {
    const xVal = Number(sliderX.value);
    const yVal = Number(sliderY.value);
    const zVal = Number(sliderZ.value);
    const suctionOn = checkboxSuction.checked;

    const payload = {
      x: xVal,
      y: yVal,
      z: zVal,
      suction: suctionOn
    };

    fetch('http://192.168.110.114:65432/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    })
      .then((response) => {
        if (!response.ok) {
          throw new Error('서버 응답 에러: ' + response.status);
        }
        return response.json();
      })
      .then((data) => {
        showModal("Please wait while the robot is moving...");
        console.log('Flask /send 응답:', data);
      })
      .catch((error) => {
        console.error('전송 중 에러 발생:', error);
        hideModal();
      });
  });

  // homing 버튼 클릭 이벤트
  btnHoming.addEventListener('click', () => {
    sliderX.value = 250;
    valueX.textContent = '250';

    sliderY.value = 0;
    valueY.textContent = '0';

    sliderZ.value = 50;
    valueZ.textContent = '50';

    fetch('http://192.168.110.114:65432/homing', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ homing: true })
    })
      .then((response) => {
        if (!response.ok) {
          throw new Error('서버 응답 에러: ' + response.status);
        }
        return response.json();
      })
      .then((data) => {
        showModal("Waiting for homing...");
        console.log('Flask /homing 응답:', data);
      })
      .catch((error) => {
        console.error('Homing 요청 실패:', error);
        hideModal();
      });
  });

  // ai 문장 입력 이벤트
  chatInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
      const message = chatInput.value.trim();
      if (message.length === 0) {
        // 빈 문자열이라면 아무 동작도 하지 않고 그냥 리턴
        return;
      }

      // 예시: 다른 Flask 서버의 엔드포인트 (ex. localhost:5001/receive_chat) 로 POST 요청
      fetch('http://192.168.110.114:65432/receive_chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ message: message })
      })
      .then((response) => {
        if (!response.ok) {
          throw new Error('다른 서버 응답 에러: ' + response.status);
        }
        return response.json();
      })
      .then((data) => {
        console.log('ai Flask 서버 응답:', data);
        // 전송 후 입력창 초기화
        chatInput.value = '';
      })
      .catch((err) => {
        console.error('채팅 전송 중 에러 발생:', err);
      });
    }
  });

  imgElem.addEventListener('click', (event) => {
    // 1) offsetX, offsetY: 이미지 요소 기준(클릭된 픽셀 좌표) - 정확한 픽셀 위치
    const offsetX = event.offsetX;
    const offsetY = event.offsetY;

    // 2) 이미지가 화면에 렌더된 실제 크기(윤곽 박스 크기) 구하기
    const rect = imgElem.getBoundingClientRect();
    const imgWidth  = rect.width;
    const imgHeight = rect.height;

    // 3) 범위 검사(보호 코드)
    if (offsetX < 0 || offsetY < 0 || offsetX > imgWidth || offsetY > imgHeight) {
      return; // 클릭이 이미지 바깥에서 일어났다면 무시
    }

    // 4) 비율 계산 (0.0 ~ 1.0 사이 숫자)
    const xRatio = offsetX / imgWidth;
    const yRatio = offsetY / imgHeight;

    // 5) 이전 dot 제거(한 번만 보이게 하려면)
    overlay.innerHTML = '';

    // 6) 새로운 dot 생성
    const dot = document.createElement('div');
    dot.classList.add('click-dot');

    // 7) dot을 퍼센트 단위로 위치시키기
    dot.style.left = (xRatio * 100) + '%';
    dot.style.top  = (yRatio * 100) + '%';
    // (transform: translate(-50%, -50%)가 CSS에 적용되어 있으므로, dot의 중심이 이 퍼센트 지점에 맞춰집니다.)

    overlay.appendChild(dot);

    // 8) Flask 서버로 비율 정보 전송
    fetch('http://192.168.110.114:65432/image_click', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        x_ratio: xRatio,
        y_ratio: yRatio
      })
    })
    .then((resp) => {
      if (!resp.ok) throw new Error('이미지 클릭 전송 실패: ' + resp.status);
      return resp.json();
    })
    .then((data) => {
      showModal("Please wait while the robot is moving...");
      console.log('Flask /image_click 응답:', data);
    })
    .catch((err) => {
      console.error('이미지 클릭 전송 에러:', err);
    });
  });
});
