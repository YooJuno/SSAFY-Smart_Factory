// static/app.js

// ─── 0) 서버 주소와 포트 전역 변수 ───
const SERVER_HOST = '192.168.110.114';
const SERVER_PORT = 65432;
const SERVER_URL  = `http://${SERVER_HOST}:${SERVER_PORT}`;

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

  let paused = false;

  document.addEventListener('click', (e) => {
    const ignoreIds = ['btn-send', 'btn-homing'];
    if (ignoreIds.some(id => e.target.id === id || e.target.closest(`#${id}`))) {
      return;
    }
    paused = true;
  });

  function showModal(text) {
    modalText.textContent = text;
    modalOverlay.style.opacity = '1';
    modalOverlay.style.visibility = 'visible';

    modalWrapper.style.opacity = '1';
    modalWrapper.style.visibility = 'visible';

    setTimeout(() => {
      hideModal();
    }, 1500);
  }

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

  function fetchStatusAndUpdate() {
    fetch(`${SERVER_URL}/status`)
      .then(response => {
        if (!response.ok) throw new Error(`HTTP 에러! status: ${response.status}`);
        return response.json();
      })
      .then(data => {
        if (paused) return;

        const jointRanges = [
          { lo: -135.0, hi: 125.0 },
          { lo: -5.0,   hi: 40.0 },
          { lo: -15.0,  hi: 80.0 },
          { lo: 110.0,  hi: 170.0 }
        ];

        const degValues = [
          Number(data.joints1),
          Number(data.joints2),
          Number(data.joints3),
          Number(data.joints4)
        ];

        degValues.forEach((deg, i) => {
          if (isNaN(deg)) deg = 0;
          const { lo, hi } = jointRanges[i];
          let p = (deg - lo) / (hi - lo) * 100.0;
          p = Math.max(0.0, Math.min(100.0, p));
          charts[i].data.datasets[0].data = [p, 100 - p];
          charts[i].update();
        });

        

        if (typeof data.x === 'number') {
          let xv = Math.round(data.x);
          xv = Math.max(Number(sliderX.min), Math.min(Number(sliderX.max), xv));
          sliderX.value = xv;
          valueX.textContent = xv;
        }
        if (typeof data.y === 'number') {
          let yv = Math.round(data.y);
          yv = Math.max(Number(sliderY.min), Math.min(Number(sliderY.max), yv));
          sliderY.value = yv;
          valueY.textContent = yv;
        }
        if (typeof data.z === 'number') {
          let zv = Math.round(data.z);
          zv = Math.max(Number(sliderZ.min), Math.min(Number(sliderZ.max), zv));
          sliderZ.value = zv;
          valueZ.textContent = zv;
        }

        if (typeof data.suction === 'number') {
          checkboxSuction.checked = (data.suction >= 0.5);
        }

        console.log(`${data.x} ${data.y} ${data.z}`);
      })
      .catch(err => {
        console.error('Status fetch 에러:', err);
      });
  }

  fetchStatusAndUpdate();
  setInterval(fetchStatusAndUpdate, 500);

  sliderX.addEventListener('input', () => valueX.textContent = sliderX.value);
  sliderY.addEventListener('input', () => valueY.textContent = sliderY.value);
  sliderZ.addEventListener('input', () => valueZ.textContent = sliderZ.value);

  window.addEventListener('DOMContentLoaded', () => {
    valueX.textContent = sliderX.value;
    valueY.textContent = sliderY.value;
    valueZ.textContent = sliderZ.value;
  });

  selectZ.addEventListener('change', () => {
    const choice = selectZ.value;
    if (choice === 'box') {
      sliderZ.value = -105;
    }
    else if(choice === 'panel') {
      sliderZ.value = -127;
    }
    else if(choice === 'conv-box') {
      sliderX.value = 272
      sliderY.value = 4
      sliderZ.value = -53;

      valueX.textContent = sliderX.value
      valueY.textContent = sliderY.value
    }
    else if(choice === 'conv-panel') {
      sliderX.value = 266
      sliderY.value = 4
      sliderZ.value = -72;

      valueX.textContent = sliderX.value
      valueY.textContent = sliderY.value
    }
    else {
      sliderZ.value = 50;
    }
    valueZ.textContent = sliderZ.value;
  });

  // homing 버튼 클릭 이벤트
  btnHoming.addEventListener('click', (e) => {
    e.stopPropagation();

    sliderX.value = 180;
    valueX.textContent = '180';

    sliderY.value = 0;
    valueY.textContent = '0';

    sliderZ.value = 50;
    valueZ.textContent = '50';

    fetch(`${SERVER_URL}/homing`, {
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
        paused = false;
      })
      .catch((error) => {
        console.error('Homing 요청 실패:', error);
        paused = false;
        hideModal();
      });
  });

  // ai 문장 입력 이벤트
  chatInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
      e.stopPropagation();
      const message = chatInput.value.trim();
      if (message.length === 0) {
        // 빈 문자열이라면 아무 동작도 하지 않고 그냥 리턴
        return;
      }

      // 예시: 다른 Flask 서버의 엔드포인트 (ex. localhost:5001/receive_chat) 로 POST 요청
      fetch(`${SERVER_URL}/receive_chat`, {
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
        paused = false;
      })
      .catch((err) => {
        console.error('채팅 전송 중 에러 발생:', err);
      });
    }
  });

  let clickXVal = null;
  let clickYVal = null;

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

    clickXVal = 320 - (320 + 100)*yRatio
    clickYVal = 300 - (300 + 280)*xRatio

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
  });  


  // send 버튼 클릭 이벤트
  btnSend.addEventListener('click', (e) => {
    e.stopPropagation();

    let xValToSend, yValToSend;

    if (clickXVal !== null && clickYVal !== null) {
      xValToSend = clickXVal;
      yValToSend = clickYVal;
    } else {
      // (2) 아니면 슬라이더 값을 사용
      xValToSend = Number(sliderX.value);
      yValToSend = Number(sliderY.value);
    }

    zVal = Number(sliderZ.value);
    suctionOn = checkboxSuction.checked;

    const payload = {
      x: xValToSend,
      y: yValToSend,
      z: zVal,
      suction: suctionOn
    };

    fetch(`${SERVER_URL}/send`, {
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
        clickXVal = null;
        clickYVal = null;
        overlay.innerHTML = '';
        paused = false;
      })
      .catch((error) => {
        console.error('전송 중 에러 발생:', error);
        hideModal();
      });
  });

});