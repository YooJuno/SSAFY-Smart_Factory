document.addEventListener('DOMContentLoaded', () => {
  for (let i = 0; i < 4; i++) {
    const chartElement = document.getElementById(`chart-${i}`);
    chartElement.innerHTML = '';

    const canvas = document.createElement('canvas');
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    chartElement.appendChild(canvas);

    const ctx = canvas.getContext('2d');

    const data = {
      datasets: [{
        label: `Dataset ${i}`,
        data: [300, 50, 100],
        backgroundColor: [
          'rgba(255, 99, 132, 0.7)',
          'rgba(54, 162, 235, 0.7)',
          'rgba(255, 206, 86, 0.7)'
        ],
        borderColor: [
          'rgba(255, 99, 132, 1)',
          'rgba(54, 162, 235, 1)',
          'rgba(255, 206, 86, 1)'
        ],
        borderWidth: 1
      }]
    };

    const options = {
      responsive: true,
      plugins: {
        legend: {
          position: 'top',
        },
        title: {
          display: true,
          text: `Joint ${i}`,
        },
      },
    };

    new Chart(ctx, {
      type: 'doughnut',
      data: data,
      options: options
    });
  }
});
