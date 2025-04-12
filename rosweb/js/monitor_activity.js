const startButton = document.getElementById('startActivity');
const endButton = document.getElementById('endActivity');
const pauseButton = document.getElementById('pauseButton');
const resumeButton = document.getElementById('resumeButton');
const timer = document.getElementById('timer');
const minutesDisplay = document.getElementById('minutes');
const secondsDisplay = document.getElementById('seconds');
const camaraButton = document.getElementById("cambiar_camara");



let timeLeft = 5 * 60; // 5 minutos en segundos
let timerInterval;
let isPaused = false;
let data = {
    ros: null,
    rosbridge_address: 'ws://127.0.0.1:9090/',
    connected: false
  };

startButton.addEventListener('click', () => {
    startButton.textContent = 'En proceso...';
    startButton.disabled = true;
    endButton.disabled = false;
    endButton.classList.remove('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    endButton.classList.add('bg-red-500', 'text-white', 'hover:bg-red-600');
    timer.classList.remove('hidden');
    pauseButton.classList.remove('hidden');
    
    startTimer();
});

pauseButton.addEventListener('click', () => {
    if (!isPaused) {
        clearInterval(timerInterval);
        isPaused = true;
        pauseButton.classList.add('hidden');
        resumeButton.classList.remove('hidden');
    }
});

resumeButton.addEventListener('click', () => {
    if (isPaused) {
        startTimer();
        isPaused = false;
        resumeButton.classList.add('hidden');
        pauseButton.classList.remove('hidden');
    }
});

endButton.addEventListener('click', () => {
    clearInterval(timerInterval);
    resetTimer();
});
camaraButton.addEventListener("click", cambiarAVideoReal);

function startTimer() {
    timerInterval = setInterval(() => {
        const minutes = Math.floor(timeLeft / 60);
        const seconds = timeLeft % 60;
        
        minutesDisplay.textContent = minutes.toString().padStart(2, '0');
        secondsDisplay.textContent = seconds.toString().padStart(2, '0');
        
        if (timeLeft <= 0) {
            clearInterval(timerInterval);
            startButton.textContent = 'Tiempo completado';
            pauseButton.classList.add('hidden');
            resumeButton.classList.add('hidden');
        } else {
            timeLeft--;
        }
    }, 1000);
}

function resetTimer() {
    startButton.textContent = 'Comenzar actividad';
    startButton.disabled = false;
    endButton.disabled = true;
    endButton.classList.remove('bg-red-500', 'text-white', 'hover:bg-red-600');
    endButton.classList.add('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    timer.classList.add('hidden');
    pauseButton.classList.add('hidden');
    resumeButton.classList.add('hidden');
    timeLeft = 5 * 60;
    minutesDisplay.textContent = '05';
    secondsDisplay.textContent = '00';
    isPaused = false;
}
function cambiarAVideoReal() {

    if (!data.connected) {
        console.warn("[ROS] No conectado todavía. Llamando a connect()...");
        connect(); // Establece conexión si aún no está
    }

    const contenedor = document.querySelector('.w-full.md\\:w-1\\/2.order-1.md\\:order-1');

    if (!contenedor) {
        console.error("❌ No se encontró el contenedor del video");
        return;
    }

    // Este es el topic que se está utilizando para el stream de la cámara
    const cameraTopic = "/camera/image_raw";
    const cameraURL = `http://localhost:8080/stream?topic=${cameraTopic}`;

    console.log(`🧪 Insertando imagen con stream desde: ${cameraURL}`);

    const nuevoContenido = `
      <div id="divCamera" class="relative w-full aspect-video bg-black rounded-lg shadow-lg overflow-hidden">
        <div style="border: 2px solid red;">
            <img id="cameraFeed" src="http://localhost:8080/stream?topic=/camera/image_raw" style="max-width: 100%;">
        </div>

        <div class="absolute top-4 left-4 flex items-center space-x-2 text-white bg-black/50 px-2 py-1 rounded">
          <span class="w-3 h-3 bg-red-600 rounded-full inline-block animate-pulse"></span>
          <span class="text-sm font-medium">REC</span>
        </div>
      </div>
    `;

    contenedor.innerHTML = nuevoContenido;

    updateCameraFeed(); // Actualiza imagen (fuerza recarga)
}

function connect(){
    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    })
    let topic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
    })
    // Define callbacks
    data.ros.on("connection", () => {
        data.connected = true
        console.log("Conexion con ROSBridge correcta")
    })
    data.ros.on("error", (error) => {
        console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
        console.log(error)
    })
  }
  function updateCameraFeed() {
    const img = document.getElementById("cameraFeed");
    const timestamp = new Date().getTime(); // Evita caché agregando un timestamp
    img.src = `http://localhost:8080/stream?topic=/camera/image_raw`;
  }

  