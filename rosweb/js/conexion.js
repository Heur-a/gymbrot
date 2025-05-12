const cameraTopic = "/camera/processed_image"; 
const cameraURL = `http://localhost:8080/stream?topic=${cameraTopic}`; 
const camaraButton = document.getElementById("cambiar_camara"); 
 
let data = { 
    ros: null, 
    rosbridge_address: 'ws://127.0.0.1:9090/', 
    connected: false 
  };  
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
    
 
    console.log(`🧪 Insertando imagen con stream desde: ${cameraURL}`); 
 
    const nuevoContenido = ` 
      <div id="divCamera" class="relative w-full aspect-video bg-black rounded-lg shadow-lg overflow-hidden"> 
        <div style="border: 2px solid red;"> 
            <img id="cameraFeed" src="http://localhost:8080/stream?topic=${cameraTopic}" style="max-width: 100%;"> 
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
    img.src = `http://localhost:8080/stream?topic=${cameraTopic}`; 
  } 
 
  document.addEventListener("DOMContentLoaded", () => {
    const startStreamButton = document.getElementById("startStreamButton");
    if (startStreamButton) {
        startStreamButton.addEventListener("click", cambiarAVideoReal);
    }
});
