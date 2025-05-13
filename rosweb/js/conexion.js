// Tema de ROS donde se publica la imagen procesada (por ejemplo, template matching o detección)
const cameraTopic = "/camera/processed_image";

// URL del servidor web_video_server que sirve la imagen como MJPEG stream
const cameraURL = `http://localhost:8080/stream?topic=${cameraTopic}`;

// Botón HTML para cambiar la vista a video real
const camaraButton = document.getElementById("cambiar_camara");

// Datos de conexión al ROSBridge
let data = {
    ros: null, // Objeto ROSLIB.Ros
    rosbridge_address: 'ws://127.0.0.1:9090/', // Dirección del servidor ROSBridge
    connected: false // Estado de conexión
};

/**
 * Función para cambiar dinámicamente el DOM e insertar el stream de la cámara.
 * También establece conexión con ROS si aún no está conectada.
 * 
 * Returns:
 *   void
 */
function cambiarAVideoReal() {
    // Asegurar conexión con ROSBridge
    if (!data.connected) {
        console.warn("[ROS] No conectado todavía. Llamando a connect()...");
        connect(); // Inicia conexión WebSocket
    }

    // Buscar el contenedor donde se colocará el video (usa clases de Tailwind con caracteres escapados)
    const contenedor = document.querySelector('.w-full.md\\:w-1\\/2.order-1.md\\:order-1');

    if (!contenedor) {
        console.error("❌ No se encontró el contenedor del video");
        return;
    }

    // Plantilla HTML para insertar el stream MJPEG y el indicador de grabación
    console.log(`🧪 Insertando imagen con stream desde: ${cameraURL}`);
    const nuevoContenido = `
      <div id="divCamera" class="relative w-full aspect-video bg-black rounded-lg shadow-lg overflow-hidden"> 
        <div style="border: 2px solid red;"> 
            <img id="cameraFeed" src="${cameraURL}" style="max-width: 100%;"> 
        </div> 
        <div class="absolute top-4 left-4 flex items-center space-x-2 text-white bg-black/50 px-2 py-1 rounded"> 
          <span class="w-3 h-3 bg-red-600 rounded-full inline-block animate-pulse"></span> 
          <span class="text-sm font-medium">REC</span> 
        </div> 
      </div> 
    `;

    // Insertar contenido HTML en el contenedor
    contenedor.innerHTML = nuevoContenido;

    // Forzar actualización del stream para evitar imágenes cacheadas
    updateCameraFeed();
}

/**
 * Establece una conexión WebSocket con el servidor ROSBridge y define un topic de prueba (/odom).
 * 
 * Returns:
 *   void
 */
function connect() {
    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    });

    // Suscribirse a un topic de prueba solo para confirmar conexión
    let topic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
    });

    // Callback en caso de conexión exitosa
    data.ros.on("connection", () => {
        data.connected = true;
        console.log("✅ Conexión con ROSBridge correcta");
    });

    // Callback para errores de conexión
    data.ros.on("error", (error) => {
        console.log("❌ Error al conectar con ROSBridge");
        console.log(error);
    });
}

/**
 * Fuerza la recarga de la imagen del stream para evitar el uso de caché del navegador.
 * 
 * Returns:
 *   void
 */
function updateCameraFeed() {
    const img = document.getElementById("cameraFeed");

    if (img) {
        // Agrega un timestamp al URL para evitar que el navegador use una versión en caché
        const timestamp = new Date().getTime();
        img.src = `${cameraURL}&_=${timestamp}`;
    }
}

// Espera a que el DOM esté listo y luego conecta el botón a la función principal
document.addEventListener("DOMContentLoaded", () => {
    const startStreamButton = document.getElementById("startStreamButton");

    if (startStreamButton) {
        startStreamButton.addEventListener("click", cambiarAVideoReal);
    }
});
