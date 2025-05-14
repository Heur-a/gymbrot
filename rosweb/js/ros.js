/**
 * Application state and ROS configuration.
 * @typedef {Object} Data
 * @property {ROSLIB.Ros|null} ros - The ROS connection instance.
 * @property {string} rosbridge_address - The WebSocket URL to connect to ROSBridge.
 * @property {boolean} connected - Connection status to ROSBridge.
 */

/** 
 * @type {Data}
 * Holds the global ROS connection and configuration state.
 */
data = {
    ros: null,
    rosbridge_address: 'ws://127.0.0.1:9090/',
    connected: false
}

// DATOS DEL MAPA 
// Código de ayuda para cargar canvas y marcar robot en el mapa
// Cargar metadatos del mapa
let mapYamlUrl
let mapImageUrl
const mapYamlUrlSim = '../assets/gym_map_new.yaml'; // poner ubicación
const mapImageUrlSim = '../assets/gym_map_new.png'; // poner ubicación
const mapYamlUrlReal = '../assets/mapa_real.yaml'
const mapImageUrlReal = '../assets/mapa_real.png'

let mapInfo = null;
let canvas
let ctx
let image = new Image();
let robotPosition = { x: 0, y: 0 };
let aspectRatio = 1;

/**
 * Publisher for sending location goals to the robot.
 * @type {ROSLIB.Topic}
 */
let locationGoal = new ROSLIB.Topic({
    ros: data.ros,
    name: '/locationGoal',
    messageType: 'interfaces_gymbrot/msg/LocationGoal'
})

let irMaquina = new ROSLIB.Service({
    ros: data.ros,
    name: '/ir_maquina',
    serviceType: 'interfaces_gymbrot/srv/IrMaquina'
})

/**
 * Predefined coordinates for machine positions on the map.
 * @type {{x: number, y: number}}
 */
let machine_1 = { x: -3, y: -3.85 }

/** @type {{x: number, y: number}} */
let machine_2 = { x: -3, y: -1.0 }

/** @type {{x: number, y: number}} */
let machine_3 = { x: -3, y: 2.0 }

/**
 * Sends a goal position to the `/locationGoal` topic for the robot to navigate.
 *
 * @param {number} pos_X - Target X coordinate.
 * @param {number} pos_Y - Target Y coordinate.
 */
function moveToMachine(pos_X, pos_Y) {
    let request = new ROSLIB.ServiceRequest({
        x: pos_X,
        y: pos_Y
    })
    irMaquina.callService(request, (result) => {
        data.service_busy = false
        data.service_response = JSON.stringify(result)
        alert("Mensaje enviado correctamente")
    }, (error) => {
        data.service_busy = false
        data.service_response = JSON.stringify(result)
        alert("Mensaje enviado con errores")
    })
}

/**
 * Initializes a ROS connection, subscribes to `/odom`,
 * and sets up connection, error, and disconnection callbacks.
 */
async function connect() {
    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    })

    let odom = new ROSLIB.Topic({
        ros: data.ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
    })

    // Load map things
    canvas = document.getElementById("mapCanvas");
    ctx = canvas.getContext("2d");
    changeMap(false)

    odom.subscribe((message) => {
        robotPosition.x = message.pose.pose.position.x + 2.0;
        robotPosition.y = message.pose.pose.position.y;
        draw();  // redibuja mapa + posición del robot
    })

    // Re-create the locationGoal publisher with the active ROS connection
    locationGoal = new ROSLIB.Topic({
        ros: data.ros,
        name: '/locationGoal',
        messageType: 'interfaces_gymbrot/msg/LocationGoal'
    })

    // Conectar a servicio
    irMaquina = new ROSLIB.Service({
        ros: data.ros,
        name: '/ir_maquina',
        serviceType: 'interfaces_gymbrot/srv/IrMaquina'
    })

    // Connection event handlers
    data.ros.on("connection", () => {
        data.connected = true
        console.log("Conexion con ROSBridge correcta")
        alert("Conectado")
    })

    data.ros.on("error", (error) => {
        console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
        console.log(error)
        alert("Ha habido un error: " + error)
    })

    data.ros.on("close", () => {
        data.connected = false
        console.log("Conexion con ROSBridge cerrada")
        alert("Desconectado")
    })
}

/**
 * Event listener that runs once the document is fully loaded.
 * It connects to ROSBridge and registers local control functions.
 */
document.addEventListener('DOMContentLoaded', event => {

    connect()

    /**
     * Disconnects from the ROSBridge server and updates the connection state.
     */
    function disconnect() {
        data.ros.close()
        data.connected = false
        console.log('Clic en botón de desconexión')
    }

    /**
     * Publishes a Twist message to the `/cmd_vel` topic to move forward and rotate left.
     */
    function move() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0.1, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: -0.2 }
        })
        topic.publish(message)
    }

    /**
     * Publishes a Twist message with zero velocities to stop the robot's movement.
     */
    function stop() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        })
        topic.publish(message)
    }

    /**
     * Publishes a Twist message to the `/cmd_vel` topic to move forward and rotate right.
     */
    function move_inv() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0.1, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0.2 }
        })
        topic.publish(message)
    }

})

/**
 * Loads map metadata from a YAML file and displays the map image on the canvas.
 * This function fetches map information, updates the global mapInfo, and draws the image when loaded.
 *
 * @param {HTMLImageElement} image - The image element to display the map.
 * @param {string} mapYamlUrl - The URL of the map YAML metadata file.
 * @param {HTMLCanvasElement} canvas - The canvas element to draw the map on.
 * @param {CanvasRenderingContext2D} ctx - The 2D rendering context for the canvas.
 */
async function loadmap() {
    try {
        // Cargar metadatos del mapa
        const yamlResponse = await fetch(mapYamlUrl);
        const yamlText = await yamlResponse.text();
        mapInfo = jsyaml.load(yamlText);

        // Cargar imagen del mapa
        image = new Image();
        image.src = mapImageUrl;
        
        await new Promise((resolve, reject) => {
            image.onload = resolve;
            image.onerror = reject;
        });

        // Calcular aspect ratio y actualizar CSS
        aspectRatio = image.height / image.width;
        const aspectWrapper = document.getElementById('aspect-ratio-wrapper');
        aspectWrapper.style.paddingTop = `${aspectRatio * 100}%`;

        // Configurar canvas (buffer)
        canvas.width = image.width;  // Dimensiones reales
        canvas.height = image.height;
        
        // Redibujar
        draw();

    } catch (error) {
        console.error('Error loading map:', error);
        alert('Error al cargar el mapa');
    }
}

/**
 * Redraws the map and the robot's position on the canvas.
 * This function clears the canvas, draws the map image, and overlays the robot's current location.
 */
function draw() {
    if (!mapInfo || !image.complete) return;

    // Limpiar y dibujar imagen
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);

    // Obtener factores de escala reales
    const displayWidth = canvas.offsetWidth;
    const displayHeight = canvas.offsetHeight;
    const scaleX = displayWidth / canvas.width;
    const scaleY = displayHeight / canvas.height;

    // Dibujar robot (coordenadas ajustadas)
    const pixelX = (robotPosition.x - mapInfo.origin[0]) / mapInfo.resolution;
    const pixelY = (robotPosition.y - mapInfo.origin[1]) / mapInfo.resolution;
    
    
    ctx.beginPath();
    ctx.fillStyle = 'green';
    ctx.arc(pixelX, canvas.height - pixelY, 5, 0, 2 * Math.PI);
    ctx.fill();
}

/**
 * Switches between the real and simulated map sources and loads the selected map.
 * This function updates the map YAML and image URLs based on the input and triggers map loading.
 *
 * @param {boolean} real - If true, loads the real map; otherwise, loads the simulated map.
 */
function changeMap(real){
    if (real) {
        mapYamlUrl = mapYamlUrlReal
        mapImageUrl = mapImageUrlReal
        
    } else {
        mapYamlUrl = mapYamlUrlSim
        mapImageUrl = mapImageUrlSim
    }
    loadmap()
}
