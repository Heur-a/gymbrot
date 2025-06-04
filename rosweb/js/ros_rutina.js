/** 
 * @type {Data}
 * Holds the global ROS connection and configuration state.
 */

// Iniciar variables
let real = false
let susc_pos = null
let esperando_robot = false



let selector


data = {
    ros: null,
    rosbridge_address: 'ws://127.0.0.1:9090/',
    connected: false
}

let liberarService

function liberarRobot(cond) {
    let request = new ROSLIB.ServiceRequest({
        lib: cond
    })
    liberarService.callService(request, (result) => {
        data.service_busy = false
        data.service_response = JSON.stringify(result)
        alert("Mensaje enviado correctamente")
        if (data.service_response.codigo == 0) {
            esperando_robot = false
        }
    }, (error) => {
        esperando_robot = false
        data.service_busy = false
        data.service_response = JSON.stringify(result)
        alert("Mensaje enviado con errores" + error)
    })
}

function checkRobotHaLlegadoMaquina(maquina, x, y, epsilon) {
    if (
        Math.abs(maquina.x - x) < epsilon
        &&
        Math.abs(maquina.y - y) < epsilon
    ) {
        return true
    }
    return false
}


async function connect_rutina() {
    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    })

    liberarService = new ROSLIB.Service({
        ros: data.ros,
        name: '/liberar',
        serviceType: 'interfaces_gymbrot/srv/LiberarRobotActividad'
    })


    if (real) {
        susc_pos = new ROSLIB.Topic({
            ros: data.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        })
    } else {
        susc_pos = new ROSLIB.Topic({
            ros: data.ros,
            name: '/amcl_pose', // Topic correcte
            messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped' // Tipus de missatge
        });
    }

    changeMachines(real)


    susc_pos.subscribe((message) => {
        robotPosition.x = message.pose.pose.position.x;
        robotPosition.y = message.pose.pose.position.y;
        console.log("X: " + message.pose.pose.position.x + ", Y: " + message.pose.pose.position.y)
    })

}


document.addEventListener('DOMContentLoaded', event => {

    connect_rutina()

    selector = document.getElementById('mapSelector')

    selector.addEventListener('change', event => {
    switch (selector.value) {
        case 'map_siml':
            real = false
            break;
        case 'map_real':
            real = true
            break;
        default:
            alert("algo ha pasado con el selector")
            return
            break;
    }
    changeMachines(real);
    return;
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

})

