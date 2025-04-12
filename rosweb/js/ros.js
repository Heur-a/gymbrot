data = {
    // ros connection
    ros: null,
    rosbridge_address: 'ws://127.0.0.1:9090/',
    connected: false
}

let locationGoal = new ROSLIB.Topic({
    ros: data.ros,
    name: '/locationGoal',
    messageType: 'interfaces_gymbrot/msg/LocationGoal'
})



let machine_1 = { x: -3, y: -3.85 }
let machine_2 = { x: -3, y: -1.0 }
let machine_3 = {x: -3, y: 2.0}

function moveToMachine(pos_X, pos_Y) {
    let message = new ROSLIB.Message({
        x: pos_X,
        y: pos_Y
    })
    locationGoal.publish(message)
}

function connect() {

    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    })

    let odom = new ROSLIB.Topic({
        ros: data.ros,
        name: '/odom',
        messageType: 'nav_msgs/msg/Odometry'
    })

    odom.subscribe((message) => {
        //   data.position = message.pose.pose.position
        // 	document.getElementById("pos_x").innerHTML = data.position.x.toFixed(2)
        // 	document.getElementById("pos_y").innerHTML = data.position.y.toFixed(2)
    })

    locationGoal = new ROSLIB.Topic({
        ros: data.ros,
        name: '/locationGoal',
        messageType: 'interfaces_gymbrot/msg/LocationGoal'
    })

    // Define callbacks
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

document.addEventListener('DOMContentLoaded', event => {

    connect()






    function disconnect() {
        data.ros.close()
        data.connected = false
        console.log('Clic en botón de desconexión')
    }

    function move() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0.1, y: 0, z: 0, },
            angular: { x: 0, y: 0, z: -0.2, },
        })
        topic.publish(message)
    }

    function stop() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0, },
            angular: { x: 0, y: 0, z: 0, },
        })
        topic.publish(message)
    }

    function move_inv() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: { x: 0.1, y: 0, z: 0, },
            angular: { x: 0, y: 0, z: 0.2, },
        })
        topic.publish(message)
    }





});