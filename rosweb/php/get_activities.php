<?php
// Configuración y conexión a la base de datos
require_once 'config.php';

// Verificar si hay una sesión de administrador activa (comentado para pruebas)
/*
session_start();
if (!isset($_SESSION['user_id']) || $_SESSION['user_type'] != 1) {
    // Retornar error si no es un administrador
    header('Content-Type: application/json');
    echo json_encode(['error' => 'Acceso denegado']);
    exit;
}
*/

// Obtener conexión a la base de datos
$conn = getDBConnection();

// Verificar si la tabla user_activities existe
$table_exists = false;
$result = $conn->query("SHOW TABLES LIKE 'user_activities'");
if ($result && $result->num_rows > 0) {
    $table_exists = true;
}

// Inicializar array vacío para actividades
$activities = array();

if ($table_exists) {
    // Consulta para obtener actividades con información relacionada
    $query = "SELECT 
                ua.id, 
                ua.user_id, 
                ua.activity_type, 
                ua.description, 
                ua.created_at, 
                ua.robot_id, 
                ua.exercise_id,
                u.email as user_email,
                CONCAT('RB00', ua.robot_id) as robot_code,
                e.name as exercise_name
              FROM 
                user_activities ua
              LEFT JOIN 
                users u ON ua.user_id = u.id
              LEFT JOIN 
                exercises e ON ua.exercise_id = e.id
              ORDER BY 
                ua.created_at DESC";

    $result = $conn->query($query);

    if ($result && $result->num_rows > 0) {
        // Convertir resultados a array
        while ($row = $result->fetch_assoc()) {
            $activities[] = $row;
        }
    }
} else {
    // Si la tabla no existe, crear las tablas necesarias
    // Crear tabla activity_types si no existe
    $sql_activity_types = "CREATE TABLE IF NOT EXISTS `activity_types` (
      `id` int(11) NOT NULL AUTO_INCREMENT,
      `name` varchar(50) NOT NULL,
      `description` varchar(255) NOT NULL,
      PRIMARY KEY (`id`),
      UNIQUE KEY `name` (`name`)
    ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;";

    if ($conn->query($sql_activity_types) === TRUE) {
        // Insertar datos en activity_types
        $sql_insert_types = "INSERT IGNORE INTO `activity_types` (`id`, `name`, `description`) VALUES
        (1, 'login', 'Inicio de sesión del usuario'),
        (2, 'logout', 'Cierre de sesión del usuario'),
        (3, 'exercise', 'Realización de ejercicio'),
        (4, 'profile_update', 'Actualización de perfil'),
        (5, 'robot_interaction', 'Interacción con el robot');";
        $conn->query($sql_insert_types);
    }

    // Crear tabla user_activities si no existe
    $sql_user_activities = "CREATE TABLE IF NOT EXISTS `user_activities` (
      `id` int(11) NOT NULL AUTO_INCREMENT,
      `user_id` int(11) NOT NULL,
      `activity_type` varchar(50) NOT NULL,
      `description` text NOT NULL,
      `created_at` datetime NOT NULL DEFAULT current_timestamp(),
      `robot_id` tinyint(3) UNSIGNED DEFAULT NULL,
      `exercise_id` int(11) DEFAULT NULL,
      PRIMARY KEY (`id`),
      KEY `fk_user_activities_user` (`user_id`),
      KEY `fk_user_activities_exercise` (`exercise_id`)
    ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci;";

    if ($conn->query($sql_user_activities) === TRUE) {
        // Insertar algunos datos de ejemplo en user_activities
        $sql_sample_data = "INSERT INTO `user_activities` (`user_id`, `activity_type`, `description`, `created_at`, `robot_id`, `exercise_id`) VALUES
        (2, 'login', 'Inicio de sesión en la plataforma', NOW() - INTERVAL 2 HOUR, NULL, NULL),
        (2, 'exercise', 'Completó Press de banca', NOW() - INTERVAL 1 HOUR 30 MINUTE, 1, 1),
        (2, 'exercise', 'Completó Yoga Vinyasa', NOW() - INTERVAL 1 HOUR, 1, 2),
        (2, 'logout', 'Cierre de sesión en la plataforma', NOW() - INTERVAL 30 MINUTE, NULL, NULL);";
        $conn->query($sql_sample_data);
    }
}

// Devolver resultados como JSON - siempre devolvemos un array aunque esté vacío
header('Content-Type: application/json');
echo json_encode($activities);

// Cerrar conexión
closeDBConnection($conn);
?> 