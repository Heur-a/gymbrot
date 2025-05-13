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

// Obtener usuarios activos (han iniciado sesión en las últimas 24 horas)
$queryActiveUsers = "SELECT COUNT(DISTINCT user_id) as count 
                     FROM user_activities 
                     WHERE activity_type = 'login' 
                     AND created_at > DATE_SUB(NOW(), INTERVAL 24 HOUR)";

$resultActiveUsers = $conn->query($queryActiveUsers);
$activeUsers = 0;
if ($resultActiveUsers && $row = $resultActiveUsers->fetch_assoc()) {
    $activeUsers = $row['count'];
}

// Verificar si el robot está en uso (en la última hora)
$queryRobotStatus = "SELECT 
                        CASE 
                            WHEN EXISTS (
                                SELECT 1 
                                FROM user_activities 
                                WHERE robot_id = 1 
                                AND created_at > DATE_SUB(NOW(), INTERVAL 1 HOUR)
                            ) 
                            THEN 'En Uso' 
                            ELSE 'Disponible' 
                        END AS status";

$resultRobotStatus = $conn->query($queryRobotStatus);
$robotStatus = 'Desconocido';
if ($resultRobotStatus && $row = $resultRobotStatus->fetch_assoc()) {
    $robotStatus = $row['status'];
}

// Obtener número de sesiones (ejercicios realizados) hoy
$queryTodaysSessions = "SELECT COUNT(*) as count 
                        FROM user_activities 
                        WHERE activity_type = 'exercise' 
                        AND DATE(created_at) = CURDATE()";

$resultTodaysSessions = $conn->query($queryTodaysSessions);
$todaysSessions = 0;
if ($resultTodaysSessions && $row = $resultTodaysSessions->fetch_assoc()) {
    $todaysSessions = $row['count'];
}

// Preparar respuesta
$response = [
    'activeUsers' => $activeUsers,
    'robotStatus' => $robotStatus,
    'robotAvailable' => ($robotStatus === 'Disponible' ? 1 : 0),
    'todaysSessions' => $todaysSessions
];

// Devolver resultados como JSON
header('Content-Type: application/json');
echo json_encode($response);

// Cerrar conexión
closeDBConnection($conn);
?> 