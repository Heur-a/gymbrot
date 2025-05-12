<?php
// Configuración y conexión a la base de datos
require_once 'config.php';

// Verificar si hay una sesión de administrador activa
session_start();
if (!isset($_SESSION['user_id']) || $_SESSION['user_type'] != 1) {
    // Retornar error si no es un administrador
    header('Content-Type: application/json');
    echo json_encode(['error' => 'Acceso denegado']);
    exit;
}

try {
    // Crear conexión PDO
    $pdo = new PDO("mysql:host=$db_host;dbname=$db_name", $db_user, $db_pass);
    $pdo->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    
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
    
    $stmt = $pdo->prepare($query);
    $stmt->execute();
    
    // Convertir resultados a array asociativo
    $activities = $stmt->fetchAll(PDO::FETCH_ASSOC);
    
    // Devolver resultados como JSON
    header('Content-Type: application/json');
    echo json_encode($activities);
    
} catch (PDOException $e) {
    // Manejar errores de base de datos
    header('Content-Type: application/json');
    echo json_encode(['error' => 'Error de base de datos: ' . $e->getMessage()]);
}
?> 