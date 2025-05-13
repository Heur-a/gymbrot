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

// Consulta para obtener usuarios
$query = "SELECT 
            u.id, 
            u.email, 
            u.user_type,
            ut.rol as rol_name
          FROM 
            users u
          JOIN 
            user_type ut ON u.user_type = ut.id
          ORDER BY 
            u.id ASC";

$result = $conn->query($query);

if ($result) {
    $users = array();
    
    // Convertir resultados a array
    while ($row = $result->fetch_assoc()) {
        $users[] = $row;
    }
    
    // Devolver resultados como JSON
    header('Content-Type: application/json');
    echo json_encode($users);
} else {
    // Manejar errores de base de datos
    header('Content-Type: application/json');
    echo json_encode(['error' => 'Error de base de datos: ' . $conn->error]);
}

// Cerrar conexión
closeDBConnection($conn);
?> 