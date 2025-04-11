<?php
header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: POST');
header('Access-Control-Allow-Headers: Content-Type');

// Incluir archivo de configuración
require_once 'config.php';

// Obtener conexión
$conn = getDBConnection();

// Obtener datos del POST
$data = json_decode(file_get_contents('php://input'), true);
$email = $data['email'] ?? '';
$password = $data['password'] ?? '';

if (empty($email) || empty($password)) {
    echo json_encode(['success' => false, 'message' => 'Email y contraseña son requeridos']);
    exit;
}

// Preparar y ejecutar la consulta
$stmt = $conn->prepare("SELECT id, password, user_type FROM users WHERE email = ?");
$stmt->bind_param("s", $email);
$stmt->execute();
$result = $stmt->get_result();

if ($result->num_rows === 0) {
    echo json_encode(['success' => false, 'message' => 'Usuario no encontrado']);
    exit;
}

$user = $result->fetch_assoc();

// Verificar la contraseña
if ($password === $user['password']) {
    // Iniciar sesión
    session_start();
    $_SESSION['user_id'] = $user['id'];
    $_SESSION['user_type'] = $user['user_type'];
    
    echo json_encode([
        'success' => true,
        'message' => 'Login exitoso',
        'user_type' => $user['user_type']
    ]);
} else {
    echo json_encode(['success' => false, 'message' => 'Contraseña incorrecta']);
}

$stmt->close();
closeDBConnection($conn);
?> 