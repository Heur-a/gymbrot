<?php
// 1. Iniciar sesión lo primero
session_start();

// 2. Evitar que cualquier warning/notice rompa el JSON
ini_set('display_errors', 0);
error_reporting(0);

// 3. Cabeceras CORS y JSON
header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: POST');
header('Access-Control-Allow-Headers: Content-Type');

// 4. Carga la configuración y obtiene la conexión
require_once __DIR__ . '/config.php';
$conn = getDBConnection();

// 5. Leer el JSON entrante
$payload  = file_get_contents('php://input');
$data     = json_decode($payload, true);
$email    = trim($data['email']    ?? '');
$password = trim($data['password'] ?? '');

// 6. Validaciones básicas
if ($email === '' || $password === '') {
    echo json_encode([
        'success' => false,
        'message' => 'Email y contraseña son requeridos'
    ]);
    exit;
}

// 7. Buscar al usuario
$stmt = $conn->prepare("
    SELECT id, password, user_type
      FROM users
     WHERE email = ?
    LIMIT 1
");
$stmt->bind_param('s', $email);
$stmt->execute();
$result = $stmt->get_result();

if ($result->num_rows === 0) {
    echo json_encode([
        'success' => false,
        'message' => 'Usuario no encontrado'
    ]);
    exit;
}

$user = $result->fetch_assoc();

// 8. Verificar contraseña (aquí a futuro cambia a password_verify si la guardas hasheada)
if ($password === $user['password']) {
    // 9. Éxito: almacenar datos de sesión
    $_SESSION['user_id']   = (int)$user['id'];
    $_SESSION['user_type'] = (int)$user['user_type'];

    echo json_encode([
        'success'   => true,
        'message'   => 'Login exitoso',
        'user_type' => $_SESSION['user_type']
    ]);
    exit;
} else {
    // 10. Contraseña incorrecta
    echo json_encode([
        'success' => false,
        'message' => 'Contraseña incorrecta'
    ]);
    exit;
}

// 11. Cerrar recursos (nunca se alcanza porque todos los caminos hacen exit)
$stmt->close();
closeDBConnection($conn);
