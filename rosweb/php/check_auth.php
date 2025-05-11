<?php
session_start();

header('Content-Type: application/json');
header('Access-Control-Allow-Origin: *');
header('Access-Control-Allow-Methods: GET');
header('Access-Control-Allow-Headers: Content-Type');

if (isset($_SESSION['user_id']) && isset($_SESSION['user_type'])) {
    echo json_encode([
        'authenticated' => true,
        'user_id' => $_SESSION['user_id'],
        'user_type' => $_SESSION['user_type']
    ]);
} else {
    echo json_encode([
        'authenticated' => false,
        'message' => 'Usuario no autenticado'
    ]);
}
?> 