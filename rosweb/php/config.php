<?php
// Configuración de la base de datos
define('DB_SERVER', 'localhost');
define('DB_USERNAME', 'root');
define('DB_PASSWORD', '');
define('DB_NAME', 'gymbrot');

// Función para obtener la conexión
function getDBConnection() {
    $conn = new mysqli(DB_SERVER, DB_USERNAME, DB_PASSWORD, DB_NAME);
    
    // Verificar conexión
    if ($conn->connect_error) {
        die("Error de conexión: " . $conn->connect_error);
    }
    
    // Establecer el conjunto de caracteres
    $conn->set_charset("utf8");
    
    return $conn;
}

// Función para cerrar la conexión
function closeDBConnection($conn) {
    if ($conn) {
        $conn->close();
    }
}
?> 