<?php
require 'config.php';

$conn = getDBConnection();
if ($conn) {
    echo "<p style='color:green'>Conexión exitosa a «".DB_NAME."» en ".DB_SERVER."</p>";
} else {
    echo "<p style='color:red'>No se pudo conectar.</p>";
}
closeDBConnection($conn);
?>