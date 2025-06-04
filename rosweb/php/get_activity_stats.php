<?php
require_once 'config.php';

$conn = getDBConnection();

// Si se solicita ?type=exercises, devolver los ejercicios más realizados
if (isset($_GET['type']) && $_GET['type'] === 'exercises') {
    $sql = "SELECT e.name AS exercise_name, COUNT(*) AS count
            FROM user_activities ua
            JOIN exercises e ON ua.exercise_id = e.id
            WHERE ua.activity_type = 'exercise'
            GROUP BY ua.exercise_id
            ORDER BY count DESC";
    $result = $conn->query($sql);
    $stats = [];
    while ($row = $result->fetch_assoc()) {
        $stats[] = $row;
    }
    header('Content-Type: application/json');
    echo json_encode($stats);
    closeDBConnection($conn);
    exit;
}

// Por defecto, devolver el resumen de tipos de actividad
$sql = "SELECT activity_type, COUNT(*) as count FROM user_activities GROUP BY activity_type";
$result = $conn->query($sql);

$stats = [];
while ($row = $result->fetch_assoc()) {
    $stats[] = $row;
}

header('Content-Type: application/json');
echo json_encode($stats);

closeDBConnection($conn); 