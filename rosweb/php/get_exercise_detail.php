<?php
require_once 'config.php';

header('Content-Type: application/json');

if (!isset($_GET['id'])) {
    echo json_encode(['error' => 'Exercise ID is required']);
    exit;
}

$exerciseId = intval($_GET['id']);
$conn = getDBConnection();

$query = "
    SELECT 
        e.id,
        e.name,
        e.description,
        e.image,
        e.task,
        e.video,
        CONCAT('Máquina ', m.id) AS machine,
        mu.name AS muscle
    FROM exercises e
    JOIN machines m ON e.machine = m.id
    JOIN muscles mu ON e.muscle = mu.id
    WHERE e.id = ?
";

$stmt = $conn->prepare($query);
$stmt->bind_param('i', $exerciseId);
$stmt->execute();
$result = $stmt->get_result();

if ($row = $result->fetch_assoc()) {
    echo json_encode($row);
} else {
    echo json_encode(['error' => 'Exercise not found']);
}

$stmt->close();
$conn->close();
?> 