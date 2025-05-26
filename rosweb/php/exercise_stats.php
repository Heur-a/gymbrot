<?php
require_once 'config.php';

$conn = getDBConnection();

$sql = "SELECT 
            e.name, 
            COUNT(ua.id) as total,
            MAX(ua.created_at) as last_date
        FROM user_activities ua
        JOIN exercises e ON ua.exercise_id = e.id
        WHERE ua.activity_type = 'exercise'
        GROUP BY e.name
        ORDER BY total DESC";

$result = $conn->query($sql);
$stats = [];
while ($row = $result->fetch_assoc()) {
    $stats[] = $row;
}

header('Content-Type: application/json');
echo json_encode($stats);

closeDBConnection($conn); 