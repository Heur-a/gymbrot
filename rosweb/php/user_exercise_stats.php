<?php
require_once 'config.php';

$conn = getDBConnection();

$sql = "SELECT u.email, COUNT(ua.id) as total 
        FROM user_activities ua
        JOIN users u ON ua.user_id = u.id
        WHERE ua.activity_type = 'exercise'
        GROUP BY ua.user_id";

$result = $conn->query($sql);
$data = [];
while ($row = $result->fetch_assoc()) {
    $data[] = $row;
}

header('Content-Type: application/json');
echo json_encode($data);

closeDBConnection($conn); 