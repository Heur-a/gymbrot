<?php
require_once 'config.php';

header('Content-Type: application/json');

if (!isset($_GET['id'])) {
    echo json_encode(['error' => 'Routine ID is required']);
    exit;
}

$routineId = intval($_GET['id']);
$conn = getDBConnection();

// Get routine details
$query = "
    SELECT 
        r.id,
        r.name,
        e.id as exercise_id,
        e.name as exercise_name,
        e.image as exercise_image,
        e.task as exercise_task,
        CONCAT('Máquina ', m.id) as machine_name,
        m.id as machine_id
    FROM routine r
    JOIN routine_exercise re ON r.id = re.routine
    JOIN exercises e ON re.exercise = e.id
    JOIN machines m ON e.machine = m.id
    WHERE r.id = ?
    ORDER BY re.id ASC
";

$stmt = $conn->prepare($query);
$stmt->bind_param('i', $routineId);
$stmt->execute();
$result = $stmt->get_result();

$routine = [
    'id' => $routineId,
    'name' => '',
    'exercises' => []
];

if ($result->num_rows > 0) {
    while ($row = $result->fetch_assoc()) {
        if (empty($routine['name'])) {
            $routine['name'] = $row['name'];
        }
        $routine['exercises'][] = [
            'id' => $row['exercise_id'],
            'name' => $row['exercise_name'],
            'image' => $row['exercise_image'],
            'task' => $row['exercise_task'],
            'machine' => [
                'id' => $row['machine_id'],
                'name' => $row['machine_name']
            ]
        ];
    }
    echo json_encode($routine);
} else {
    echo json_encode(['error' => 'Routine not found']);
}

$stmt->close();
$conn->close();
?> 