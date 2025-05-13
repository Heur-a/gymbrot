<?php
require_once 'config.php';

header('Content-Type: application/json');

$conn = getDBConnection();

$data = [
    'exercises' => [],
    'routines' => [],
    'machines' => []
];

// Obtener ejercicios con detalles de máquina y músculo
$queryExercises = "
    SELECT 
        e.id,
        e.name AS title,
        e.description,
        e.image,
        CONCAT('Máquina ', m.id) AS machine,
        mu.name AS muscle
    FROM exercises e
    JOIN machines m ON e.machine = m.id
    JOIN muscles mu ON e.muscle = mu.id
";
$result = $conn->query($queryExercises);
if ($result) {
    while ($row = $result->fetch_assoc()) {
        $data['exercises'][] = $row;
    }
    $result->free();
} else {
    echo json_encode(['error' => 'Error al obtener ejercicios: ' . $conn->error]);
    exit;
}

// Obtener rutinas con conteo de ejercicios
$queryRoutines = "
    SELECT 
        r.id,
        r.name AS title,
        COUNT(re.exercise) AS exercise_count,
        CASE 
            WHEN r.id = 1 THEN 'https://dmtxworkout.com/wp-content/uploads/2022/05/RUTINA_3_900x.jpg'
            WHEN r.id = 2 THEN 'https://i0.wp.com/dmtxworkout.com/wp-content/uploads/2022/05/RUTINA_1_900x.jpg?w=900&ssl=1'
            WHEN r.id = 3 THEN 'https://i0.wp.com/dmtxworkout.com/wp-content/uploads/2022/05/RUTINA_4_900x.jpg?w=900&ssl=1'
        END as image
    FROM routine r
    LEFT JOIN routine_exercise re ON r.id = re.routine
    GROUP BY r.id
";
$result = $conn->query($queryRoutines);
if ($result) {
    while ($row = $result->fetch_assoc()) {
        $data['routines'][] = [
            'id' => $row['id'],
            'title' => $row['title'],
            'description' => 'Ejercicios: ' . $row['exercise_count'],
            'image' => $row['image']
        ];
    }
    $result->free();
} else {
    echo json_encode(['error' => 'Error al obtener rutinas: ' . $conn->error]);
    exit;
}

// Obtener máquinas con título generado
$queryMachines = "
    SELECT 
        id,
        locX,
        locY,
        orientation,
        CONCAT('Máquina ', id) AS title
    FROM machines
";
$result = $conn->query($queryMachines);
if ($result) {
    while ($row = $result->fetch_assoc()) {
        $data['machines'][] = $row;
    }
    $result->free();
} else {
    echo json_encode(['error' => 'Error al obtener máquinas: ' . $conn->error]);
    exit;
}

$conn->close();
echo json_encode($data);
?>