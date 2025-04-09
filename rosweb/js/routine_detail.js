document.addEventListener('DOMContentLoaded', () => {
    const exerciseCard1 = document.getElementById('exercise-card-1');
    const nextActivityButton = document.getElementById('next-activity-button');
    const exerciseCard2 = document.getElementById('exercise-card-2');
    const robotButton1 = document.getElementById('robot-button-1');
    const robotButton2 = document.getElementById('robot-button-2');

    let robotTimer;
    let robotArrivalTime = 15; // 15 segundos

    function startRobotTimer(button) {
        console.log('Iniciando temporizador para el botón:', button.id);
        button.disabled = true;
        button.innerHTML = 'El robot está de camino';
        button.classList.remove('bg-gray-500', 'text-white', 'hover:bg-gray-600');
        button.classList.add('bg-yellow-500', 'text-white', 'cursor-not-allowed');

        robotTimer = setInterval(() => {
            robotArrivalTime--;
            console.log('Tiempo restante:', robotArrivalTime);
            
            if (robotArrivalTime <= 0) {
                clearInterval(robotTimer);
                button.innerHTML = 'Monitorizar';
                button.disabled = false;
                button.classList.remove('bg-yellow-500', 'cursor-not-allowed');
                button.classList.add('bg-green-500', 'hover:bg-green-600');
                
                // Añadir event listener para redirigir cuando se haga clic en "Monitorizar"
                button.addEventListener('click', () => {
                    window.location.href = 'monitor_activity.html';
                });
            }
        }, 1000);
    }

    if (robotButton1) {
        robotButton1.addEventListener('click', () => {
            console.log('Botón 1 clickeado');
            startRobotTimer(robotButton1);
        });
    }

    nextActivityButton.addEventListener('click', () => {
        // Marcar la primera tarjeta como completada
        exerciseCard1.style.backgroundColor = '#4CAF50'; // Verde para indicar completado
        exerciseCard1.classList.add('completed'); // Añadir clase para estilos adicionales si es necesario

        // Habilitar el botón de la segunda máquina
        robotButton2.disabled = false;
        robotButton2.classList.remove('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
        robotButton2.classList.add('bg-gray-500', 'text-white', 'hover:bg-gray-600');

        // Deshabilitar el botón siguiente
        nextActivityButton.disabled = true;
        nextActivityButton.classList.add('opacity-50', 'cursor-not-allowed');
    });

    if (robotButton2) {
        robotButton2.addEventListener('click', () => {
            console.log('Botón 2 clickeado');
            startRobotTimer(robotButton2);
        });
    }
}); 