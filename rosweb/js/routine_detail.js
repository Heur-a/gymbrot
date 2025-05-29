let robotTimer;
let robotArrivalTime = 3; // 15 segundos
const epsilonTime = 0.5;

async function esperarRobot(button, str_maquna) {
    let maquina;

    switch (str_maquna) {
        case "Máquina 1":
            maquina = machine_1
            break;
        case "Máquina 2":
            maquina = machine_2
            break;
        case "Máquina 3":
            maquina = machine_3
            break;
        default:
            alert("No se sabe la maquina" + str_maquna)
            return;
            break;
    }

    moveToMachine(maquina.x, maquina.y)

    esperando_robot = true

    button.disabled = true;
    button.innerHTML = 'El robot está de camino';
    button.classList.remove('bg-gray-500', 'text-white', 'hover:bg-gray-600');
    button.classList.add('bg-yellow-500', 'text-white', 'cursor-not-allowed');

    let liberado = await esperarLiberacion(maquina)
    //esperar hasta que esperando_robot sea false otra vez
    liberarBotones(button, liberado)

}



function liberarBotones(button, liberado) {
    button.innerHTML = 'Monitorizar';
    button.disabled = false;
    button.classList.remove('bg-yellow-500', 'cursor-not-allowed');
    button.classList.add('bg-green-500', 'hover:bg-green-600');

    //         Añadir event listener para redirigir cuando se haga clic en "Monitorizar"
    button.addEventListener('click', () => {
        window.location.href = 'monitor_activity.html';
    });
    if (!liberado) {
        alert("Se ha agotado el tiempo de espera")
    }
}



function esperarLiberacion(maquina) {
    return new Promise((resolve, reject) => {
        const startTime = Date.now();
        const timeout = 120000; // 120 segundos máximo de espera

        const checkInterval = setInterval(() => {
            if (
                checkRobotHaLlegadoMaquina(maquina, robotPosition.x, robotPosition.y, epsilonTime)
            ) {
                clearInterval(checkInterval);
                resolve(true); // Éxito: robot liberado
            }
            else if (Date.now() - startTime > timeout) {
                clearInterval(checkInterval);
                resolve(false); // Timeout: no se liberó a tiempo
            }
        }, 100); // Comprobar cada 100ms
    });

}

// Función para obtener el ID de la rutina de la URL
function getRoutineId() {
    const urlParams = new URLSearchParams(window.location.search);
    return urlParams.get('id');
}

// Función para cargar los detalles de la rutina
async function loadRoutineDetails() {
    const routineId = getRoutineId();
    if (!routineId) {
        alert('ID de rutina no proporcionado');
        return;
    }

    try {
        const response = await fetch(`../php/get_routine_detail.php?id=${routineId}`);
        const data = await response.json();

        if (data.error) {
            alert(data.error);
            return;
        }

        // Actualizar el título de la página
        document.title = `${data.name} - GymBrot`;
        document.getElementById('routineTitle').textContent = data.name;

        // Contenedor de ejercicios
        const exercisesContainer = document.getElementById('exercisesContainer');
        exercisesContainer.innerHTML = '';

        // Crear las tarjetas de ejercicio
        data.exercises.forEach((exercise, index) => {
            const template = document.getElementById('exerciseCardTemplate');
            const card = template.content.cloneNode(true);

            // Configurar la tarjeta
            const exerciseCard = card.querySelector('.exercise-card');
            exerciseCard.id = `exercise-card-${index + 1}`;

            // Configurar la imagen
            const img = card.querySelector('img');
            img.src = exercise.image || '../assets/ejercicio_prensa.jpg';
            img.alt = exercise.name;

            // Configurar el nombre de la máquina
            const machineName = card.querySelector('.machine-name');
            machineName.textContent = exercise.machine.name;

            // Configurar la descripción
            const description = card.querySelector('.exercise-description');
            description.textContent = `Podrás realizar ${exercise.name}`;

            // Configurar el enlace de detalle
            const detailLink = card.querySelector('.exercise-detail-link');
            detailLink.href = `exercise_detail.html?id=${exercise.id}`;

            // Configurar el botón del robot
            const robotButton = card.querySelector('.robot-button');
            robotButton.id = `robot-button-${index + 1}`;
            if (index > 0) {
                robotButton.disabled = true;
                robotButton.classList.remove('bg-gray-500', 'text-white', 'hover:bg-gray-600');
                robotButton.classList.add('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
            }
            robotButton.addEventListener('click', () => esperarRobot(robotButton, machineName.textContent));

            // Añadir la tarjeta al contenedor
            exercisesContainer.appendChild(card);

            // Añadir botón "Siguiente actividad" entre ejercicios
            if (index < data.exercises.length - 1) {
                const nextButtonContainer = document.createElement('div');
                nextButtonContainer.className = 'flex justify-center py-2';
                nextButtonContainer.innerHTML = `
                            <button class="next-activity-button bg-black text-white px-6 py-2 rounded-full text-sm font-medium hover:bg-gray-800 transition-colors">
                                Siguiente actividad
                            </button>
                        `;
                exercisesContainer.appendChild(nextButtonContainer);

                // Configurar el botón "Siguiente actividad"
                const nextButton = nextButtonContainer.querySelector('.next-activity-button');
                nextButton.addEventListener('click', () => {
                    exerciseCard.style.backgroundColor = '#4CAF50';
                    exerciseCard.classList.add('completed');

                    const nextRobotButton = document.getElementById(`robot-button-${index + 2}`);
                    if (nextRobotButton) {
                        nextRobotButton.disabled = false;
                        nextRobotButton.classList.remove('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
                        nextRobotButton.classList.add('bg-gray-500', 'text-white', 'hover:bg-gray-600');
                    }

                    nextButton.disabled = true;
                    nextButton.classList.add('opacity-50', 'cursor-not-allowed');
                });
            }
        });

    } catch (error) {
        console.error('Error al cargar los detalles de la rutina:', error);
        alert('Error al cargar los detalles de la rutina');
    }
}
//--------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------

// Cargar los detalles cuando la página se cargue
document.addEventListener('DOMContentLoaded', async event => {
    await loadRoutineDetails()
});