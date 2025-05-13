// Variables globales
let currentPage = 1;
const itemsPerPage = 10;
let totalPages = 1;
let filteredData = [];
let allActivities = [];
let allUsers = [];

// Inicialización cuando el DOM está completamente cargado
document.addEventListener('DOMContentLoaded', function() {
    // Cargar datos iniciales
    fetchDashboardStats();
    fetchUsers();
    fetchActivities();
    
    // Inicializar listeners de eventos
    initEventListeners();
});

// Inicializar event listeners
function initEventListeners() {
    // Listeners para paginación
    document.getElementById('prev-page').addEventListener('click', goToPrevPage);
    document.getElementById('next-page').addEventListener('click', goToNextPage);
    
    // Listeners para filtros
    document.getElementById('user-filter').addEventListener('change', applyFilters);
    document.getElementById('activity-filter').addEventListener('change', applyFilters);
    document.getElementById('date-filter').addEventListener('change', applyFilters);
}

// Obtener estadísticas del dashboard
function fetchDashboardStats() {
    fetch('../php/admin_stats.php')
        .then(response => response.json())
        .then(data => {
            // Actualizar contadores
            document.getElementById('active-users-count').textContent = data.activeUsers;
            // Actualizar estado del robot
            const robotElement = document.getElementById('robot-status');
            if (robotElement) {
                robotElement.textContent = data.robotStatus;
                robotElement.className = data.robotAvailable ? 
                    'text-3xl font-bold text-green-500' : 
                    'text-3xl font-bold text-yellow-500';
            }
            document.getElementById('todays-sessions-count').textContent = data.todaysSessions;
        })
        .catch(error => {
            console.error('Error fetching dashboard stats:', error);
        });
}

// Obtener usuarios para el selector
function fetchUsers() {
    fetch('../php/get_users.php')
        .then(response => response.json())
        .then(data => {
            allUsers = data;
            populateUserFilter(data);
        })
        .catch(error => {
            console.error('Error fetching users:', error);
        });
}

// Obtener actividades
function fetchActivities() {
    fetch('../php/get_activities.php')
        .then(response => response.json())
        .then(data => {
            allActivities = data;
            filteredData = [...data]; // Copia inicial sin filtros
            totalPages = Math.ceil(filteredData.length / itemsPerPage);
            updatePageInfo();
            renderActivities();
        })
        .catch(error => {
            console.error('Error fetching activities:', error);
        });
}

// Rellenar selector de usuarios
function populateUserFilter(users) {
    const userFilter = document.getElementById('user-filter');
    
    // Limpiar opciones existentes excepto la primera
    while (userFilter.options.length > 1) {
        userFilter.options.remove(1);
    }
    
    // Añadir usuarios al selector
    users.forEach(user => {
        const option = document.createElement('option');
        option.value = user.id;
        option.textContent = user.email;
        userFilter.appendChild(option);
    });
}

// Aplicar filtros a los datos
function applyFilters() {
    const userFilter = document.getElementById('user-filter').value;
    const activityFilter = document.getElementById('activity-filter').value;
    const dateFilter = document.getElementById('date-filter').value;
    
    // Filtrar actividades basadas en criterios seleccionados
    filteredData = allActivities.filter(activity => {
        // Filtro de usuario
        if (userFilter !== 'all' && activity.user_id !== userFilter) {
            return false;
        }
        
        // Filtro de tipo de actividad
        if (activityFilter !== 'all' && activity.activity_type !== activityFilter) {
            return false;
        }
        
        // Filtro de fecha
        if (dateFilter !== 'all') {
            const activityDate = new Date(activity.created_at);
            const today = new Date();
            today.setHours(0, 0, 0, 0);
            
            if (dateFilter === 'today') {
                const tomorrow = new Date(today);
                tomorrow.setDate(tomorrow.getDate() + 1);
                return activityDate >= today && activityDate < tomorrow;
            } else if (dateFilter === 'week') {
                const weekStart = new Date(today);
                weekStart.setDate(today.getDate() - today.getDay());
                const weekEnd = new Date(weekStart);
                weekEnd.setDate(weekStart.getDate() + 7);
                return activityDate >= weekStart && activityDate < weekEnd;
            } else if (dateFilter === 'month') {
                const monthStart = new Date(today.getFullYear(), today.getMonth(), 1);
                const monthEnd = new Date(today.getFullYear(), today.getMonth() + 1, 0);
                return activityDate >= monthStart && activityDate <= monthEnd;
            }
        }
        
        return true;
    });
    
    // Reiniciar a la primera página y actualizar la visualización
    currentPage = 1;
    totalPages = Math.ceil(filteredData.length / itemsPerPage);
    updatePageInfo();
    renderActivities();
}

// Renderizar actividades en la tabla
function renderActivities() {
    const tableBody = document.getElementById('activities-table-body');
    
    // Limpiar tabla
    tableBody.innerHTML = '';
    
    // Calcular índices para paginación
    const startIndex = (currentPage - 1) * itemsPerPage;
    const endIndex = Math.min(startIndex + itemsPerPage, filteredData.length);
    
    // Si no hay datos después de filtrar
    if (filteredData.length === 0) {
        const emptyRow = document.createElement('tr');
        emptyRow.innerHTML = `
            <td colspan="6" class="py-4 text-center text-gray-500">No se encontraron actividades con los filtros seleccionados</td>
        `;
        tableBody.appendChild(emptyRow);
        return;
    }
    
    // Añadir filas para los datos filtrados
    for (let i = startIndex; i < endIndex; i++) {
        const activity = filteredData[i];
        const row = document.createElement('tr');
        row.className = 'border-b';
        
        // Formatear fecha para mostrar
        const date = new Date(activity.created_at);
        const formattedDate = date.toLocaleString('es-ES');
        
        // Encontrar información de usuario
        const user = allUsers.find(u => u.id === activity.user_id) || { email: 'Desconocido' };
        
        // Formatear clase según tipo de actividad
        let activityClass = 'bg-gray-100 text-gray-800';
        switch (activity.activity_type) {
            case 'login':
                activityClass = 'bg-green-100 text-green-800';
                break;
            case 'logout':
                activityClass = 'bg-yellow-100 text-yellow-800';
                break;
            case 'exercise':
                activityClass = 'bg-blue-100 text-blue-800';
                break;
            case 'profile_update':
                activityClass = 'bg-purple-100 text-purple-800';
                break;
            case 'robot_interaction':
                activityClass = 'bg-orange-100 text-orange-800';
                break;
        }
        
        row.innerHTML = `
            <td class="py-2">${user.email}</td>
            <td class="py-2">
                <span class="px-2 py-1 ${activityClass} rounded-full text-sm">
                    ${activity.activity_type}
                </span>
            </td>
            <td class="py-2">${activity.description}</td>
            <td class="py-2">${formattedDate}</td>
            <td class="py-2">${activity.robot_id ? 'GymBrot' : '-'}</td>
            <td class="py-2">${activity.exercise_name || '-'}</td>
        `;
        
        tableBody.appendChild(row);
    }
}

// Navegación de páginas
function goToPrevPage() {
    if (currentPage > 1) {
        currentPage--;
        updatePageInfo();
        renderActivities();
    }
}

function goToNextPage() {
    if (currentPage < totalPages) {
        currentPage++;
        updatePageInfo();
        renderActivities();
    }
}

// Actualizar información de paginación
function updatePageInfo() {
    document.getElementById('page-info').textContent = `Página ${currentPage} de ${totalPages || 1}`;
    
    // Habilitar/deshabilitar botones de navegación
    document.getElementById('prev-page').disabled = currentPage === 1;
    document.getElementById('next-page').disabled = currentPage === totalPages || totalPages === 0;
    
    // Aplicar estilos visuales según estado
    if (currentPage === 1) {
        document.getElementById('prev-page').classList.add('opacity-50', 'cursor-not-allowed');
    } else {
        document.getElementById('prev-page').classList.remove('opacity-50', 'cursor-not-allowed');
    }
    
    if (currentPage === totalPages || totalPages === 0) {
        document.getElementById('next-page').classList.add('opacity-50', 'cursor-not-allowed');
    } else {
        document.getElementById('next-page').classList.remove('opacity-50', 'cursor-not-allowed');
    }
}

// Actualizar datos del robot
function loadRobotData() {
    fetch('../php/get_robots.php')
        .then(response => response.json())
        .then(data => {
            if (data.length > 0) {
                const robot = data[0];
                const robotInfo = document.getElementById('robot-info');
                if (robotInfo) {
                    // Actualizar la información del robot
                    const statusClass = robot.status === 'Activo' 
                        ? 'bg-green-100 text-green-800' 
                        : 'bg-yellow-100 text-yellow-800';
                    
                    robotInfo.innerHTML = `
                        <div class="py-2">
                            <h3 class="text-lg font-semibold text-gray-700 mb-2">Estado del Robot</h3>
                            <div class="flex items-center space-x-2 mb-2">
                                <span class="px-2 py-1 ${statusClass} rounded-full text-sm">${robot.status}</span>
                                <span>Ubicación: ${robot.location}</span>
                            </div>
                            <button id="view-robot-details" class="text-[#F1E1A5] hover:text-[#727272] transition-colors">
                                Ver Detalles
                            </button>
                        </div>
                    `;
                    
                    // Añadir event listener al botón de detalles
                    document.getElementById('view-robot-details')?.addEventListener('click', function() {
                        viewRobotDetails(robot.id);
                    });
                }
            }
        })
        .catch(error => {
            console.error('Error loading robot data:', error);
        });
}

// Actualizar datos de usuarios
function loadUsersData() {
    fetch('../php/get_users.php')
        .then(response => response.json())
        .then(data => {
            const usersTableBody = document.getElementById('users-table-body');
            usersTableBody.innerHTML = '';
            
            data.forEach(user => {
                const typeClass = user.user_type === 1 
                    ? 'bg-blue-100 text-blue-800' 
                    : 'bg-gray-100 text-gray-800';
                
                const typeName = user.user_type === 1 ? 'Admin' : 'Usuario';
                
                const row = document.createElement('tr');
                row.className = 'border-b';
                row.innerHTML = `
                    <td class="py-2">${user.id}</td>
                    <td class="py-2">${user.email}</td>
                    <td class="py-2">
                        <span class="px-2 py-1 ${typeClass} rounded-full text-sm">${typeName}</span>
                    </td>
                    <td class="py-2">
                        <button class="edit-user text-[#F1E1A5] hover:text-[#727272] transition-colors mr-2" data-id="${user.id}">Editar</button>
                        <button class="view-user-activities text-[#F1E1A5] hover:text-[#727272] transition-colors" data-id="${user.id}">Ver Actividades</button>
                    </td>
                `;
                usersTableBody.appendChild(row);
            });
            
            // Añadir event listeners a los botones
            document.querySelectorAll('.edit-user').forEach(button => {
                button.addEventListener('click', function() {
                    const userId = this.getAttribute('data-id');
                    editUser(userId);
                });
            });
            
            document.querySelectorAll('.view-user-activities').forEach(button => {
                button.addEventListener('click', function() {
                    const userId = this.getAttribute('data-id');
                    document.getElementById('user-filter').value = userId;
                    applyFilters();
                    // Scroll to activities section
                    document.querySelector('.grid:contains("Histórico de Actividades")').scrollIntoView({ behavior: 'smooth' });
                });
            });
        })
        .catch(error => {
            console.error('Error loading users data:', error);
        });
}

// Funciones para gestionar el robot y usuarios (expandibles)
function viewRobotDetails(robotId) {
    // Implementación futura: mostrar detalles en un modal
    console.log('Ver detalles del robot GymBrot');
    // Podrías abrir un modal con detalles adicionales aquí
}

function editUser(userId) {
    // Implementación futura: mostrar formulario de edición
    console.log('Editar usuario:', userId);
    // Podrías abrir un modal con formulario de edición aquí
}

// Cargar datos iniciales
loadRobotData();
loadUsersData(); 