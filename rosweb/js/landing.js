// Referencia al botón y contenedor de marcadores
const callButton = document.getElementById('callButton');
const markersContainer = document.getElementById('markers-container');
let selectedMarker = null;

// Función para habilitar el botón
function enableCallButton() {
    callButton.disabled = false;
    callButton.classList.remove('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    callButton.classList.add('bg-[#F1E1A5]', 'text-[#727272]', 'hover:bg-[#F1E1A5]/80');
}

// Función para deshabilitar el botón
function disableCallButton() {
    callButton.disabled = true;
    callButton.classList.add('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    callButton.classList.remove('bg-[#F1E1A5]', 'text-[#727272]', 'hover:bg-[#F1E1A5]/80');
}

//Funcion para llamar al robot
callButton.addEventListener('click', (e) => {

    if(selectedGym === null){
        alert('No hay gym seleccionado')
        return
    }

       switch(selectedGym) {
        case gyms[0].name:
            moveToMachine(machine_1.x,machine_1.y)
            break;
        case gyms[1].name:
            moveToMachine(machine_2.x,machine_2.y)
        case gyms[2].name:
            moveToMachine(machine_3.x, machine_3.y)
       }

})

// Datos de los gimnasios
const gyms = [
    { id: 1, name: 'Máquina 1' },
    { id: 2, name: 'Máquina 2' },
    { id: 3, name: 'Máquina 3' }
];

// Crear marcadores
gyms.forEach((gym, index) => {
    const markerContainer = document.createElement('div');
    markerContainer.className = 'absolute cursor-pointer transform transition-transform hover:scale-110 flex flex-col items-center';
    
    // Posicionar cada marcador en una esquina
    switch(index) {
        case 0: // Esquina superior izquierda
            markerContainer.style.top = '0';
            markerContainer.style.left = '0';
            break;
        case 1: // Esquina superior derecha
            markerContainer.style.top = '0';
            markerContainer.style.right = '0';
            break;
        case 2: // Centro inferior
            markerContainer.style.bottom = '0';
            markerContainer.style.left = '50%';
            markerContainer.style.transform = 'translateX(-50%)';
            markerContainer.classList.add('hover:scale-110');
            break;
    }

    markerContainer.innerHTML = `
        <div class="w-16 h-16 mb-2 flex items-center justify-center">
            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke-width="1.5" class="w-full h-full transition-colors ${selectedMarker === markerContainer ? 'stroke-[#727272]' : 'stroke-[#F1E1A5]'}">
                <path stroke-linecap="round" stroke-linejoin="round" d="M15 10.5a3 3 0 11-6 0 3 3 0 016 0z" />
                <path stroke-linecap="round" stroke-linejoin="round" d="M19.5 10.5c0 7.142-7.5 11.25-7.5 11.25S4.5 17.642 4.5 10.5a7.5 7.5 0 1115 0z" />
            </svg>
        </div>
        <span class="text-[#727272] font-medium whitespace-nowrap">${gym.name}</span>
    `;

    markerContainer.addEventListener('click', (e) => {
        e.stopPropagation(); // Evitar que el clic se propague al documento
        
        const markerSvg = markerContainer.querySelector('svg');
        
        // Si este marcador ya está seleccionado, deseleccionarlo
        if (selectedMarker === markerContainer) {
            markerSvg.classList.remove('stroke-[#727272]');
            markerSvg.classList.add('stroke-[#F1E1A5]');
            selectedMarker = null;
            window.selectedGym = null;
            disableCallButton();
            return;
        }
        
        // Restaurar todos los marcadores a amarillo
        document.querySelectorAll('#markers-container svg').forEach(svg => {
            svg.classList.remove('stroke-[#727272]');
            svg.classList.add('stroke-[#F1E1A5]');
        });

        // Cambiar el marcador seleccionado a negro
        markerSvg.classList.remove('stroke-[#F1E1A5]');
        markerSvg.classList.add('stroke-[#727272]');
        
        // Actualizar selección
        selectedMarker = markerContainer;
        window.selectedGym = gym.name;
        
        // Habilitar botón
        enableCallButton();
    });

    markersContainer.appendChild(markerContainer);
});

// Evento de clic fuera de los marcadores
document.addEventListener('click', (e) => {
    if (!markersContainer.contains(e.target)) {
        document.querySelectorAll('#markers-container svg').forEach(svg => {
            svg.classList.remove('stroke-[#727272]');
            svg.classList.add('stroke-[#F1E1A5]');
        });
        selectedMarker = null;
        window.selectedGym = null;
        disableCallButton();
    }
});

// Funcionalidad del paginador
const pages = [
    {
        title: 'EJERCICIOS',
        content: [
            {
                title: 'Extensión',
                description: 'Realizar extensión de cuadricep 12x3rep',
                machine: 'MÁQUINA 1',
                image: '../assets/ejercicio1.jpg'
            },
            {
                title: 'Press de pierna',
                description: 'Press de pierna inclinado 10x4rep',
                machine: 'MÁQUINA 2',
                image: '../assets/ejercicio2.jpg'
            }
        ]
    },
    {
        title: 'RUTINAS',
        content: [
            {
                title: 'Rutina de Pierna',
                description: 'Rutina completa para piernas',
                machine: 'MÁQUINA 1',
                image: '../assets/rutina1.jpg'
            }
        ]
    },
    {
        title: 'MÁQUINAS',
        content: [
            {
                title: 'Prensa',
                description: 'Máquina para ejercicios de pierna',
                machine: 'MÁQUINA 2',
                image: '../assets/maquina1.jpg'
            }
        ]
    }
];

let currentPage = 0;

function renderPage(pageIndex) {
    const pageContent = document.getElementById('pageContent');
    const searchContainer = document.getElementById('searchContainer'); // Obtener el contenedor del buscador
    const page = pages[pageIndex];
    
    // Controlar visibilidad del buscador
    if (searchContainer) { // Asegurarse de que el elemento exista
        if (pageIndex === 0) {
            searchContainer.style.display = ''; // Mostrar (o resetear a default)
        } else {
            searchContainer.style.display = 'none'; // Ocultar
        }
    }
    
    // Actualizar tabs
    document.querySelectorAll('.page-tab').forEach((tab, index) => {
        if (index === pageIndex) {
            tab.classList.add('border-[#F1E1A5]', 'text-[#727272]');
            tab.classList.remove('text-gray-400', 'border-transparent');
        } else {
            tab.classList.remove('border-[#F1E1A5]', 'text-[#727272]');
            tab.classList.add('text-gray-400', 'border-transparent');
        }
    });

    // Renderizar contenido
    let html = '<div class="space-y-3 md:space-y-4">';
    page.content.forEach(item => {
        // Determinar el enlace correcto basado en la página actual
        const detailPage = pageIndex === 1 ? 'routine_detail.html' : 'exercise_detail.html';

        // Añadir clase filterable-card, card-title, card-description
        html += `
            <div class="filterable-card bg-gray-50 rounded-xl p-3 md:p-4 flex items-center space-x-3 md:space-x-4">
                <img src="${item.image}" alt="${item.title}" class="w-20 h-20 md:w-24 md:h-24 rounded-lg object-cover">
                <div class="flex-1 min-w-0">
                    <h3 class="card-title text-lg md:text-xl font-semibold text-[#727272] truncate">${item.title}</h3>
                    <p class="card-description text-gray-600 text-xs md:text-sm mb-2 line-clamp-2">${item.description}</p>
                    <div class="flex items-center space-x-2">
                        <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke-width="1.5" stroke="currentColor" class="w-4 h-4 md:w-5 md:h-5 text-[#727272]">
                            <path stroke-linecap="round" stroke-linejoin="round" d="M15 10.5a3 3 0 11-6 0 3 3 0 016 0z" />
                            <path stroke-linecap="round" stroke-linejoin="round" d="M19.5 10.5c0 7.142-7.5 11.25-7.5 11.25S4.5 17.642 4.5 10.5a7.5 7.5 0 1115 0z" />
                        </svg>
                        <span class="text-xs md:text-sm text-[#727272]">${item.machine}</span>
                    </div>
                    <a href="${detailPage}" class="inline-block mt-2 bg-[#F1E1A5] text-[#727272] px-3 md:px-4 py-1 rounded-full text-xs md:text-sm font-medium hover:bg-[#F1E1A5]/80 transition-colors no-underline">
                        Ver más
                    </a>
                </div>
            </div>
        `;
    });
    html += '</div>';
    pageContent.innerHTML = html;
    
    // Re-aplicar filtro después de renderizar
    filterCurrentPage(); 
}

function changePage(direction) {
    if (direction === 'next') {
        currentPage = (currentPage + 1) % pages.length;
    } else if (direction === 'prev') {
        currentPage = (currentPage - 1 + pages.length) % pages.length;
    } else {
        currentPage = direction;
    }
    renderPage(currentPage);
}

// Función para filtrar la página actual
function filterCurrentPage() {
    const searchInput = document.getElementById('exerciseSearch'); // Usar el ID correcto del input
    if (!searchInput) {
      return;
    } // Salir si el input no existe
    
    const searchTerm = searchInput.value.toLowerCase();
    const cards = document.querySelectorAll('#pageContent .filterable-card');

    cards.forEach(card => {
        const titleElement = card.querySelector('.card-title');
        const descriptionElement = card.querySelector('.card-description');
        
        // Verificar que los elementos existan antes de acceder a textContent
        const title = titleElement ? titleElement.textContent.toLowerCase() : '';
        const description = descriptionElement ? descriptionElement.textContent.toLowerCase() : '';
        
        if (title.includes(searchTerm) || description.includes(searchTerm)) {
            card.style.display = ''; // Mostrar la tarjeta
        } else {
            card.style.display = 'none'; // Ocultar la tarjeta
        }
    });
}

// Inicializar la primera página
renderPage(0);

// Funcionalidad del buscador - Asociar al input correcto y llamar a filterCurrentPage
document.getElementById('exerciseSearch').addEventListener('input', filterCurrentPage);