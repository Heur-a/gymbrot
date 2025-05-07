function createLogoutPopup() {
    // Crear el overlay
    const overlay = document.createElement('div');
    overlay.className = 'fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center';
    overlay.id = 'logout-overlay';

    // Crear el popup
    const popup = document.createElement('div');
    popup.className = 'bg-white rounded-lg p-6 max-w-sm w-full mx-4 shadow-xl';
    popup.innerHTML = `
        <div class="text-center">
            <h3 class="text-lg font-semibold text-gray-900 mb-4">¿Estás seguro de que quieres cerrar sesión?</h3>
            <div class="flex justify-center space-x-4">
                <button id="cancel-logout" class="px-4 py-2 bg-gray-200 text-gray-800 rounded-lg hover:bg-gray-300 transition-colors">
                    Cancelar
                </button>
                <button id="confirm-logout" class="px-4 py-2 bg-[#F1E1A5] text-gray-800 rounded-lg hover:bg-[#F1E1A5]/80 transition-colors">
                    Cerrar sesión
                </button>
            </div>
        </div>
    `;

    // Añadir el popup al overlay
    overlay.appendChild(popup);
    document.body.appendChild(overlay);

    // Event listeners
    document.getElementById('cancel-logout').addEventListener('click', () => {
        document.body.removeChild(overlay);
    });

    document.getElementById('confirm-logout').addEventListener('click', () => {
        window.location.href = '../php/logout.php';
    });
}

// Añadir el evento al botón de perfil
document.addEventListener('DOMContentLoaded', () => {
    const profileButton = document.getElementById('profile-button');
    if (profileButton) {
        profileButton.addEventListener('click', (e) => {
            e.preventDefault();
            createLogoutPopup();
        });
    }
}); 