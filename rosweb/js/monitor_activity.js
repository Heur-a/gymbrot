const startButton = document.getElementById('startActivity');
const endButton = document.getElementById('endActivity');
const pauseButton = document.getElementById('pauseButton');
const resumeButton = document.getElementById('resumeButton');
const timer = document.getElementById('timer');
const minutesDisplay = document.getElementById('minutes');
const secondsDisplay = document.getElementById('seconds');

let timeLeft = 5 * 60; // 5 minutos en segundos
let timerInterval;
let isPaused = false;

startButton.addEventListener('click', () => {
    startButton.textContent = 'En proceso...';
    startButton.disabled = true;
    endButton.disabled = false;
    endButton.classList.remove('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    endButton.classList.add('bg-red-500', 'text-white', 'hover:bg-red-600');
    timer.classList.remove('hidden');
    pauseButton.classList.remove('hidden');
    
    startTimer();
});

pauseButton.addEventListener('click', () => {
    if (!isPaused) {
        clearInterval(timerInterval);
        isPaused = true;
        pauseButton.classList.add('hidden');
        resumeButton.classList.remove('hidden');
    }
});

resumeButton.addEventListener('click', () => {
    if (isPaused) {
        startTimer();
        isPaused = false;
        resumeButton.classList.add('hidden');
        pauseButton.classList.remove('hidden');
    }
});

endButton.addEventListener('click', () => {
    clearInterval(timerInterval);
    resetTimer();
});

function startTimer() {
    timerInterval = setInterval(() => {
        const minutes = Math.floor(timeLeft / 60);
        const seconds = timeLeft % 60;
        
        minutesDisplay.textContent = minutes.toString().padStart(2, '0');
        secondsDisplay.textContent = seconds.toString().padStart(2, '0');
        
        if (timeLeft <= 0) {
            clearInterval(timerInterval);
            startButton.textContent = 'Tiempo completado';
            pauseButton.classList.add('hidden');
            resumeButton.classList.add('hidden');
        } else {
            timeLeft--;
        }
    }, 1000);
}

function resetTimer() {
    startButton.textContent = 'Comenzar actividad';
    startButton.disabled = false;
    endButton.disabled = true;
    endButton.classList.remove('bg-red-500', 'text-white', 'hover:bg-red-600');
    endButton.classList.add('bg-gray-300', 'text-gray-500', 'cursor-not-allowed');
    timer.classList.add('hidden');
    pauseButton.classList.add('hidden');
    resumeButton.classList.add('hidden');
    timeLeft = 5 * 60;
    minutesDisplay.textContent = '05';
    secondsDisplay.textContent = '00';
    isPaused = false;
}