#!/usr/bin/env bash
#
# iniciar_gymbrot.sh
# Inicia servicios y prepara el entorno para tu proyecto GymBrot

set -euo pipefail

PROJECT_DIR="/var/www/html/rosweb"
SQL_DUMP="$PROJECT_DIR/gymbrot.sql"
DB_NAME="gymbrot"
DB_USER="root"

# Comprueba que se ejecute como root
if [[ $EUID -ne 0 ]]; then
  echo "⚠️  Este script debe ejecutarse con sudo o como root."
  exit 1
fi

# Pedir la contraseña de root al usuario
read -s -p "Introduce la contraseña de root para MySQL: " DB_PASS
echo

echo "🔄 Reiniciando servicios..."
systemctl restart apache2
systemctl restart mariadb || systemctl restart mysql

echo "🔧 Ajustando permisos en $PROJECT_DIR..."
chown -R www-data:www-data "$PROJECT_DIR"
chmod -R 755 "$PROJECT_DIR"

# Eliminar la base de datos si existe
echo "🗑️  Eliminando la base de datos '$DB_NAME' si existe..."
mysql -u"$DB_USER" -p"$DB_PASS" -e "DROP DATABASE IF EXISTS $DB_NAME;"

# Crear la base de datos y ejecutar el volcado SQL
echo "📥 Importando volcado desde $SQL_DUMP..."
mysql -u"$DB_USER" -p"$DB_PASS" < "$SQL_DUMP"
echo "✔️  Importación completada."

# Insertar datos de ejemplo en las tablas
echo "📝 Insertando datos de ejemplo en las tablas..."
mysql -u"$DB_USER" -p"$DB_PASS" $DB_NAME << EOF
INSERT INTO exercises (id, name, description, machine, muscle, video, image, task) VALUES
(1, 'Press de banca', 'El press de banca es un ejercicio fundamental para desarrollar la fuerza y masa muscular del pecho. Se realiza acostado en un banco horizontal, sujetando una barra con las manos separadas al ancho de los hombros. Descender la barra controladamente hasta tocar el pecho y luego empujar hacia arriba manteniendo la estabilidad corporal. Trabaja principalmente pectorales, deltoides anteriores y tríceps.', 1, 1, 'https://www.youtube.com/watch?v=vcBig73ojpE', 'https://www.fisioterapiaconmueve.com/wp-content/uploads/2018/04/1.jpg', 'Realizar 4 series progresivas: \r\n1. 12 repeticiones con 50% RM \r\n2. 10 repeticiones con 70% RM \r\n3. '),
(2, 'Yoga Vinyasa', 'Práctica dinámica de yoga que sincroniza movimiento y respiración. Incluye secuencias fluidas como el saludo al sol, posturas de equilibrio y torsiones. Mejora flexibilidad, equilibrio y concentración. Ideal para movilidad articular y recuperación activa.', 2, 4, 'https://www.youtube.com/watch?v=4vTJGUQ5QDA', 'https://hips.hearstapps.com/hmg-prod/images/yoga-vinyasa-portada-womenshealth-1622106042.jpg', 'Sesión de 45 minutos:\r\n- 10 min calentamiento con saludo al sol\r\n- 25 min secuencia de posturas (gue'),
(3, 'Sentadillas con barra', 'Ejercicio rey para desarrollo de piernas y glúteos. Colocar la barra sobre trapecios, mantener espalda neutral y descender flexionando rodillas hasta paralelo. Enfocarse en técnica perfecta para prevenir lesiones. Trabaja cuádriceps, isquiotibiales y glúteos mayor.', 3, 5, 'https://www.youtube.com/watch?v=Dy28eq2PjcM', 'https://vitarclub.es/wp-content/uploads/2023/01/close-up-on-woman-doing-crossfit-workout-scaled.jpg', 'Pirámide de intensidad:\r\n1. 15 repeticiones con barra vacía\r\n2. 12 repeticiones +20kg\r\n3. 10 repetic'),
(4, 'Press de banca', 'El press de banca es un ejercicio fundamental para desarrollar la fuerza y masa muscular del pecho. Se realiza acostado en un banco horizontal, sujetando una barra con las manos separadas al ancho de los hombros. Descender la barra controladamente hasta tocar el pecho y luego empujar hacia arriba manteniendo la estabilidad corporal. Trabaja principalmente pectorales, deltoides anteriores y tríceps.', 1, 1, 'https://www.youtube.com/watch?v=vcBig73ojpE', 'https://blogscdn.thehut.net/app/uploads/sites/450/2020/08/militari-press_1598867922.jpg', 'Series: 4 progresivas | Reps: 12-10-8-6 | Carga: 50%-90% RM | Descanso: 90s'),
(5, 'Yoga Vinyasa', 'Práctica dinámica de yoga que sincroniza movimiento y respiración. Incluye secuencias fluidas como el saludo al sol, posturas de equilibrio y torsiones. Mejora flexibilidad, equilibrio y concentración. Ideal para movilidad articular y recuperación activa.', 2, 4, 'https://www.youtube.com/watch?v=4vTJGUQ5QDA', 'https://ignisfisioterapiagirona.cat/wp-content/uploads/2020/01/pilates-girona.jpg', 'Duración: 45min | Frecuencia: 3x/semana | Componentes: Calentamiento, posturas, relajación'),
(6, 'Sentadillas con barra', 'Ejercicio rey para desarrollo de piernas y glúteos. Colocar la barra sobre trapecios, mantener espalda neutral y descender flexionando rodillas hasta paralelo. Enfocarse en técnica perfecta para prevenir lesiones. Trabaja cuádriceps, isquiotibiales y glúteos mayor.', 3, 5, 'https://www.youtube.com/watch?v=Dy28eq2PjcM', 'https://vitruve.fit/wp-content/uploads/2024/05/Main-differences-Between-Squats-Deadlifts-1.jpg', 'Pirámide: 15-12-10-8 reps | Carga: +0kg/+20kg/+40kg/+60kg | Descanso: 2min');

INSERT INTO machines (id, locX, locY, orientation) VALUES
(1, 12.5, 8.3, 90),
(2, 15, 3.7, 180),
(3, 7.2, 10.1, 45);

INSERT INTO muscles (id, name) VALUES
(1, 'Pectorales'),
(2, 'Bíceps'),
(3, 'Tríceps'),
(4, 'Abdominales'),
(5, 'Cuádriceps');

INSERT INTO routine (id, name) VALUES
(1, 'Rutina tren superior'),
(2, 'Rutina tren inferior'),
(3, 'Rutina full body');

INSERT INTO routine_exercise (id, routine, exercise) VALUES
(4, 1, 1),
(5, 1, 3),
(6, 2, 2);

INSERT INTO users (id, email, password, user_type) VALUES
(1, 'admin@gymbrot.com', 'admin123', 1),
(2, 'usuario1@gymbrot.com', 'user123', 2);

INSERT INTO user_type (id, rol) VALUES
(1, 'Admin'),
(2, 'Usuario');
EOF
echo "✔️  Datos de ejemplo insertados correctamente."

# Limpieza del directorio web:

rm -rf "$WEB_ROOT"/*

# Copia de los archivos

cp -r "$SOURCE_DIR"/rosweb/* "$WEB_ROOT"/


# Nuevas variables de rutas:

SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WEB_ROOT="/var/www/html"
PROJECT_DIR="$WEB_ROOT"

echo "✅ GymBrot está listo. Accede en http://localhost/html/login.html"