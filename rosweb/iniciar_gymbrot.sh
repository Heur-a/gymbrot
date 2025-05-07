#!/usr/bin/env bash
#
# iniciar_gymbrot.sh
# Inicia servicios y prepara el entorno para tu proyecto GymBrot

set -euo pipefail

PROJECT_DIR="/home/${SUDO_USER:-$USER}/turtlebot3_ws/src/gymbrot/rosweb"
SQL_DUMP="$PROJECT_DIR/gymbrot.sql"
DB_NAME="gymbrot"
DB_USER="root"
WEB_ROOT="/var/www/html/rosweb"

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


# Eliminar la base de datos si existe
echo "🗑️  Eliminando la base de datos '$DB_NAME' si existe..."
mysql -u"$DB_USER" -p"$DB_PASS" -e "DROP DATABASE IF EXISTS $DB_NAME;"

# Crear la base de datos y ejecutar el volcado SQL
echo "📥 Importando volcado desde $SQL_DUMP..."
mysql -u"$DB_USER" -p"$DB_PASS" < "$SQL_DUMP"
echo "✔️  Importación completada."

# Limpieza del directorio web:

rm -rf "$WEB_ROOT"/

# Copia de los archivos

mkdir -p "$WEB_ROOT"

cp -r "$PROJECT_DIR"/* "$WEB_ROOT"/


#Permisos en el web root:
echo "🔧 Ajustando permisos en $WEB_ROOT..."
chown -R www-data:www-data "$WEB_ROOT"
chmod -R 755 "$WEB_ROOT"

echo "✅ GymBrot está listo. Accede en http://localhost/rosweb/html/login.html"