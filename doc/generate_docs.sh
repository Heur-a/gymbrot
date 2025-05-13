#!/usr/bin/env bash

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuración
DOCS_PORT=8000
DOCS_DIR="web"
REQUIREMENTS="requirements-docs.txt"

check_dependencies() {
    local missing=0
    for cmd in python3 pip3 make; do
        if ! command -v $cmd &> /dev/null; then
            echo -e "${RED}Error: $cmd no está instalado${NC}"
            missing=1
        fi
    done
    [ $missing -eq 1 ] && exit 1
}

install_deps() {
    echo -e "${YELLOW}Instalando dependencias...${NC}"
    if ! pip3 install -r "$REQUIREMENTS"; then
        echo -e "${RED}Error instalando dependencias${NC}"
        exit 1
    fi
}

setup_docs() {
    echo -e "${YELLOW}Configurando documentación...${NC}"
    make setup
    cp _static/* "$DOCS_DIR/_static/" 2>/dev/null || :
}

build_docs() {
    echo -e "${YELLOW}Construyendo documentación...${NC}"
    make html
}

serve_docs() {
    echo -e "${GREEN}Iniciando servidor en http://localhost:$DOCS_PORT${NC}"
    make livehtml DOCS_PORT=$DOCS_PORT
}

clean_docs() {
    echo -e "${YELLOW}Limpiando builds...${NC}"
    make clean
}

deploy_docs() {
    build_docs
    echo -e "${YELLOW}Desplegando documentación...${NC}"
    # Añadir comandos de despliegue específicos
}

usage() {
    echo -e "${GREEN}Uso: $0 [opción]${NC}"
    echo "Opciones:"
    echo "  install     Instalar dependencias"
    echo "  setup       Configurar estructura de documentación"
    echo "  build       Generar documentación HTML"
    echo "  serve       Iniciar servidor de desarrollo"
    echo "  deploy      Desplegar documentación"
    echo "  clean       Eliminar archivos generados"
    echo "  all         Ejecutar install, setup, build y serve"
    exit 1
}

main() {
    check_dependencies

    case $1 in
        install) install_deps ;;
        setup) setup_docs ;;
        build) build_docs ;;
        serve) serve_docs ;;
        deploy) deploy_docs ;;
        clean) clean_docs ;;
        all)
            clean_docs
            install_deps
            setup_docs
            build_docs
            serve_docs
            ;;
        *) usage ;;
    esac
}

main "$@"