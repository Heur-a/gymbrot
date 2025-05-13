# Variables de configuración
PROJECT_NAME = Gymbrot
DOCS_PORT ?= 8000
DOCS_DIR = web
SPHINX_BUILD_OPTS = -W --keep-going -n
SPHINX_AUTOBUILD_OPTS = --watch ../../..

.PHONY: help install-deps setup clean html livehtml check-env

help:  ## Mostrar esta ayuda
	@awk 'BEGIN {FS = ":.*##"; printf "\nUso:\n  make \033[36m<target>\033[0m\n\nTargets:\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

install-deps:  ## Instalar dependencias
	@echo "Instalando dependencias..."
	pip install -r requirements-docs.txt

setup: check-env clean  ## Configurar entorno
	@echo "Configurando estructura inicial..."
	rm -rf $(DOCS_DIR)
	mkdir -p $(DOCS_DIR)
	cd $(DOCS_DIR) && sphinx-quickstart --quiet --project "$(PROJECT_NAME)" \
	--author "Gymbrot Team" --language "es" --extensions "sphinx.ext.autodoc,sphinx.ext.viewcode,sphinx.ext.napoleon,sphinx.ext.intersphinx,autoapi.extension,sphinxcontrib.mermaid" \
	--makefile --no-batchfile .

	@echo "\nCopiando configuración personalizada..."
	cp conf.py $(DOCS_DIR)/conf.py

clean:  ## Limpiar todo
	@echo "Limpiando build..."
	rm -rf $(DOCS_DIR)/_build
	rm -f $(DOCS_DIR)/conf.py $(DOCS_DIR)/index.rst

html: check-env  ## Generar HTML estático
	@echo "Construyendo documentación..."
	cd $(DOCS_DIR) && sphinx-build $(SPHINX_BUILD_OPTS) -D nitpicky=0 -b html -d _build/doctrees . _build/html
	@echo "\nDocumentación generada en: file://$(shell pwd)/$(DOCS_DIR)/_build/html/index.html"

livehtml: check-env  ## Servidor live
	@echo "Iniciando servidor en puerto $(DOCS_PORT)..."
	cd $(DOCS_DIR) && sphinx-autobuild -b html --port $(DOCS_PORT) --host 0.0.0.0 \
	--ignore "*.swp" --ignore "*.tmp" $(SPHINX_AUTOBUILD_OPTS) . _build/html

check-env:
	@command -v sphinx-build >/dev/null 2>&1 || { echo >&2 "Error: Sphinx no instalado. Ejecuta 'make install-deps' primero."; exit 1; }