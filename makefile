# ---------------------------
# Configuration
# ---------------------------
ifneq (,$(wildcard .env))
include .env
endif

ENV_NAME := rirun
PYTHON_VERSION ?= 3.8
CUDA_VERSION := 12.1

CONDA := conda
PIP := pip

# ---------------------------
# Phony targets
# ---------------------------
.PHONY: help env.create env.cuda install install-dev clean rebuild hooks

help:
	@echo "Available targets for RISE-Carla-Runner:"
	@echo "  make env.create    Create base conda environment"
	@echo "  make env.cuda      Install CUDA-enabled PyTorch"
	@echo "  make install       Install project (core deps)"
	@echo "  make install-dev   Install project with dev extras"
	@echo "  make hooks         Install pre-commit hooks (run after install-dev)"
	@echo "  make clean         Remove conda environment"
	@echo "  make rebuild       Clean + full reinstall"

# ---------------------------
# Conda environment
# ---------------------------
env.create:
	$(CONDA) create -y -n $(ENV_NAME) python=$(PYTHON_VERSION) pip
	@echo "→ Created environment"
	
env.cuda:
	$(CONDA) install -y -n $(ENV_NAME) \
		pytorch torchvision torchaudio pytorch-cuda=$(CUDA_VERSION) \
		-c pytorch -c nvidia
	@echo "→ Installed CUDA-enabled PyTorch"

# ---------------------------
# Project installation
# ---------------------------
install:
	$(CONDA) run -n $(ENV_NAME) $(PIP) install -e .
	@if [ -f .env ]; then . ./.env; fi; \
	PYTHON_VERSION="$${PYTHON_VERSION:-$(PYTHON_VERSION)}"; \
	if [ -n "$$CARLA_ROOT" ]; then \
		python_tag="cp$$(printf '%s' "$$PYTHON_VERSION" | tr -d .)"; \
		carla_wheel=$$(find "$$CARLA_ROOT/PythonAPI/carla/dist" -maxdepth 1 -name "carla-*-$$python_tag-$$python_tag-*.whl" | head -n 1); \
		if [ -z "$$carla_wheel" ]; then \
			echo "ERROR: Could not find $$CARLA_ROOT/PythonAPI/carla/dist/carla-*-$$python_tag-$$python_tag-*.whl"; \
			exit 1; \
		fi; \
		$(CONDA) run -n $(ENV_NAME) $(PIP) install --force-reinstall "$$carla_wheel"; \
		echo "→ Installed CARLA Python wheel: $$carla_wheel"; \
	fi
# install pre-trained weights
	@bash build-scripts/install-weights.sh
	@echo "→ Installed project (core dependencies)"

install-dev: install
	$(CONDA) run -n $(ENV_NAME) $(PIP) install -e ".[train,dev]"
	$(MAKE) hooks
	@echo "→ Installed project (development dependencies)"

hooks:
	$(CONDA) run -n $(ENV_NAME) pre-commit install
	@echo "→ Installed pre-commit hooks"

# ---------------------------
# Cleanup
# ---------------------------
clean:
# check if environment is active
	@if [ "$$CONDA_DEFAULT_ENV" = "$(ENV_NAME)" ]; then \
		echo "ERROR: Environment $(ENV_NAME) is currently active in your shell."; \
		echo "Please run 'conda deactivate' first, then run 'make clean' again."; \
		exit 1; \
	fi
# remove environment if it exists
	@if $(CONDA) env list | awk '{print $$1}' | grep -qx "$(ENV_NAME)"; then \
        $(CONDA) remove -y -n $(ENV_NAME) --all; \
        echo "→ Removed environment"; \
	fi

rebuild: clean env.create env.cuda install-dev
	@echo "→ Rebuilt complete: environment and project installation"
