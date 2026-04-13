PACKAGES := rover_sim rover_drive rover_sim_emulator rover_sim_startup

.PHONY: help install dev test clean lint

help:
	@echo "PolarRover Software — development targets"
	@echo ""
	@echo "  make install   install all four packages in editable mode"
	@echo "  make dev       install with dev extras (pytest, etc.)"
	@echo "  make test      run tests for all packages"
	@echo "  make clean     remove __pycache__ and build artifacts"
	@echo ""
	@echo "Packages: $(PACKAGES)"

install:
	@for pkg in $(PACKAGES); do \
		echo "==> installing $$pkg"; \
		pip install -e ./$$pkg || exit 1; \
	done

dev:
	@for pkg in $(PACKAGES); do \
		echo "==> installing $$pkg [dev]"; \
		pip install -e "./$$pkg[dev]" || exit 1; \
	done

test:
	@for pkg in $(PACKAGES); do \
		echo "==> testing $$pkg"; \
		(cd $$pkg && python -m pytest tests) || exit 1; \
	done

clean:
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
	find . -type d -name '*.egg-info' -prune -exec rm -rf {} +
	find . -type d -name .pytest_cache -prune -exec rm -rf {} +
