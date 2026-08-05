PYTHON ?= python3
RUNTIME_ROOT ?= runtime-data

.PHONY: demo test ui-smoke compile container-validate

demo:
	PYTHONPATH=modules/contracts:modules/device_io:modules/data_capture:modules/evidence:services/map_factory:apps/site_console/backend \
	$(PYTHON) -m gogoguard_site_console --mode demo --map-worker demo --data-root $(RUNTIME_ROOT)

test:
	PYTHONPATH=modules/contracts:modules/device_io:modules/data_capture:modules/evidence:services/map_factory:apps/site_console/backend \
	$(PYTHON) -m unittest discover -s tests -v

compile:
	$(PYTHON) -m compileall -q modules apps services

ui-smoke:
	$(PYTHON) tools/ui_smoke.py apps/site_console/frontend

container-validate:
	$(PYTHON) tools/validate_container_contract.py deployment/container/Dockerfile deployment/robot/gogoguard-edge.service

