PYTHON ?= python3
RUNTIME_ROOT ?= runtime-data
PYTHONPATH_V2 = modules/contracts:modules/calibration:modules/device_io:modules/data_capture:modules/evidence:services/map_factory:apps/site_console/backend

.PHONY: demo test ui-smoke compile container-validate knowledge knowledge-check

demo:
	PYTHONPATH=$(PYTHONPATH_V2) \
	$(PYTHON) -m gogoguard_site_console --mode demo --map-worker demo --data-root $(RUNTIME_ROOT)

test:
	PYTHONPATH=$(PYTHONPATH_V2) \
	$(PYTHON) -m unittest discover -s tests -v

compile:
	$(PYTHON) -m compileall -q modules apps services

ui-smoke:
	$(PYTHON) tools/ui_smoke.py apps/site_console/frontend

container-validate:
	$(PYTHON) tools/validate_container_contract.py deployment/container/Dockerfile deployment/robot/gogoguard-edge.service deployment/robot/install-release

knowledge:
	$(PYTHON) tools/knowledge.py write

knowledge-check:
	$(PYTHON) tools/knowledge.py check
