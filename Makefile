PYTHON ?= python3
RUNTIME_ROOT ?= runtime-data
PYTHONPATH_V2 = modules/contracts:modules/calibration:modules/device_io:modules/interaction:modules/inspection:modules/mission:modules/data_capture:modules/evidence:modules/transfer:modules/route:modules/navigation:services/map_factory:services/interaction_edge:services/platform_edge:apps/site_console/backend:apps/field_workstation/backend

.PHONY: demo workstation workstation-demo workstation-container test ui-smoke compile container-validate knowledge knowledge-check

demo:
	PYTHONPATH=$(PYTHONPATH_V2) \
	$(PYTHON) -m gogoguard_site_console --mode demo --map-worker demo --data-root $(RUNTIME_ROOT)

workstation-demo:
	PYTHONPATH=$(PYTHONPATH_V2) \
	$(PYTHON) -m gogoguard_field_workstation --map-worker demo --data-root workstation-data

workstation:
	deployment/workstation/run-native

workstation-container:
	docker compose -f deployment/workstation/compose.yaml up --build -d

test:
	PYTHONPATH=$(PYTHONPATH_V2) \
	$(PYTHON) -m unittest discover -s tests -v

compile:
	$(PYTHON) -m compileall -q modules apps services

ui-smoke:
	$(PYTHON) tools/ui_smoke.py apps/site_console/frontend

container-validate:
	$(PYTHON) tools/validate_container_contract.py deployment/container/Dockerfile deployment/robot/gogoguard-edge.service deployment/robot/install-release deployment/cloud/gogoguard-map-job

knowledge:
	$(PYTHON) tools/knowledge.py write

knowledge-check:
	$(PYTHON) tools/knowledge.py check
