FROM python:3.12-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates openssh-client rsync tini \
    && rm -rf /var/lib/apt/lists/*
ENV PYTHONPATH=/opt/gogoguard/app/modules/contracts:/opt/gogoguard/app/modules/calibration:/opt/gogoguard/app/modules/device_io:/opt/gogoguard/app/modules/data_capture:/opt/gogoguard/app/modules/evidence:/opt/gogoguard/app/modules/transfer:/opt/gogoguard/app/modules/route:/opt/gogoguard/app/modules/navigation:/opt/gogoguard/app/services/map_factory:/opt/gogoguard/app/apps/site_console/backend:/opt/gogoguard/app/apps/field_workstation/backend

WORKDIR /opt/gogoguard/app
COPY pyproject.toml setup.py ./
COPY modules ./modules
COPY services ./services
COPY apps ./apps
COPY config ./config
RUN python -m compileall -q modules services apps

VOLUME ["/var/lib/gogoguard-workstation"]
EXPOSE 8080
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["python", "-m", "gogoguard_field_workstation", "--host", "0.0.0.0", "--port", "8080", "--data-root", "/var/lib/gogoguard-workstation", "--config", "/etc/gogoguard/workstation.json", "--static-root", "/opt/gogoguard/app/apps/site_console/frontend"]
