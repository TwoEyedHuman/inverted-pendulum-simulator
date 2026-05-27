VENV := .venv
PYTHON := $(VENV)/bin/python
PIP := $(VENV)/bin/pip

.PHONY: dev docker-build docker-run test preflight

$(VENV):
	python3 -m venv $(VENV)
	$(PIP) install --quiet -r requirements.txt

dev: $(VENV)
	$(VENV)/bin/uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

docker-build:
	docker build -t pendulum .

docker-run:
	docker run --rm -p 8000:8000 --env-file .env pendulum

test: $(VENV)
	$(VENV)/bin/pytest tests/; ret=$$?; [ $$ret -eq 5 ] && exit 0 || exit $$ret

preflight: $(VENV)
	@python3 --version
	@docker --version
	@$(MAKE) test
