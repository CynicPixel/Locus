# ─────────────────────────────────────────────────────────────
# Locus MLAT — Service Runner
# ─────────────────────────────────────────────────────────────

PUBLIC_IP   ?= 147.185.221.17
PUBLIC_PORT ?= 25677
LOCAL_PORT  ?= 61339

PIDS_DIR := .pids
LOGS_DIR := .logs

$(PIDS_DIR) $(LOGS_DIR):
	mkdir -p $@

# ── Individual services ───────────────────────────────────────

.PHONY: ingestor
ingestor: | $(PIDS_DIR) $(LOGS_DIR)
	@echo "[ingestor] starting..."
	@cd go-ingestor && go run main.go \
		--port=$(LOCAL_PORT) \
		--mode=peer \
		--buyer-or-seller=buyer \
		--list-of-sellers-source=env \
		--envFile=.env \
		--my-public-ip=$(PUBLIC_IP) \
		--my-public-port=$(PUBLIC_PORT) \
		> ../$(LOGS_DIR)/ingestor.log 2>&1 & echo $$! > ../$(PIDS_DIR)/ingestor.pid
	@echo "[ingestor] pid=$$(cat $(PIDS_DIR)/ingestor.pid)  logs → $(LOGS_DIR)/ingestor.log"

.PHONY: backend
backend: | $(PIDS_DIR) $(LOGS_DIR)
	@echo "[backend] building + starting..."
	@mkdir -p /tmp/locus
	@cd rust-backend && RUST_LOG=locus_backend=debug cargo r \
		> ../$(LOGS_DIR)/backend.log 2>&1 & echo $$! > ../$(PIDS_DIR)/backend.pid
	@echo "[backend]  pid=$$(cat $(PIDS_DIR)/backend.pid)  logs → $(LOGS_DIR)/backend.log"

.PHONY: ml
ml: | $(PIDS_DIR) $(LOGS_DIR)
	@echo "[ml]      starting..."
	@cd ml-service && source .venv/bin/activate && \
		PYTHONPATH="" uvicorn main:app --host 0.0.0.0 --port 8000 \
		> ../$(LOGS_DIR)/ml.log 2>&1 & echo $$! > ../$(PIDS_DIR)/ml.pid
	@echo "[ml]       pid=$$(cat $(PIDS_DIR)/ml.pid)  logs → $(LOGS_DIR)/ml.log"

.PHONY: frontend
frontend: | $(PIDS_DIR) $(LOGS_DIR)
	@echo "[frontend] starting..."
	@cd frontend && python3 -m http.server 3000 \
		> ../$(LOGS_DIR)/frontend.log 2>&1 & echo $$! > ../$(PIDS_DIR)/frontend.pid
	@echo "[frontend] pid=$$(cat $(PIDS_DIR)/frontend.pid)  logs → $(LOGS_DIR)/frontend.log"
	@echo "[frontend] open http://localhost:3000"

# ── Run all (no Docker) ───────────────────────────────────────

.PHONY: run
run: ingestor backend ml frontend
	@echo ""
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
	@echo "  Locus is running"
	@echo "  Frontend  → http://localhost:3000"
	@echo "  ML API    → http://localhost:8000"
	@echo "  WS engine → ws://localhost:9001"
	@echo "  Logs      → $(LOGS_DIR)/"
	@echo "  Stop with → make stop"
	@echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ── Stop all ──────────────────────────────────────────────────

.PHONY: stop
stop:
	@for svc in ingestor backend ml frontend; do \
		pid_file=$(PIDS_DIR)/$$svc.pid; \
		if [ -f $$pid_file ]; then \
			pid=$$(cat $$pid_file); \
			if kill -0 $$pid 2>/dev/null; then \
				kill $$pid && echo "[$$svc] stopped (pid=$$pid)"; \
			else \
				echo "[$$svc] already stopped"; \
			fi; \
			rm -f $$pid_file; \
		else \
			echo "[$$svc] not running"; \
		fi; \
	done

# ── Docker Compose ────────────────────────────────────────────

.PHONY: docker-up
docker-up:
	docker compose up --build -d
	@echo "Frontend → http://localhost:3000"

.PHONY: docker-down
docker-down:
	docker compose down

# ── Logs (tail all) ───────────────────────────────────────────

.PHONY: logs
logs:
	@tail -f $(LOGS_DIR)/*.log

.PHONY: logs-ingestor
logs-ingestor:
	@tail -f $(LOGS_DIR)/ingestor.log

.PHONY: logs-backend
logs-backend:
	@tail -f $(LOGS_DIR)/backend.log

.PHONY: logs-ml
logs-ml:
	@tail -f $(LOGS_DIR)/ml.log

# ── Train ML model ────────────────────────────────────────────

.PHONY: train
train:
	cd ml-service && source .venv/bin/activate && \
		PYTHONPATH="" python train.py --epochs 20 --output model.pt

# ── Status ────────────────────────────────────────────────────

.PHONY: status
status:
	@for svc in ingestor backend ml frontend; do \
		pid_file=$(PIDS_DIR)/$$svc.pid; \
		if [ -f $$pid_file ]; then \
			pid=$$(cat $$pid_file); \
			if kill -0 $$pid 2>/dev/null; then \
				echo "  ✓ $$svc (pid=$$pid)"; \
			else \
				echo "  ✗ $$svc (dead)"; \
			fi; \
		else \
			echo "  - $$svc (not started)"; \
		fi; \
	done
