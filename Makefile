# Developer entry points. Everything here is CPU-only and GPU-free (ADR 0003).
# Dataset downloads are cached under $(SLAM_DATA_DIR) (default ./data, git-ignored).

SHELL := /bin/bash
VENV := eval/.venv
PY := $(VENV)/bin/python
SLAM_DATA_DIR ?= $(CURDIR)/data

.DEFAULT_GOAL := help

.PHONY: help
help: ## Show this help
	@grep -hE '^[a-zA-Z0-9_-]+:.*?## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-22s\033[0m %s\n", $$1, $$2}'

# ---- Build & test ---------------------------------------------------------------------

.PHONY: build
build: ## Build the whole workspace (release)
	cargo build --release --workspace

.PHONY: test
test: test-rust test-py ## Run all tests (Rust + Python harness)

.PHONY: test-rust
test-rust: ## Run Rust tests
	cargo test --workspace

.PHONY: test-py
test-py: $(VENV) ## Run Python harness tests (builds bag tool first)
	cargo build --release -p slam-datasets
	cd eval && . .venv/bin/activate && python -m pytest

.PHONY: fmt
fmt: ## Check Rust formatting
	cargo fmt --all --check

.PHONY: clippy
clippy: ## Lint (warnings are errors)
	cargo clippy --workspace --all-targets -- -D warnings

.PHONY: bench
bench: $(VENV) ## Run the gated end-to-end self-test benchmark (synthetic, no GPU)
	cargo build --release -p slam-replay
	cd eval && . .venv/bin/activate && python -m harness.selftest

# ---- Python environment ---------------------------------------------------------------

.PHONY: setup
setup: $(VENV) ## Create the Python venv and install harness deps

$(VENV): eval/requirements.txt
	python3 -m venv $(VENV)
	$(PY) -m pip install --upgrade pip
	$(PY) -m pip install -r eval/requirements.txt
	@touch $(VENV)

# ---- Dataset download + cache ---------------------------------------------------------

.PHONY: data-euroc
data-euroc: $(VENV) ## Download the EuRoC collection for SEQ (SEQ=MH_01_easy) — large
	@test -n "$(SEQ)" || (echo "usage: make data-euroc SEQ=MH_01_easy" && false)
	cd eval && SLAM_DATA_DIR=$(SLAM_DATA_DIR) . .venv/bin/activate && python -m harness.fetch euroc $(SEQ)

.PHONY: data-openloris
data-openloris: $(VENV) ## Download an OpenLORIS scene (SCENE=office1) — large
	@test -n "$(SCENE)" || (echo "usage: make data-openloris SCENE=office1" && false)
	cd eval && SLAM_DATA_DIR=$(SLAM_DATA_DIR) . .venv/bin/activate && python -m harness.fetch openloris $(SCENE)

.PHONY: data-openloris-gt
data-openloris-gt: $(VENV) ## Download the OpenLORIS ground-truth bundle (~11 MB)
	cd eval && SLAM_DATA_DIR=$(SLAM_DATA_DIR) . .venv/bin/activate && python -m harness.fetch openloris-gt
