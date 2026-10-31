# X2 digital twin — single entry point.   make help
SHELL := /bin/bash
COMPOSE := docker compose -f docker/compose.yaml --env-file $(or $(TWIN_ENVFILE),$(wildcard .env),.env.example)
# navigation stack source: sibling control_backend/ (dev workspace) or the repo root (this directory inside qazbot-x2-stack)
BACKEND := $(if $(wildcard ../control_backend/ws),../control_backend,..)
export TWIN_BACKEND := ../$(BACKEND)
RUN_NAME ?= run-$(shell date +%Y%m%d-%H%M%S)
export RUN_NAME

.DEFAULT_GOAL := help
.PHONY: nightly test-i1 scenario-i1 scenario-turn scenario-truth help doctor vendor vendor-verify build test lint dev-image shell run pytest up down logs ps scenario lite-up lite-scenario results clean

help:            ## this list
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  %-16s %s\n", $$1, $$2}'

doctor:          ## check the host (docker, compose, vendor artifacts, disk)
	@scripts/doctor.sh

vendor:          ## extract the AgiBot archives (ZIPS=dir with the 4 zips, default $(ZIPS), env TWIN_ARCHIVES) into vendor/ with sha256 verification
	@scripts/fetch_vendor.sh $(ZIPS)

vendor-verify:   ## verify vendor/ against vendor/manifest.sha256
	@scripts/fetch_vendor.sh --verify

build:           ## build the images (sim, stack)
	$(COMPOSE) build sim stack

test:            ## unit tests inside the build container (fails the build if a test fails)
	docker build -f docker/Dockerfile --target test --build-context vendor=vendor --build-context backend=$(BACKEND) -t x2-twin-test:dev .

dev-image:       ## build the dev image (stage `test`; all development runs in it, nothing on the host)
	docker build -f docker/Dockerfile --target test --build-context vendor=vendor --build-context backend=$(BACKEND) -t x2-twin-test:dev .

shell:           ## interactive shell in the dev container (source bind-mounted)
	$(COMPOSE) --profile dev run --rm dev bash

run:             ## run a command in the dev container: make run CMD="python3 -m pytest -q x2_twin_sim/test"
	$(COMPOSE) --profile dev run --rm dev bash -lc '. /opt/ros/$${ROS_DISTRO}/setup.sh; . /opt/x2twin/overlay/setup.sh; export PYTHONPATH=/ws/src/twin/x2_twin_sim:/ws/src/twin/x2_twin_bringup:/ws/src/twin/x2_twin_scenarios:$$PYTHONPATH; $(CMD)'

lint:            ## flake8 inside the dev container (project style, 140 columns)
	$(COMPOSE) --profile dev run --rm dev python3 -m flake8 --config /ws/src/twin/.flake8 /ws/src/twin

pytest:          ## unit tests inside the dev container against the live source
	$(MAKE) run CMD="python3 -m pytest -q -p no:cacheprovider x2_twin_sim/test x2_twin_bringup/test x2_twin_scenarios/test"

up:              ## sim + stack in the background (headless); waits until healthy
	$(COMPOSE) up -d --wait sim stack

down:            ## stop and remove containers and the network (volumes kept)
	$(COMPOSE) --profile test --profile lite down

logs:            ## follow all logs
	$(COMPOSE) logs -f --tail=100

ps:              ## status and health
	$(COMPOSE) ps

scenario-truth:  ## A2: same scenario with ground-truth localisation (AMCL + Trust off): isolates controller and MC
	@TWIN_LOCALIZATION=truth $(MAKE) up scenario RUN_NAME=$(RUN_NAME)

test-i1:         ## I1 candidate unit tests (scan semantics) in the dev container
	$(MAKE) run CMD="python3 -m pytest -q -p no:cacheprovider /work/candidates/i1_rpp_collision/test_level_scan.py x2_twin_sim/test"

scenario-i1:     ## I1 A/B (legacy vs candidate) in the twin, then the verdict: make scenario-i1 TAG=x
	scripts/i1/run_ab.sh $(or $(TAG),$(shell date +%H%M%S))
	$(MAKE) run CMD="python3 /work/scripts/i1/summarize.py $(or $(TAG),$(shell date +%H%M%S))"

scenario-turn:   ## A/B: SIM-only MC turn regulator (candidate, wmax 0.8) with ground-truth localisation
	@TWIN_LOCALIZATION=truth TWIN_MC_TURN=1 MAX_ANGULAR=0.8 $(MAKE) up scenario RUN_NAME=$(RUN_NAME)

scenario:        ## operator session against the running official-MC stack -> results/$(RUN_NAME)/results.json
	$(COMPOSE) --profile test run --rm scenario
	@echo "results: results/$(RUN_NAME)/results.json"

lite-up:         ## lightweight mode (virtual MC + kinematic plant, no MuJoCo)
	$(COMPOSE) --profile lite up -d --wait stack-lite

lite-scenario:   ## the same scenario against the lightweight mode
	$(COMPOSE) --profile lite run --rm scenario-lite

nightly:         ## current SHA-pinned D1/P23 campaign through the existing queue authorization (historical TSV is not automatic)
	../tools/nightly/enqueue_nightly.sh

results:         ## summarize results/ (PREFIX=run)
	$(COMPOSE) --profile test run --rm --no-deps scenario ros2 run x2_twin_scenarios summarize $(or $(PREFIX),run)

clean:           ## remove containers, volumes (re-seeds vendor copies) and dangling build cache of this project
	$(COMPOSE) --profile test --profile lite down -v

# Runtime entrypoints cannot bypass the queue's capabilities. Offline build/test targets remain available.
.PHONY: runtime-authorized
runtime-authorized:
	@python3 ../tools/nightly/execution.py L3_QUALIFICATION
up scenario scenario-truth scenario-turn scenario-i1 lite-up lite-scenario: runtime-authorized
