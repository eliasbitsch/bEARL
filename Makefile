# From committed results: make install && make figures && make paper   (from scratch: README, "Reproducing the paper")
.PHONY: install refmaps run eval mapeval cache figures replay viewer paper clean

install:
	python -m pip install -r requirements.txt

run:            ## all systems x sequences x N runs (slow; skips finished runs)
	python -m pipeline.run_all

eval:           ## ATE / robustness -> results/metrics.csv, results/summary.csv
	python -m pipeline.evaluate

refmaps:        ## LiDAR pseudo-GT maps -> data/M3DGR/<seq>/reference_map.ply (needs data/)
	for s in Dynamic01 Sha-turn01 Varying-illu01; do python -m pipeline.reference_map --sequence $$s; done

mapeval:        ## map F-score etc. -> results/map_metrics.csv
	python -m pipeline.map_eval

cache:          ## small derivatives for the notebook -> results/cache/ (needs data/)
	python tools/build_cache.py

figures:        ## story notebook -> paper/figures, paper/tables, notebooks/story.html
	python tools/write_paper_figures.py
	marimo export html notebooks/story.py -o notebooks/story.html

replay:         ## 3D replays of all systems -> results/replay/*.mcap (needs data/)
	python tools/build_replay.py

viewer:         ## local Lichtblick with the replays on :8781
	python tools/viewer.py

paper:
	cd paper && latexmk -pdf main.tex

clean:
	cd paper && latexmk -C
