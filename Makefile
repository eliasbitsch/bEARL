# Reproduce everything:  make install && make run && make eval && make figures && make paper
.PHONY: install run eval figures paper clean

install:
	python -m pip install -r requirements.txt

run:            ## all systems x sequences x N runs (slow; skips finished runs)
	python -m pipeline.run_all

eval:           ## ATE / robustness -> results/metrics.csv, results/summary.csv
	python -m pipeline.evaluate

figures:        ## story notebook -> paper/figures, paper/tables, notebooks/story.html
	python tools/write_paper_figures.py
	marimo export html notebooks/story.py -o notebooks/story.html

paper:
	cd paper && latexmk -pdf main.tex

clean:
	cd paper && latexmk -C
