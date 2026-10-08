# Reproduce everything:  make install && make run && make eval && make figures && make paper
.PHONY: install run eval figures paper clean

install:
	python -m pip install -r requirements.txt

run:            ## all systems x sequences x N runs (slow; skips finished runs)
	python -m pipeline.run_all

eval:           ## ATE / RPE / robustness -> results/metrics.csv, results/summary.csv
	python -m pipeline.evaluate

figures:        ## execute the notebooks -> paper/figures, paper/tables
	jupyter nbconvert --to notebook --execute --inplace notebooks/*.ipynb

paper:
	cd paper && latexmk -pdf main.tex

clean:
	cd paper && latexmk -C
