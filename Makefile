pg:
	python patterngenerator.py --model simple --out-dir out/pg.simple
	python patterngenerator.py --model heicub --out-dir out/pg.heicub
	python patterngenerator.py --model simplelx5 --out-dir out/pg.simplelx5
	python patterngenerator.py --model simplelx10 --out-dir out/pg.simplelx10

all: pg
