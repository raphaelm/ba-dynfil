pg:
	python patterngenerator.py --model simple --out-dir out/pg.simple
	python patterngenerator.py --model heicub --out-dir out/pg.heicub
	python patterngenerator.py --model simplelx2 --out-dir out/pg.simplelx2
	python patterngenerator.py --model simplelx5 --out-dir out/pg.simplelx5
	python patterngenerator.py --model simplelx10 --out-dir out/pg.simplelx10

evaluate-simple:
	python main.py --trajectory out/pg.simple/pg_data.txt --model simple --out-dir out/simple/ evaluate --ik-method analytical

evaluate-heicub:
	python main.py --trajectory out/pg.heicub/pg_data.txt --model heicub --out-dir out/heicub/ evaluate --ik-method numerical

all: pg evaluate-simple evaluate-heicub
