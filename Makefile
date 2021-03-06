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

evaluate-speed:
	python main.py --trajectory out/pg.simple/pg_data.txt --model simple --out-dir out/simple/ evaluate_speed --ik-method analytical

evaluate-accuracy:
	python main.py --trajectory out/pg.simple/pg_data.txt --model simple --out-dir out/zmpaccuracy.simple/ evaluate_zmp_accuracy
	python main.py --trajectory out/pg.heicub/pg_data.txt --model heicub --out-dir out/zmpaccuracy.heicub/ evaluate_zmp_accuracy
	python main.py --trajectory out/pg.simplelx5/pg_data.txt --model simplelx5 --out-dir out/zmpaccuracy.simplelx5/ evaluate_zmp_accuracy

all: pg evaluate-simple evaluate-heicub evaluate-speed evaluate-accuracy
