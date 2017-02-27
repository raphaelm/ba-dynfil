pg:
	python patterngenerator.py

lin-newton-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton1 filter --iterations 1 --filter-method newton --ik-method analytical --interpolate savgol

lin-newton-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton2 filter --iterations 2 --filter-method newton --ik-method analytical --interpolate savgol

lin-newton-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton5 filter --iterations 5 --filter-method newton --ik-method analytical --interpolate savgol

lin-leastsquares-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares1 filter --iterations 1 --filter-method leastsquares --ik-method analytical --interpolate savgol

lin-leastsquares-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares2 filter --iterations 2 --filter-method newton --ik-method analytical --interpolate savgol

lin-leastsquares-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares5 filter --iterations 5 --filter-method newton --ik-method analytical --interpolate savgol

lin-pc-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/pc1 filter --iterations 1 --filter-method pc --ik-method analytical

lin-pc-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/pc2 filter --iterations 2 --filter-method pc --ik-method analytical


newton: lin-newton-1 lin-newton-2 lin-newton-5

leastsquares: lin-leastsquares-1 lin-leastsquares-2 lin-leastsquares-5

pc: lin-pc-1 lin-pc-2

all: pg newton