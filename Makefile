pg:
	python patterngenerator.py

lin-newton-noi-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton1.noi filter --iterations 1 --filter-method newton --ik-method analytical --interpolate none

lin-newton-noi-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton5.noi filter --iterations 5 --filter-method newton --ik-method analytical --interpolate none

lin-newton-noi: lin-newton-noi-1 lin-newton-noi-5

lin-newton-savgol-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton1.sg filter --iterations 1 --filter-method newton --ik-method analytical --interpolate savgol

lin-newton-savgol-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton2.sg filter --iterations 2 --filter-method newton --ik-method analytical --interpolate savgol

lin-newton-savgol-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/newton5.sg filter --iterations 5 --filter-method newton --ik-method analytical --interpolate savgol

lin-newton-savgol: lin-newton-savgol-1 lin-newton-savgol-2 lin-newton-savgol-5

lin-newton: lin-newton-noi lin-newton-savgol

lin-leastsquares-noi-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares1.noi filter --iterations 1 --filter-method leastsquares --ik-method analytical

lin-leastsquares-noi-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares2.noi filter --iterations 2 --filter-method leastsquares --ik-method analytical

lin-leastsquares-noi-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares5.noi filter --iterations 5 --filter-method leastsquares --ik-method analytical

lin-leastsquares-noi: lin-leastsquares-noi-1 lin-leastsquares-noi-2 lin-leastsquares-noi-5

lin-leastsquares-savgol-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares1.sg filter --iterations 1 --filter-method leastsquares --ik-method analytical --interpolate savgol

lin-leastsquares-savgol-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares2.sg filter --iterations 2 --filter-method newton --ik-method analytical --interpolate savgol

lin-leastsquares-savgol-5:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/leastsquares5.sg filter --iterations 5 --filter-method newton --ik-method analytical --interpolate savgol

lin-leastsquares-savgol: lin-leastsquares-savgol-1 lin-leastsquares-savgol-2 lin-leastsquares-savgol 5

lin-pc-1:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/pc1 filter --iterations 1 --filter-method pc --ik-method analytical

lin-pc-2:
	python main.py --trajectory out/pg.linear/pg_data.txt --out-dir out/pc2 filter --iterations 2 --filter-method pc --ik-method analytical

newton: lin-newton-noi lin-newton-savgol

leastsquares: lin-leastsquares-noi lin-leastsquares-savgol

pc: lin-pc-1 lin-pc-2

all: pg newton leastsquares pc