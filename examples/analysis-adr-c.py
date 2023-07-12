import concurrent.futures
import matplotlib.pyplot as plt
import math
import numpy as np
import os
import pandas as pd
import time

ns3_dir = os.getcwd()
ns3 = f'{ns3_dir}/./ns3'

def calcPdr(adr, num_reps=30):
	files = [f'{ns3_dir}/phyPerformance102ADR-{i}.csv' for i in range(1, num_reps+1)]
	x_s = [pd.read_csv(file)['received'].mean() for file in files]
	adr['PDR'] = np.mean(x_s)
	adr['PDR_std'] = np.std(x_s)

def calcEnergy(adr, num_reps=30):
	files = [f'{ns3_dir}/battery-level-component-{i}.txt' for i in range(1, num_reps+1)]
	x_s = [(10000 - pd.read_csv(file, names=['num', 'energy'])['energy'].mean()) for file in files]
	adr['Energy'] = np.mean(x_s)
	adr['Energy_std'] = np.std(x_s)

def execute(run):
	cmd = f"{ns3} run \"scenario102  --nDevices=136 --intervalTx=15 --nRun={run}\""
	os.system(cmd)

def compile_now():
	cmd = f'{ns3} configure -d optimized --enable-tests --enable-examples --enable-mpi'
	os.system(cmd)

def simulate(adr, num_reps=30):
	executor = concurrent.futures.ThreadPoolExecutor()
	
	num_threads, start, end = 6, 0, 0
	its = math.floor(num_reps / num_threads)
	if its > 0:
		for i in range(0, its):
			start = i * num_threads + 1
			end = start + num_threads if i < its-1 else start + (num_reps - start) + 1
			compile_now()
			results = [executor.submit(execute, i) for i in range(start, end)]
			concurrent.futures.wait(results)
	else:
		compile_now()
		results = [executor.submit(execute, i) for i in range(1, num_reps+1)]
		concurrent.futures.wait(results)

	executor.shutdown()

def calc(adr, num_reps=30):
	executor = concurrent.futures.ThreadPoolExecutor()
	results = [executor.submit(calcPdr, adr, num_reps), executor.submit(calcEnergy, adr, num_reps)]
	concurrent.futures.wait(results)
	executor.shutdown()
	print(f'PDR = {adr["PDR"]}, Energy = {adr["Energy"]}')

def snr(adr, num_reps = 30):
	files = [f'{ns3_dir}/snr{i}.csv' for i in range(1, num_reps+1)]
	x_s, mins, maxs, means = [], [], [], []
	for file in files:
		data = pd.read_csv(file, names=['snr', 'snr_req', 'snr_margin'])
		x_s.append(data['snr_margin'].mean())
		snr_describe = data['snr_margin'].describe()
		mins.append(snr_describe.min) 
		maxs.append(snr_describe.max)
		means.append(snr_describe.mean)
	
	adr['SNR'] = np.mean(x_s)
	adr['SNR_std'] = np.std(x_s)
	mins.sort()
	maxs.sort(reverse=True)
	print(f'Lista de min_snr = {mins}')
	print(f'Lista de max_snr = {mins}')
	print(f'Lista de mean_snr = {means}')

if __name__ == '__main__':
	start = time.time()
	adr = {
		'PDR': -np.inf,
		'PDR_std': 0,
		'Energy': -np.inf,
		'Energy_std': 0,
		'SNR': -np.inf,
		'SNR_std': 0
	}
	num_reps = 120
	simulate(adr,num_reps)
	calc(adr,num_reps)
	snr(adr, num_reps)
	end = time.time()
	
	print(f'Tempo de execução = {end-start} segundos <-> {(end-start)/60} min')
	print(adr)
