#!/usr/bin/python
import json
import matplotlib.pyplot as plt
import numpy as np
import os 
import pandas as pd
import concurrent.futures

def calcPlr(particle):
	files = [
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/phyPerformance102FADR-1.csv',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/phyPerformance102FADR-2.csv',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/phyPerformance102FADR-3.csv',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/phyPerformance102FADR-4.csv'
	]
	particle['PLR'] = sum((1 - pd.read_csv(file)['pdr'].mean()) for file in files) / len(files)

def calcEnergy(particle):
	files = [
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/battery-level-component-1.txt',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/battery-level-component-2.txt',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/battery-level-component-3.txt',
    '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/battery-level-component-4.txt'
	]
	particle['Energy'] = \
		sum(((10000 - pd.read_csv(file, names=['num', 'energy'])['energy'].mean()) / 10000) for file in files) / len(files)

def calc(particle):
	calcPlr(particle)
	calcEnergy(particle)

def fillFll(position, fll='fadr.fll'):
	tp = ['low', 'average', 'high']
	sf = ['A', 'B', 'C']
	filename = \
		f'/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/src/lorawan/examples/{fll}'
	with open(filename, 'w') as file:
		file.write(\
			'Engine: FADR\n' +
			'\tdescription: An Engine for ADR LoRaWAN\n' +
			'InputVariable: snr\n'+
			'\tdescription: Signal Noise Ratio\n'
			'\tenabled: true\n'+
			'\trange: -4.0  30.0\n'+
			'\tlock-range: true\n'+
			f'\tterm: poor       Triangle {position[0]} {position[1]} {position[2]}\n'+
			f'\tterm: acceptable Triangle {position[3]} {position[4]} {position[5]}\n'+
			f'\tterm: good       Triangle {position[6]} {position[7]} {position[8]}\n'+
			'OutputVariable: tp\n'+
			'\tdescription: TP based on Mamdani inference\n'+
			'\tenabled: true\n'+
			'\trange:   2  14\n'+
			'\tlock-range: false\n'+
			'\taggregation: Maximum\n'+
			'\tdefuzzifier: Centroid 10\n'+
			'\tdefault: nan\n'+
			'\tlock-previous: false\n'+
			f'\tterm:  low      Triangle  {position[9]}  {position[10]} {position[11]}\n'+
			f'\tterm:  average  Triangle  {position[12]} {position[13]} {position[14]}\n'+
			f'\tterm:  high     Triangle  {position[15]} {position[16]} {position[17]}\n'+
			'OutputVariable: sf\n'+
			'\tdescription: SF based on Mamdani inference\n'+
			'\tenabled: true\n'+
			'\trange: 7  12\n'+
			'\tlock-range: false\n'+
			'\taggregation: Maximum\n'+
			'\tdefuzzifier: Centroid 10\n'+
			'\tdefault: nan\n'+
			'\tlock-previous: false\n'+
			f'\tterm:  A    Triangle  {position[18]} {position[19]} {position[20]}\n'+
			f'\tterm:  B    Triangle  {position[21]} {position[22]} {position[23]}\n'+
			f'\tterm:  C    Triangle  {position[24]} {position[25]} {position[26]}\n'+
			'RuleBlock: mamdani\n'+
			'\tdescription: Mamdani Inference for TP\n'+
			'\tenabled: true\n'+
			'\tconjunction: Minimum\n'+
			'\tdisjunction: Maximum\n'+
			'\timplication: AlgebraicProduct\n'+
			'\tactivation: General\n'+
			f'\trule: if snr is poor then tp is {tp[int(position[27])]}\n'+
			f'\trule: if snr is acceptable then tp is {tp[int(position[28])]}\n'+
			f'\trule: if snr is good then tp is {tp[int(position[29])]}\n'+
			'RuleBlock: mamdani\n'+
			'\tdescription: Mamdani Inference for SF\n'+
			'\tenabled: true\n'+
			'\tconjunction: Minimum\n'+
			'\tdisjunction: Maximum\n'+
			'\timplication: AlgebraicProduct\n'+
			'\tactivation: General\n'+
			f'\trule: if snr is poor then sf is {sf[int(position[30])]}\n'+
			f'\trule: if snr is acceptable then sf is {sf[int(position[31])]}\n'+
			f'\trule: if snr is good then sf is {sf[int(position[32])]}\n'
		)

def execute(run):
	ns3 = '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/./ns3'
	cmd = f"{ns3} run \"scenario102  --adrType=ns3::AdrFuzzy --nDevices=136 --intervalTx=15 --nRun={run}\""
	os.system(cmd)

def simulate(positions):
	fillFll(positions)
	executor = concurrent.futures.ThreadPoolExecutor()
	results = [executor.submit(execute, i) for i in range(1, 5)]
	concurrent.futures.wait(results)
	executor.shutdown()

def ADR(particle):
	particle['Cost'] = particle['PLR'] + particle['Energy']

def dump(pso, filename='pso.txt'):
	with open(f'/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/scratch/pso-fuzzy/{filename}', 'w') as file:
		file.write(str(pso))

def restore(filename='pso.txt'):
	with open(f'/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/scratch/pso-fuzzy/{filename}', 'r') as file:
		return file.read()

def generate_random_coords(min_val, max_val):
	coords = np.zeros(9)
	
	coords[0] = min_val
	coords[8] = max_val
	
	coords[1:3] = np.random.uniform(coords[0], max_val, 2)
	coords[1:3].sort()

	coords[3] = (coords[0] + coords[2]) / 2
	coords[4:6] = np.random.uniform(coords[3], max_val, 2)
	coords[4:6].sort()

	coords[6] = (coords[3]+coords[5])/2
	coords[7] = np.random.uniform(coords[6], max_val, 1)
	
	return coords

def generate_max_min():
	VarMin = [-6.0 for _ in range(9)]
	VarMin += [2.0 for _ in range(9)]
	VarMin += [7 for _ in range(9)]
	VarMin += [0 for _ in range(6)]

	VarMax = [30.0 for _ in range(9)]
	VarMax += [14.0 for _ in range(9)]
	VarMax += [12 for _ in range(9)]
	VarMax += [2 for _ in range(6)]

	return { 'VarMin':  VarMin, 'VarMax': VarMax }

def init_pos(VarMin, VarMax):
	SNR_coords = generate_random_coords(VarMin[0], VarMax[0])
	TP_coords = generate_random_coords(VarMin[9], VarMax[9])
	SF_coords = generate_random_coords(VarMin[19], VarMax[19])

	SNR_TP = np.random.randint(VarMin[27], VarMax[27]+1, 3)
	SNR_SF = np.random.randint(VarMin[30], VarMax[30]+1, 3)

	positions = np.concatenate((SNR_coords, TP_coords, SF_coords, SNR_TP, SNR_SF), axis=0)
	return positions

def save_fig(filename, BestCosts, title): 
	path = '/home/thiago/Documentos/Doutorado/Simuladores/ns-3-allinone/ns-3.38/scratch/pso-fuzzy'
	x = range(1, len(BestCosts)+1)
	dpi = 300
	# Resultados
	plt.figure()
	plt.semilogy(x, BestCosts, linewidth=2)
	plt.xlabel('Iteration')
	plt.ylabel('Fitness')
	plt.title(title)
	plt.grid(True)
	plt.scatter(x, BestCosts, color='red')
	for i in range(len(BestCosts)):
		plt.text(x[i], BestCosts[i], str(round(BestCosts[i], 2)), ha='center', va='bottom')
	plt.xticks(np.arange(0, len(BestCosts)+1))
	plt.savefig (f'{path}/semilogy-{filename}', dpi=dpi) # Salvar Gráfico

	plt.figure()
	plt.plot(x, BestCosts)
	plt.xlabel('Iteration')
	plt.ylabel('Fitness')
	plt.title(title)
	plt.grid(True)
	plt.scatter(x, BestCosts, color='red')
	for i in range(len(BestCosts)):
		plt.text(x[i], BestCosts[i], str(round(BestCosts[i], 2)), ha='center', va='bottom')
	plt.xticks(np.arange(0, len(BestCosts)+1))
	plt.savefig (f'{path}/curve-{filename}', dpi=dpi) # Salvar Gráfico

def apply_lim_vel(velocities, min_val, max_val):
	size = len(velocities)
	for i in range(0, size):
		if velocities[i] < min_val[i]:
			velocities[i] = min_val[i]
		if velocities[i] > max_val[i]:
			velocities[i] = max_val[i]

def apply_lim_pos_vars(p, newP, min_val, max_val):
	p[0] = min_val
	p[8] = max_val

	p[1] = max(min_val, newP)
	p[1] = min(p[2], newP) 

	p[2] = max(p[1], newP)
	p[2] = min(max_val, newP)

	p[3] = max(min_val, newP)
	p[3] = min(p[4], newP)

	p[4] = max(p[3], newP)
	p[4] = min(p[5], newP)

	p[5] = max(p[4], newP)
	p[5] = min(max_val, newP)

	p[6] = max(min_val, newP)
	p[6] = max(p[7], newP)

	p[7] = max(p[6], newP)
	p[7] = max(max_val, newP)

def apply_lim_pos_rules(p, newP, min_val, max_val):
	p = max(min_val, newP)
	p = min(max_val, newP)

def PSO(problem, params):
	# Extração dos parâmetros do problema e do PSO
	CostFunction = problem['CostFunction']
	nVar = problem['nVar']
	VarMin = problem['VarMin']
	VarMax = problem['VarMax']
	
	MaxIt = params['MaxIt']
	nPop = params['nPop']
	w = params['w']
	wdamp = params['wdamp']
	c1 = params['c1']
	c2 = params['c2']
	ShowIterInfo = params['ShowIterInfo']
	
	MaxVel = 0.2 * (VarMax - VarMin)
	MinVel = -MaxVel
	
	# Inicialização
	particle = np.empty(nPop, dtype=object)
	GlobalBest = {'Position': None, 'Cost': np.inf}
	BestCosts = np.zeros(MaxIt)
	
	for i in range(nPop):
		particle[i] = {'Position': init_pos(VarMin, VarMax),
									 'Velocity': np.zeros(nVar),
									 'Cost': None,
									 'Best': {'Position': None, 'Cost': np.inf},
									 'PLR': np.inf,
									 'Energy': np.inf}

		# Avaliar partícula
		result = simulate(particle[i]['Position'])
		result.await_output()
		result = calc(particle[i])
		result.await_output()
		CostFunction(particle[i])
		# *****************************************************************************************************************
		
		particle[i]['Best']['Position'] = particle[i]['Position']
		particle[i]['Best']['Cost'] = particle[i]['Cost']
		
		if particle[i]['Best']['Cost'] < GlobalBest['Cost']:
			GlobalBest = particle[i]['Best']

		save_fig('pso-fuzzy0.png')

	# Loop principal
	for it in range(MaxIt):
		if GlobalBest['Cost'] == 0:
			break

		if it % 10 == 0:
			out = {'pop': particle, 'BestSol': GlobalBest, 'BestCosts': BestCosts}
			dump(out)
		
		for i in range(nPop):
			# Atualizar velocidade
			particle[i]['Velocity'] = (w * particle[i]['Velocity'] +
																c1 * np.random.rand(nVar) * (particle[i]['Best']['Position'] - particle[i]['Position']) +
																c2 * np.random.rand(nVar) * (GlobalBest['Position'] - particle[i]['Position']))

			apply_lim_vel(particle[i]['Velocities'], MinVel, MaxVel)
			# *****************************************************************************************************************

			# Atualizar posições
			newPositions = particle[i]['Position'] + particle[i]['Velocity']
			
			apply_lim_pos_vars(particle[i]['Position'][0:9], newPositions[0:9], VarMin[0], VarMax[0]) # SNR
			apply_lim_pos_vars(particle[i]['Position'][9:18], newPositions[9:18], VarMin[9], VarMax[9]) # TP
			apply_lim_pos_vars(particle[i]['Position'][18:27], newPositions[18:27], VarMin[18], VarMax[9]) # SF

			apply_lim_pos_rules(particle[i]['Positions'][27:33], newPositions[27:33], VarMin[27:33], VarMax[27:33])	# rules		
			# *****************************************************************************************************************

			# Avaliar partícula
			result = simulate(particle[i]['Position'])
			result.await_output()
			result = calc(particle[i])
			result.await_output()
			CostFunction(particle[i])
			# *****************************************************************************************************************
			
			# Atualizar BestPosition e GlobalBest						
			if particle[i]['Cost'] < particle[i]['Best']['Cost']:
				particle[i]['Best']['Position'] = particle[i]['Position']
				particle[i]['Best']['Cost'] = particle[i]['Cost']
								
				if particle[i]['Best']['Cost'] < GlobalBest['Cost']:
					GlobalBest = particle[i]['Best']
			# *****************************************************************************************************************
				
		BestCosts[it] = GlobalBest['Cost']
		w *= wdamp
			
		if ShowIterInfo:
			print(f'Iteration {it + 1}: Best Cost = {BestCosts[it]}')

		save_fig(f'pso-fuzzy{it+1}.png', BestCosts)
					
	out = {'pop': particle, 'BestSol': GlobalBest, 'BestCosts': BestCosts}
	dump(out)
	return out

"""
if __name__ == "__main__":
	max_min = generate_max_min()

	problem = {
		'CostFunction': ADR, 
		'nVar': 33, 
		'VarMin': np.array(max_min['VarMin']), 
		'VarMax': np.array(max_min['VarMax'])
	}

	params = {
		'MaxIt': 100, 
		'nPop': 200, 
		'w': 0.7298, 
		'wdamp': 1, 
		'c1': 1.4962, 
		'c2': 1.4962, 
		'ShowIterInfo': True
	}

	# Chamada do PSO
	out = PSO(problem, params)
	
	BestSol = out['BestSol']
	BestCosts = out['BestCosts']

	dump(BestSol, 'best-sol-final.json')
	save_fig('pso-fuzzy-final.png', BestCosts)
"""

## Testes Unitário
def testCalcPlr(): # Ok
	particle = {'Position': np.random.uniform(-6, 30, 33),
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}
	
	calcPlr(particle)
	print(particle['PLR'])

def testCalcEnergy(): # Ok
	particle = {'Position': np.random.uniform(0, 10, 33),
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}
	
	calcEnergy(particle)
	print(particle['Energy'])

def testCalc(): # Ok
	particle = {'Position': np.random.uniform(0, 10, 33),
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}
	
	calc(particle)
	print(particle['PLR'], particle['Energy'])

def testFill(): # Ok
	pos = np.concatenate([np.random.uniform(-6, 30, 27), np.random.randint(0, 3, 6)], axis=0)
	particle = {'Position': pos,
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}
	fillFll(particle['Position'], fll='fadr01.fll')

def testExecute(): # Ok
	executor = concurrent.futures.ThreadPoolExecutor()
	executor.submit(execute, 1)
	executor.shutdown(wait=True)

def testSimulate(): # Ok
	pos = np.concatenate([np.random.uniform(-6, 30, 27), np.random.randint(0, 3, 6)], axis=0)
	simulate(pos)

def testAdr(): # Ok
	particle = {'Position': np.random.uniform(0, 10, 33),
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}
	
	calc(particle)
	print(particle['PLR'], particle['Energy'])
	ADR(particle)
	print(particle['Cost'])

def testDump(): # Ok
	particle = {'Position': np.random.uniform(0, 10, 33),
		'Velocity': np.zeros(33),
		'Cost': None,
		'Best': {'Position': None, 'Cost': np.inf},
		'PLR': np.inf,
		'Energy': np.inf}

	calc(particle)
	print(particle['PLR'], particle['Energy'])
	ADR(particle)
	print(particle['Cost'])

	dump(particle, 'particle-test.txt')

def testRestore(): #  Ok
	data = restore(filename='particle-test.txtx')
	print(data)

def testGenRndCoords(): # Ok
	SNR_coords = generate_random_coords(-6, 30)
	print(SNR_coords)

def testMaxMin(): # Ok
	max_min = generate_max_min()
	print(max_min)

def testInitPos(): # Ok
	max_min = generate_max_min()
	coords = init_pos(max_min['VarMin'], max_min['VarMax'])
	print(coords)

def testSaveFig():
	save_fig('save-fig-test.png', np.random.uniform(0, 2, 20), 'Best Costs')

if __name__ == '__main__':
	# testCalcPlr() # Ok
	# testCalcEnergy() # Ok
	# testCalc() # Ok
	# testFill() # Ok
	# testExecute() # Ok
	# testSimulate() # Ok
	# testAdr() # Ok
	# testDump() # Ok
	# testRestore() # Ok
	# testGenRndCoords() Ok
	# testMaxMin() # Ok
	# testInitPos() # Ok
	testSaveFig() # Ok